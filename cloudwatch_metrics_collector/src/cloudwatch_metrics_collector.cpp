/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#include <aws/core/Aws.h>
#include <aws/core/client/ClientConfiguration.h>
#include <aws/core/utils/StringUtils.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/monitoring/CloudWatchClient.h>
#include <aws/monitoring/model/PutMetricDataRequest.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/logging/aws_ros_logger.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <ros/ros.h>
#include <ros_monitoring_msgs/MetricData.h>
#include <ros_monitoring_msgs/MetricDimension.h>
#include <ros_monitoring_msgs/MetricList.h>
#include <std_msgs/String.h>

#include <cloudwatch_metrics_collector/metrics_collector.hpp>
#include <cloudwatch_metrics_common/metric_manager.hpp>
#include <cloudwatch_metrics_common/metric_manager_factory.hpp>
#include <string>
#include <vector>

using namespace Aws::CloudWatch::Metrics;
using namespace Aws::Utils::Logging;

const std::string MetricsCollector::kNodeParamMonitorTopicsListKey = "aws_monitored_metric_topics";
const std::string MetricsCollector::kNodeParamMetricNamespaceKey = "aws_metrics_namespace";
const std::string MetricsCollector::kNodeParamDefaultMetricDimensionsKey =
  "aws_default_metric_dimensions";
const std::string MetricsCollector::kNodeName = "cloudwatch_metrics_collector";
const int MetricsCollector::kNodeSubQueueSize = 100;
const int MetricsCollector::kNodeMetricManagerServiceTimeSec = 1;
const std::string MetricsCollector::kNodeDefaultMetricNamespace = "ROS";
const std::string MetricsCollector::kNodeDefaulMetricsTopic = "metrics";
const int MetricsCollector::kNodeDefaultMetricDatumStorageResolution = 60;
const std::string MetricsCollector::kNodeParamMetricDatumStorageResolutionKey =
  "storage_resolution";
const std::set<int> MetricsCollector::kNodeParamMetricDatumStorageResolutionValidValues = {1, 60};

MetricsCollector::MetricsCollector(std::shared_ptr<MetricManager> metric_manager,
                                   std::map<std::string, std::string> && default_dimensions)
: metric_manager_(metric_manager), default_dimensions_(default_dimensions)
{
}

void MetricsCollector::ReceiveMetricCallback(
  const ros_monitoring_msgs::MetricList::ConstPtr & metric_list_msg)
{
  AWS_LOGSTREAM_DEBUG(__func__, "Received " << metric_list_msg->metrics.size() << " metrics");

  for (auto metric_msg = metric_list_msg->metrics.begin();
       metric_msg != metric_list_msg->metrics.end(); ++metric_msg) {
    int64_t timestamp =
      ((int64_t)metric_msg->time_stamp.sec * 1000) + ((int64_t)metric_msg->time_stamp.nsec / 1000);
    std::map<std::string, std::string> dimensions;
    Aws::AwsError status = Aws::AWS_ERR_OK;
    for (auto it = default_dimensions_.begin(); it != default_dimensions_.end(); ++it) {
      dimensions.emplace(it->first, it->second);  // ignore the return, if we get a duplicate we're
                                                  // going to stick with the first one
    }
    for (auto it = metric_msg->dimensions.begin(); it != metric_msg->dimensions.end(); ++it) {
      dimensions.emplace((*it).name, (*it).value);  // ignore the return, if we get a duplicate
                                                    // we're going to stick with the first one
    }
    AWS_LOGSTREAM_DEBUG(__func__, "Recording metric with name=[" << metric_msg->metric_name << "]");
    status = metric_manager_->RecordMetric(metric_msg->metric_name, metric_msg->value,
                                           metric_msg->unit, timestamp, dimensions);
    if (Aws::AWS_ERR_OK != status) {
      AWS_LOGSTREAM_ERROR(__func__, "Failed to record metric: " << status);
    }
  }
}

void MetricsCollector::ServiceMetricManagerCallback(const ros::TimerEvent &)
{
  AWS_LOG_DEBUG(__func__, "Flushing metrics");
  metric_manager_->Service();
}

Aws::AwsError MetricsCollector::SubscribeAllTopics(ros::NodeHandle & nh)
{
  Aws::AwsError status = Aws::AWS_ERR_OK;
  std::vector<std::string> topics;
  std::string param_key;
  if (ros::param::search(MetricsCollector::kNodeParamMonitorTopicsListKey, param_key)) {
    ros::param::get(param_key, topics);
  }
  if (topics.empty()) {
    AWS_LOGSTREAM_INFO(
      __func__, "Topic list not defined or empty. Listening on topic: " << kNodeDefaulMetricsTopic);
    topics.push_back(kNodeDefaulMetricsTopic);
  }

  for (auto it = topics.begin(); it != topics.end(); ++it) {
    ros::Subscriber sub = nh.subscribe<ros_monitoring_msgs::MetricList>(
      *it, kNodeSubQueueSize,
      [this](const ros_monitoring_msgs::MetricList::ConstPtr & metric_list_msg) -> void {
        this->ReceiveMetricCallback(metric_list_msg);
      });
    subscriptions_.push_back(sub);
  }
  return status;
}

MetricsCollector MetricsCollector::Build(Aws::AwsError & status)
{
  auto param_reader = std::make_shared<Aws::Client::Ros1NodeParameterReader>();
  Aws::Client::ClientConfigurationProvider client_config_provider(param_reader);
  Aws::Client::ClientConfiguration client_config = client_config_provider.GetClientConfiguration();

  Aws::SDKOptions sdk_options;
  MetricManagerFactory mm_factory;
  std::string metric_namespace(kNodeDefaultMetricNamespace);
  Aws::String dimensions_param;
  std::map<std::string, std::string> default_metric_dims;

  // Load the metric namespace
  Aws::AwsError read_namespace_status =
    param_reader->ReadStdString(kNodeParamMetricNamespaceKey.c_str(), metric_namespace);
  if (Aws::AWS_ERR_OK == read_namespace_status) {
    AWS_LOGSTREAM_INFO(__func__, "Namespace: " << metric_namespace);
  } else {
    AWS_LOGSTREAM_WARN(
      __func__,
      "No namespace configuration found. Falling back to default namespace: " << metric_namespace);
  }

  // Load the storage resolution
  int storage_resolution = kNodeDefaultMetricDatumStorageResolution;
  Aws::AwsError read_storage_resolution_status =
    param_reader->ReadInt(kNodeParamMetricDatumStorageResolutionKey.c_str(), storage_resolution);
  if (Aws::AWS_ERR_OK == read_storage_resolution_status) {
    if (kNodeParamMetricDatumStorageResolutionValidValues.find(storage_resolution) ==
        kNodeParamMetricDatumStorageResolutionValidValues.end()) {
      AWS_LOGSTREAM_WARN(__func__,
                         "Storage Resolution value of ["
                           << storage_resolution
                           << "] is not allowed. Falling back to default Storage Resolution: "
                           << kNodeDefaultMetricDatumStorageResolution);
      storage_resolution = kNodeDefaultMetricDatumStorageResolution;
    } else {
      AWS_LOGSTREAM_INFO(__func__, "Storage Resolution: " << storage_resolution);
    }
  } else {
    AWS_LOGSTREAM_WARN(
      __func__,
      "No Storage Resolution configuration found. Falling back to default Storage Resolution: "
        << storage_resolution);
  }

  // Load the default dimensions
  Aws::AwsError read_dimensions_status =
    param_reader->ReadString(kNodeParamDefaultMetricDimensionsKey.c_str(), dimensions_param);
  Aws::OStringStream logging_stream;
  logging_stream << "Default Metric Dimensions: { ";
  if (Aws::AWS_ERR_OK == read_dimensions_status) {
    auto dims = Aws::Utils::StringUtils::Split(dimensions_param, ';');
    for (auto it = dims.begin(); it != dims.end(); ++it) {
      if (!it->empty()) {
        auto dim_vec = Aws::Utils::StringUtils::Split(*it, ':');
        if (dim_vec.size() == 2) {
          default_metric_dims.emplace(dim_vec[0].c_str(), dim_vec[1].c_str());
          logging_stream << dim_vec[0] << ": " << dim_vec[1] << ", ";
        } else {
          AWS_LOGSTREAM_WARN(
            __func__, "Could not parse dimension: "
                        << *it << ". Should be in the format <DimensionName>:<DimensionValue>");
        }
      }
    }
  }
  logging_stream << " }";
  AWS_LOGSTREAM_INFO(__func__, logging_stream.str());

  status = Aws::AWS_ERR_OK;
  return MetricsCollector(mm_factory.CreateMetricManager(client_config, sdk_options,
                                                         metric_namespace, storage_resolution),
                          std::move(default_metric_dims));
}

Aws::AwsError MetricsCollector::Initialize(ros::NodeHandle & nh)
{
  Aws::AwsError status = SubscribeAllTopics(nh);

  if (Aws::AWS_ERR_OK == status) {
    this->timer_ =
      std::make_unique<ros::Timer>(nh.createTimer(ros::Duration(kNodeMetricManagerServiceTimeSec),
                                                  [this](const ros::TimerEvent & event) -> void {
                                                    this->ServiceMetricManagerCallback(event);
                                                  }));
  }

  return status;
}

int main(int argc, char * argv[])
{
  ros::init(argc, argv, MetricsCollector::kNodeName);

  ros::NodeHandle nh;

  Aws::Utils::Logging::InitializeAWSLogging(
    Aws::MakeShared<Aws::Utils::Logging::AWSROSLogger>(MetricsCollector::kNodeName.c_str()));
  Aws::AwsError status;

  MetricsCollector metrics_collector = MetricsCollector::Build(status);
  if (Aws::AWS_ERR_OK == status) {
    status = metrics_collector.Initialize(nh);
  }

  if (Aws::AWS_ERR_OK == status) {
    AWS_LOG_INFO(__func__, "Starting Metrics Collector ...");
    ros::spin();
  }

  AWS_LOG_INFO(__func__, "Shutting down Metrics Collector ...");
  Aws::Utils::Logging::ShutdownAWSLogging();
  ros::shutdown();
  return status;
}
