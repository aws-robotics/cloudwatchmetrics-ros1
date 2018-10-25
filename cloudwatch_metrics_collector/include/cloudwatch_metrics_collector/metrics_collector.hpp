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

#pragma once

#include <aws_common/sdk_utils/logging/aws_log_system.h>
#include <ros/ros.h>
#include <ros_monitoring_msgs/MetricList.h>

#include <cloudwatch_metrics_common/metric_manager.hpp>

namespace Aws {
namespace CloudWatch {
namespace Metrics {

class MetricsCollector
{
public:
  static const std::string kNodeParamMonitorTopicsListKey;
  static const std::string kNodeParamMetricNamespaceKey;
  static const std::string kNodeParamDefaultMetricDimensionsKey;
  static const std::string kNodeName;
  static const int kNodeSubQueueSize;
  static const int kNodeMetricManagerServiceTimeSec;
  static const std::string kNodeDefaultMetricNamespace;
  static const std::string kNodeDefaulMetricsTopic;
  static const int kNodeDefaultMetricDatumStorageResolution;
  static const std::string kNodeParamMetricDatumStorageResolutionKey;

  explicit MetricsCollector(std::shared_ptr<MetricManager> metric_manager,
                            std::map<std::string, std::string> && default_dimensions);

  void ReceiveMetricCallback(const ros_monitoring_msgs::MetricList::ConstPtr & metric_list_msg);

  void ServiceMetricManagerCallback(const ros::TimerEvent &);

  /**
   *  Subscribes to all the monitoring topics from the parameter service and saves the references in
   * the provided vector
   */
  Aws::AwsError SubscribeAllTopics(ros::NodeHandle & nh);

  /**
   * Parses a default list of metric dimensions from the parameters. The dimensions will be in the
   * format:
   *      "<dimension1>:<value1>;<dimension2>:<value2>;<dimension3>:<value3>"
   */
  Aws::AwsError ParseDefaultDimensions();

  Aws::AwsError Initialize(ros::NodeHandle & nh);

  static MetricsCollector Build(Aws::AwsError & status);

private:
  std::shared_ptr<MetricManager> metric_manager_ = nullptr;
  std::unique_ptr<ros::Timer> timer_ = nullptr;
  std::vector<ros::Subscriber> subscriptions_;
  std::map<std::string, std::string> default_dimensions_;

  static const std::set<int> kNodeParamMetricDatumStorageResolutionValidValues;
};

}  // namespace Metrics
}  // namespace CloudWatch
}  // namespace Aws
