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
#include <ros_monitoring_msgs/MetricData.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>

#include <string>
#include <map>

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

class MetricsCollector : public Service
{
public:

  MetricsCollector() = default;
  ~MetricsCollector() = default;

  /**
   * Accept input metric message to be batched for publishing.
   *
   * @param metric_list_msg
   * @return the number of metrics successfully batched
   */
  int RecordMetrics(const ros_monitoring_msgs::MetricList::ConstPtr & metric_list_msg);

  /**
   * Force all batched data to be published to CloudWatch.
   */
  void TriggerPublish(const ros::TimerEvent &);

  /**
   * Initialize the MetricsCollector with parameters read from the config file.
   *
   * @param metric_namespace
   * @param default_dimensions
   * @param storage_resolution
   * @param config
   * @param sdk_options
   * @param metric_service_factory
   */
  void Initialize(std::string metric_namespace,
                  std::map<std::string, std::string> & default_dimensions,
                  int storage_resolution,
                  ros::NodeHandle node_handle,
                  const Aws::Client::ClientConfiguration & config,
                  const Aws::SDKOptions & sdk_options,
                  const Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options,
                  std::shared_ptr<MetricServiceFactory> metric_service_factory = std::make_shared<MetricServiceFactory>());

  void SubscribeAllTopics();

  bool start() override;
  bool shutdown() override;

  /**
   * Return a Trigger response detailing the MetricService online status.
   *
   * @param request input request
   * @param response output response
   * @return true if the request was handled successfully, false otherwise
   */
  bool checkIfOnline(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

protected:

  /**
   * Gets the timestamp for the input metric message as milliseconds since epoch
   */
  static int64_t GetMetricDataEpochMillis(const ros_monitoring_msgs::MetricData & metric_msg);

private:

  std::string metric_namespace_;
  std::map<std::string, std::string> default_dimensions_;
  std::atomic<int> storage_resolution_;
  std::shared_ptr<MetricService> metric_service_;
  std::vector<ros::Subscriber> subscriptions_;
  ros::NodeHandle node_handle_;
  std::vector<std::string> topics_;
};

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
