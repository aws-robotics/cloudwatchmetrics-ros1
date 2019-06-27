/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <ros/ros.h>
#include <unordered_set>
#include <iostream>

using namespace Aws::Client;

const std::string kNodeParamMonitorTopicsListKey("aws_monitored_metric_topics");
const std::string kNodeParamMetricNamespaceKey = "aws_metrics_namespace";
const std::string kNodeParamDefaultMetricDimensionsKey  = "aws_default_metric_dimensions";
const std::string kNodeName = "cloudwatch_metrics_collector";
const char kNodeParamPublishFrequencyKey[] = "publish_frequency";

constexpr int kNodeSubQueueSize = 100;
constexpr int kNodePublishFrequencyDefaultValue = 10;
const int kNodeMetricServiceTimeSec = 1;
const std::string kNodeDefaultMetricNamespace = "ROS";
const std::string kNodeDefaulMetricsTopic = "metrics";
constexpr int kNodeDefaultMetricDatumStorageResolution = 60;
const std::string kNodeParamMetricDatumStorageResolutionKey = "storage_resolution";
const std::set<int> kNodeParamMetricDatumStorageResolutionValidValues = {1, 60};


namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

/**
 * Fetch the parameter for the log publishing frequency.
 *
 * @param parameter_reader to retrieve the parameters from.
 * @param publish_frequency the parameter is stored here when it is read successfully.
 * @return an error code that indicates whether the parameter was read successfully or not,
 * as returned by \p parameter_reader
 */
void ReadPublishFrequency(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        double & publish_frequency);

/**
 *
 * @param parameter_reader
 * @param metric_namespace
 * @return
 */
void ReadMetricNamespace(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        std::string & metric_namespace);

/**
 *
 * @param parameter_reader
 * @param dimensions_param
 * @param metric_dims
 * @return
 */
void ReadMetricDimensions(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        Aws::String & dimensions_param,
        std::map<std::string, std::string> & metric_dims);
/**
 *
 * @param parameter_reader
 * @param storage_resolution
 * @return
 */
void ReadStorageResolution(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        int & storage_resolution);

void ReadTopics(std::vector<std::string> & topics);


}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
