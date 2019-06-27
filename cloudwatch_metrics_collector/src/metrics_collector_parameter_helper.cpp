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

#include <aws_common/sdk_utils/aws_error.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <ros/ros.h>
#include <unordered_set>
#include <iostream>
#include <aws/core/utils/logging/LogMacros.h>
#include <cloudwatch_metrics_collector/metrics_collector_parameter_helper.hpp>

using namespace Aws::Client;

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
        double & publish_frequency) {

  Aws::AwsError ret =
          parameter_reader->ReadParam(ParameterPath(kNodeParamPublishFrequencyKey), publish_frequency);

  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      publish_frequency = kNodePublishFrequencyDefaultValue;
      AWS_LOGSTREAM_WARN(__func__,
                         "Publish frequency configuration not found, setting to default value: "
                                 << kNodePublishFrequencyDefaultValue);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, "Publish frequency is set to: " << publish_frequency);
      break;
    default:
      publish_frequency = kNodePublishFrequencyDefaultValue;
      AWS_LOGSTREAM_ERROR(__func__,
                          "Error " << ret << " retrieving publish frequency, setting to default value: "
                                   << kNodePublishFrequencyDefaultValue);

  }
}

/**
 *
 * @param parameter_reader
 * @param metric_namespace
 * @return
 */
void ReadMetricNamespace(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        std::string & metric_namespace) {

  // Load the metric namespace
  Aws::AwsError read_namespace_status =
          parameter_reader->ReadParam(ParameterPath(kNodeParamMetricNamespaceKey), metric_namespace);
  if (Aws::AWS_ERR_OK == read_namespace_status) {
    AWS_LOGSTREAM_INFO(__func__, "Namespace: " << metric_namespace);
  } else {
    AWS_LOGSTREAM_WARN(
            __func__,
            "No namespace configuration found. Falling back to default namespace: " << metric_namespace);
    metric_namespace = kNodeDefaultMetricNamespace;
  }
}

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
        std::map<std::string, std::string> & metric_dims) {

  // Load the default dimensions
  Aws::AwsError read_dimensions_status =
          parameter_reader->ReadParam(ParameterPath(kNodeParamDefaultMetricDimensionsKey), dimensions_param);

  Aws::OStringStream logging_stream;
  logging_stream << "Default Metric Dimensions: { ";
  if (Aws::AWS_ERR_OK == read_dimensions_status) {
    auto dims = Aws::Utils::StringUtils::Split(dimensions_param, ';');
    for (auto it = dims.begin(); it != dims.end(); ++it) {
      if (!it->empty()) {
        auto dim_vec = Aws::Utils::StringUtils::Split(*it, ':');
        if (dim_vec.size() == 2) {
          metric_dims.emplace(dim_vec[0].c_str(), dim_vec[1].c_str());
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
}

/**
 *
 * @param parameter_reader
 * @param storage_resolution
 * @return
 */
void ReadStorageResolution(
        std::shared_ptr<Aws::Client::ParameterReaderInterface> parameter_reader,
        int & storage_resolution) {

  // Load the storage resolution
  storage_resolution = kNodeDefaultMetricDatumStorageResolution;

  Aws::AwsError read_storage_resolution_status =
          parameter_reader->ReadParam(ParameterPath(kNodeParamMetricDatumStorageResolutionKey), storage_resolution);

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
}

void ReadTopics(std::vector<std::string> & topics) {

  std::string param_key;
  if (ros::param::search(kNodeParamMonitorTopicsListKey, param_key)) {
    ros::param::get(param_key, topics);
  }
  if (topics.empty()) {
    AWS_LOGSTREAM_INFO(
            __func__, "Topic list not defined or empty. Listening on topic: " << kNodeDefaulMetricsTopic);
    topics.push_back(kNodeDefaulMetricsTopic);
  }
}


}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
