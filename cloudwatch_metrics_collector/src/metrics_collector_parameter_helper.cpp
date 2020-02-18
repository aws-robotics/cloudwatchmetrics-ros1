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
#include <cloudwatch_metrics_common/cloudwatch_options.h>

using Aws::Client::ParameterPath;


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
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
        double & publish_frequency) {

  Aws::AwsError ret =
          parameter_reader->ReadParam(ParameterPath(kNodeParamPublishFrequencyKey), publish_frequency);

  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      publish_frequency = kNodePublishFrequencyDefaultValue;
      AWS_LOGSTREAM_INFO(__func__,
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
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
        std::string & metric_namespace) {

  // Load the metric namespace
  Aws::AwsError read_namespace_status =
          parameter_reader->ReadParam(ParameterPath(kNodeParamMetricNamespaceKey), metric_namespace);
  if (Aws::AWS_ERR_OK == read_namespace_status) {
    AWS_LOGSTREAM_INFO(__func__, "Namespace: " << metric_namespace);
  } else {
    AWS_LOGSTREAM_INFO(
            __func__,
            "No namespace configuration found. Falling back to default namespace: " << kNodeDefaultMetricNamespace);
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
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
        Aws::String & dimensions_param,
        std::map<std::string, std::string> & metric_dims) {

  // Load the default dimensions
  Aws::AwsError read_dimensions_status =
          parameter_reader->ReadParam(ParameterPath(kNodeParamDefaultMetricDimensionsKey), dimensions_param);

  Aws::OStringStream logging_stream;
  logging_stream << "Default Metric Dimensions: { ";
  if (Aws::AWS_ERR_OK == read_dimensions_status) {
    auto dims = Aws::Utils::StringUtils::Split(dimensions_param, ';');
    for (auto & dim : dims) {
      if (!dim.empty()) {
        auto dim_vec = Aws::Utils::StringUtils::Split(dim, ':');
        if (dim_vec.size() == 2) {
          metric_dims.emplace(dim_vec[0].c_str(), dim_vec[1].c_str());
          logging_stream << dim_vec[0] << ": " << dim_vec[1] << ", ";
        } else {
          AWS_LOGSTREAM_WARN(
                  __func__, "Could not parse dimension: "
                          << dim << ". Should be in the format <DimensionName>:<DimensionValue>");
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
        const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
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
    AWS_LOGSTREAM_INFO(
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

void ReadCloudWatchOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  Aws::CloudWatchMetrics::CloudWatchOptions & cloudwatch_options) {

  Aws::DataFlow::UploaderOptions uploader_options{};
  Aws::FileManagement::FileManagerStrategyOptions file_manager_strategy_options;

  ReadUploaderOptions(parameter_reader, uploader_options);
  ReadFileManagerStrategyOptions(parameter_reader, file_manager_strategy_options);

  cloudwatch_options = {
    uploader_options,
    file_manager_strategy_options
  };
}

void ReadUploaderOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  Aws::DataFlow::UploaderOptions & uploader_options) {

  ReadOption(
    parameter_reader,
    kNodeParamFileUploadBatchSize,
    Aws::DataFlow::kDefaultUploaderOptions.file_upload_batch_size,
    uploader_options.file_upload_batch_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamFileMaxQueueSize,
    Aws::DataFlow::kDefaultUploaderOptions.file_max_queue_size,
    uploader_options.file_max_queue_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamBatchMaxQueueSize,
    Aws::DataFlow::kDefaultUploaderOptions.batch_max_queue_size,
    uploader_options.batch_max_queue_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamBatchTriggerPublishSize,
    Aws::DataFlow::kDefaultUploaderOptions.batch_trigger_publish_size,
    uploader_options.batch_trigger_publish_size
  );

  ReadOption(
    parameter_reader,
    kNodeParamStreamMaxQueueSize,
    Aws::DataFlow::kDefaultUploaderOptions.stream_max_queue_size,
    uploader_options.stream_max_queue_size
  );
}

void ReadFileManagerStrategyOptions(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  Aws::FileManagement::FileManagerStrategyOptions & file_manager_strategy_options) {

  ReadOption(
    parameter_reader,
    kNodeParamStorageDirectory,
    Aws::CloudWatchMetrics::kDefaultMetricFileManagerStrategyOptions.storage_directory,
    file_manager_strategy_options.storage_directory);

  ReadOption(
    parameter_reader,
    kNodeParamFilePrefix,
    Aws::CloudWatchMetrics::kDefaultMetricFileManagerStrategyOptions.file_prefix,
    file_manager_strategy_options.file_prefix);

  ReadOption(
    parameter_reader,
    kNodeParamFileExtension,
    Aws::CloudWatchMetrics::kDefaultMetricFileManagerStrategyOptions.file_extension,
    file_manager_strategy_options.file_extension);

  ReadOption(
    parameter_reader,
    kNodeParamMaximumFileSize,
    Aws::CloudWatchMetrics::kDefaultMetricFileManagerStrategyOptions.maximum_file_size_in_kb,
    file_manager_strategy_options.maximum_file_size_in_kb);

  ReadOption(
    parameter_reader,
    kNodeParamStorageLimit,
    Aws::CloudWatchMetrics::kDefaultMetricFileManagerStrategyOptions.storage_limit_in_kb,
    file_manager_strategy_options.storage_limit_in_kb);
}

void ReadOption(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  const std::string & option_key,
  const std::string & default_value,
  std::string & option_value) {
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(option_key), option_value);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      option_value = default_value;
      AWS_LOGSTREAM_INFO(__func__,
                         option_key << " parameter not found, setting to default value: " << default_value);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      AWS_LOGSTREAM_INFO(__func__, option_key << " is set to: " << option_value);
      break;
    default:
      option_value = default_value;
      AWS_LOGSTREAM_ERROR(__func__,
        "Error " << ret << " retrieving option " << option_key << ", setting to default value: " << default_value);
  }
}

void ReadOption(
  const std::shared_ptr<Aws::Client::ParameterReaderInterface>& parameter_reader,
  const std::string & option_key,
  const size_t & default_value,
  size_t & option_value) {

  int param_value = 0;
  Aws::AwsError ret = parameter_reader->ReadParam(ParameterPath(option_key), param_value);
  switch (ret) {
    case Aws::AwsError::AWS_ERR_NOT_FOUND:
      option_value = default_value;
      AWS_LOGSTREAM_INFO(__func__,
                         option_key << " parameter not found, setting to default value: " << default_value);
      break;
    case Aws::AwsError::AWS_ERR_OK:
      option_value = static_cast<size_t>(param_value);
      AWS_LOGSTREAM_INFO(__func__, option_key << " is set to: " << option_value);
      break;
    default:
      option_value = default_value;
      AWS_LOGSTREAM_ERROR(__func__,
        "Error " << ret << " retrieving option " << option_key << ", setting to default value: " << default_value);
  }
}

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
