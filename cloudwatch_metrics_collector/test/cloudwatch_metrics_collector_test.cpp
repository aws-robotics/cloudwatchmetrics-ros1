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
#include <aws/core/utils/logging/LogMacros.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <cloudwatch_metrics_collector/metrics_collector.hpp>
#include <cloudwatch_metrics_common/metric_manager.hpp>

using namespace Aws::CloudWatch::Metrics;

class MetricManagerMock : public MetricManager
{
public:
  MetricManagerMock() : MetricManager(nullptr, 0){};

  MOCK_METHOD0(Service, Aws::AwsError());
  MOCK_METHOD5(RecordMetric, Aws::AwsError(const std::string &, double, const std::string &,
                                           int64_t, const std::map<std::string, std::string> &));
};

class MetricsCollectorFixture : public ::testing::Test
{
protected:
  const std::string kMetricsTopic = "metrics";
  const std::string kMetricName1 = "CWMetricsNodeTestMetric";
  const std::string kMetricUnit1 = "sec";

  std::shared_ptr<ros::NodeHandle> node_handle = nullptr;
  std::shared_ptr<ros::Publisher> metrics_pub = nullptr;
  std::map<std::string, std::string> default_metric_dims;

  void SetUp()
  {
    node_handle = std::make_shared<ros::NodeHandle>();
    metrics_pub = std::make_shared<ros::Publisher>(
      node_handle->advertise<ros_monitoring_msgs::MetricList>(kMetricsTopic.c_str(), 1));
  }

  void TearDown() { ros::param::del(MetricsCollector::kNodeParamMonitorTopicsListKey); }

  void SendMetricMessages(int num_msgs, ros_monitoring_msgs::MetricData & metric_data_proto)
  {
    ros_monitoring_msgs::MetricList metric_list_msg = ros_monitoring_msgs::MetricList();
    for (int i = 0; i < num_msgs; i++) {
      metric_data_proto.value = i;
      metric_data_proto.time_stamp = ros::Time::now();
      metric_list_msg.metrics.clear();
      metric_list_msg.metrics.push_back(metric_data_proto);
      AWS_LOGSTREAM_DEBUG(__func__, "Publishing " << metric_list_msg.metrics.size()
                                                  << " metrics to topic " << kMetricsTopic.c_str());
      metrics_pub->publish(metric_list_msg);
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  ros_monitoring_msgs::MetricData BasicMetricData()
  {
    ros_monitoring_msgs::MetricData metric_data = ros_monitoring_msgs::MetricData();
    metric_data.metric_name = kMetricName1;
    metric_data.unit = kMetricUnit1;
    return metric_data;
  }
};

TEST_F(MetricsCollectorFixture, timerCallsMetricManagerService)
{
  int num_msgs = 3;

  std::shared_ptr<MetricManagerMock> metric_manager = std::make_shared<MetricManagerMock>();
  EXPECT_CALL(*metric_manager, Service())
    .Times(::testing::AtLeast(num_msgs))
    .WillRepeatedly(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  EXPECT_CALL(*metric_manager,
              RecordMetric(::testing::_, ::testing::_, ::testing::_, ::testing::_, ::testing::_))
    .Times(::testing::AnyNumber())
    .WillRepeatedly(::testing::Return(Aws::AwsError::AWS_ERR_OK));

  MetricsCollector metrics_collector =
    MetricsCollector(metric_manager, std::move(default_metric_dims));
  metrics_collector.Initialize(*node_handle);

  ros_monitoring_msgs::MetricData metric_data = BasicMetricData();
  SendMetricMessages(num_msgs, metric_data);
  for (int i = 0; i < num_msgs; i++) {
    ros::spinOnce();
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

TEST_F(MetricsCollectorFixture, metricsRecordedNoDimension)
{
  int num_msgs = 3;

  std::shared_ptr<MetricManagerMock> metric_manager = std::make_shared<MetricManagerMock>();
  EXPECT_CALL(*metric_manager, Service())
    .Times(::testing::AnyNumber())
    .WillRepeatedly(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  {
    ::testing::Sequence rm_seq;
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(0),
                             ::testing::StrEq(kMetricUnit1), ::testing::_, ::testing::IsEmpty()))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(1),
                             ::testing::StrEq(kMetricUnit1), ::testing::_, ::testing::IsEmpty()))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(2),
                             ::testing::StrEq(kMetricUnit1), ::testing::_, ::testing::IsEmpty()))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  }

  MetricsCollector metrics_collector =
    MetricsCollector(metric_manager, std::move(default_metric_dims));
  metrics_collector.Initialize(*node_handle);

  ros_monitoring_msgs::MetricData metric_data = BasicMetricData();
  SendMetricMessages(num_msgs, metric_data);
  ros::spinOnce();
}

TEST_F(MetricsCollectorFixture, metricRecordedWithDimension)
{
  int num_msgs = 3;
  const std::string metric_dimension_name = "CWMetricsNodeTestDim1";
  const std::string metric_dimension_value = "CWMetricsNodeTestDim1Value";

  std::shared_ptr<MetricManagerMock> metric_manager = std::make_shared<MetricManagerMock>();
  std::map<std::string, std::string> expected_dim;
  expected_dim[metric_dimension_name] = metric_dimension_value;
  EXPECT_CALL(*metric_manager, Service())
    .Times(::testing::AnyNumber())
    .WillRepeatedly(::testing::Return(Aws::AwsError::AWS_ERR_OK));

  {
    ::testing::Sequence rm_seq;
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(0),
                             ::testing::StrEq(kMetricUnit1), ::testing::_,
                             ::testing::ContainerEq(expected_dim)))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(1),
                             ::testing::StrEq(kMetricUnit1), ::testing::_,
                             ::testing::ContainerEq(expected_dim)))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(2),
                             ::testing::StrEq(kMetricUnit1), ::testing::_,
                             ::testing::ContainerEq(expected_dim)))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  }

  MetricsCollector metrics_collector =
    MetricsCollector(metric_manager, std::move(default_metric_dims));
  metrics_collector.Initialize(*node_handle);

  ros_monitoring_msgs::MetricData metric_data = BasicMetricData();
  ros_monitoring_msgs::MetricDimension metric_dimension = ros_monitoring_msgs::MetricDimension();
  metric_dimension.name = metric_dimension_name;
  metric_dimension.value = metric_dimension_value;
  metric_data.dimensions.push_back(metric_dimension);

  SendMetricMessages(num_msgs, metric_data);
  ros::spinOnce();
}

TEST_F(MetricsCollectorFixture, metricRecordedWithDefaultDimensions)
{
  int num_msgs = 3;
  const std::string metric_dimension_name = "CWMetricsNodeTestDim1";
  const std::string metric_dimension_value = "CWMetricsNodeTestDim1Value";

  std::shared_ptr<MetricManagerMock> metric_manager = std::make_shared<MetricManagerMock>();
  std::map<std::string, std::string> expected_dim;
  expected_dim[metric_dimension_name] = metric_dimension_value;
  EXPECT_CALL(*metric_manager, Service())
    .Times(::testing::AnyNumber())
    .WillRepeatedly(::testing::Return(Aws::AwsError::AWS_ERR_OK));

  {
    ::testing::Sequence rm_seq;
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(0),
                             ::testing::StrEq(kMetricUnit1), ::testing::_,
                             ::testing::ContainerEq(expected_dim)))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(1),
                             ::testing::StrEq(kMetricUnit1), ::testing::_,
                             ::testing::ContainerEq(expected_dim)))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
    EXPECT_CALL(*metric_manager,
                RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(2),
                             ::testing::StrEq(kMetricUnit1), ::testing::_,
                             ::testing::ContainerEq(expected_dim)))
      .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  }

  default_metric_dims.emplace(metric_dimension_name, metric_dimension_value);

  MetricsCollector metrics_collector =
    MetricsCollector(metric_manager, std::move(default_metric_dims));
  metrics_collector.Initialize(*node_handle);

  ros_monitoring_msgs::MetricData metric_data = BasicMetricData();

  SendMetricMessages(num_msgs, metric_data);
  ros::spinOnce();
}

TEST_F(MetricsCollectorFixture, customTopicsListened)
{
  std::vector<std::string> topics;
  topics.push_back("metrics_topic0");
  topics.push_back("metrics_topic1");
  ros::param::set(MetricsCollector::kNodeParamMonitorTopicsListKey, topics);

  std::shared_ptr<MetricManagerMock> metric_manager = std::make_shared<MetricManagerMock>();
  EXPECT_CALL(*metric_manager, Service())
    .Times(::testing::AnyNumber())
    .WillRepeatedly(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  EXPECT_CALL(*metric_manager, RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(0),
                                            ::testing::_, ::testing::_, ::testing::_))
    .Times(1)
    .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));
  EXPECT_CALL(*metric_manager, RecordMetric(::testing::StrEq(kMetricName1), ::testing::DoubleEq(1),
                                            ::testing::_, ::testing::_, ::testing::_))
    .Times(1)
    .WillOnce(::testing::Return(Aws::AwsError::AWS_ERR_OK));

  MetricsCollector metrics_collector =
    MetricsCollector(metric_manager, std::move(default_metric_dims));
  metrics_collector.Initialize(*node_handle);

  ros_monitoring_msgs::MetricList metric_list_msg = ros_monitoring_msgs::MetricList();
  ros_monitoring_msgs::MetricData metric_data = BasicMetricData();
  ros::Publisher metrics_pub0 =
    node_handle->advertise<ros_monitoring_msgs::MetricList>(topics[0].c_str(), 1);
  metric_data.value = 0;
  metric_data.time_stamp = ros::Time::now();
  metric_list_msg.metrics.clear();
  metric_list_msg.metrics.push_back(metric_data);
  metrics_pub0.publish(metric_list_msg);
  ros::spinOnce();
  ros::Publisher metrics_pub1 =
    node_handle->advertise<ros_monitoring_msgs::MetricList>(topics[1].c_str(), 1);
  metric_data.value = 1;
  metric_data.time_stamp = ros::Time::now();
  metric_list_msg.metrics.clear();
  metric_list_msg.metrics.push_back(metric_data);
  metrics_pub1.publish(metric_list_msg);
  ros::spinOnce();
  std::this_thread::sleep_for(std::chrono::seconds(1));
  ros::spinOnce();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "CWMetricsNodeTest");

  auto retval = RUN_ALL_TESTS();
  return retval;
}
