^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_metrics_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-03-20)
------------------
* fix broken build when cmake/rostest cannot find the gmock library (`#13 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/13>`_)
* Fix timestamp conversion bug `#10 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/10>`_
* Update CMakeLists.txt
* Update to use non-legacy ParameterReader API (`#7 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/7>`_)
  * Update to use non-legacy ParameterReader API
  * increment package version
  * address comments, remove use of Aws::CloudWatch::Metrics namespace
* Allow users to configure ROS output location
* Contributors: AAlon, Juan Rodriguez Hortala, M. M, Tim Robinson
