^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cloudwatch_metrics_collector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Bumpinng package version to match bloom release (`#45 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/45>`_)
  Bump version to 2.2.1
* Fix linting issues found by clang-tidy 6.0 (`#43 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/43>`_)
  * clang-tidy fixes
  * clang-tidy linting issues fixed manually
* Bump package version to 2.2.0 (`#42 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/42>`_)
* Fix default metric options
  - Fix bug where default metric storage directory, file prefix etc were
  being set to the same as logs causing issues when running both
  cloudwatch logs and metrics at the same time.
* Merge pull request `#37 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/37>`_ from aws-robotics/restore-tests
  Restored unit tests
* Restored unit tests
* - minor README changes (`#35 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/35>`_)
  - remove debug flag (run with level info)
* install missing library (`#33 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/33>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* increment version for offline metrics feature (`#32 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/32>`_)
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Fixing std_msgs, std_srvs dependency resolution (`#30 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/30>`_)
  * Add std_msgs to CMakeLists.txt
  * Add build_depend on std_srvs, std_msgs
* Find std_srvs in CMakeLists.txt
* Offline Metrics (`#28 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/28>`_)
  * ROS-1775: Adapt Interfaces to CW Metrics
  cr https://code.amazon.com/reviews/CR-10106570
  * Addressed review comments
  * Add reading of parameters from config file
  - Add reading of CloudWatch upload and file management settings from the
  config file. This is very similar to CloudWatchLogs parameter reading
  but using a different base CloudWatchOptions with some different default
  settings
  - Add sample configuration file that has all the same values as the
  default values for each option in alphabetical order.
  cr https://code.amazon.com/reviews/CR-10331605
  * Disable trigger_publish_size by default. Add comment that it cannot be larger than max_queue_size
  * Move advanced configuration to README, code style fixes
  - Move all the advanced configuration parameters for offline metrics to the
  README instead of the sample configuration.
  - Improve parameter reading to re-use functions more and not return any
  codes if they're just going to be ignored.
  cr https://code.amazon.com/reviews/CR-10537580
  * Style fixes. Change missing param WARN's to INFO's
  * [Bug] CW Metrics ROS1 Timer incorrectly created
  cr https://code.amazon.com/reviews/CR-10549890
  * Remove stream_max_queue_size as it's not used
  * Improved README. Removed performance benchmarking
  * Add stream_max_queue_size parameter
  Add reading this parameter from config file and describing it in the advanced parameters section of the README.
  cr https://code.amazon.com/reviews/CR-10583080
  *  - support service changes
  - add query for connection state
  * fix unit test failure
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Use standard CMake macros for adding gtest/gmock tests (`#25 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/25>`_)
  * modify cloudwatch_metrics_collector to use add_rostest_gmock()
  Signed-off-by: Miaofei <miaofei@amazon.com>
  * update travis.yml to be compatible with specifying multiple package names
  Signed-off-by: Miaofei <miaofei@amazon.com>
* Add test for timestamp extraction from MetricData
* fix broken build when cmake/rostest cannot find the gmock library (`#13 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/13>`_)
* Fix timestamp conversion bug `#10 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/10>`_
* Update CMakeLists.txt
* Update to use non-legacy ParameterReader API (`#7 <https://github.com/aws-robotics/cloudwatchmetrics-ros1/issues/7>`_)
  * Update to use non-legacy ParameterReader API
  * increment package version
  * address comments, remove use of Aws::CloudWatch::Metrics namespace
* Allow users to configure ROS output location
* Contributors: AAlon, Devin Bonnie, Juan Rodriguez Hortala, M. M, Miaofei Mei, Ragha Prasad, Ryan Newell, Tim Robinson
