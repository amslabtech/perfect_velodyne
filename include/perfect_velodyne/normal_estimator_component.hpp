// Copyright 2020 amsl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PERFECT_VELODYNE__NORMAL_ESTIMATOR_COMPONENT_HPP_
#define PERFECT_VELODYNE__NORMAL_ESTIMATOR_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_EXPORT __attribute__ ((dllexport))
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_IMPORT __attribute__ ((dllimport))
  #else
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_EXPORT __declspec(dllexport)
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef PERFECT_VELODYNE_NORMAL_ESTIMATOR_BUILDING_DLL
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC PERFECT_VELODYNE_NORMAL_ESTIMATOR_EXPORT
  #else
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC PERFECT_VELODYNE_NORMAL_ESTIMATOR_IMPORT
  #endif
  #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC_TYPE PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC
  #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_LOCAL
#else
  #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_EXPORT __attribute__ ((visibility("default")))
  #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_IMPORT
  #if __GNUC__ >= 4
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC __attribute__ ((visibility("default")))
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC
    #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_LOCAL
  #endif
  #define PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

#include <Eigen/Dense>

#include <omp.h>

namespace perfect_velodyne
{
class NormalEstimatorComponent : public rclcpp::Node
{
public:
  typedef pcl::PointXYZI PointXYZI;
  typedef pcl::PointCloud<PointXYZI> CloudXYZI;
  typedef pcl::PointCloud<PointXYZI>::Ptr CloudXYZIPtr;
  typedef pcl::PointXYZINormal PointXYZIN;
  typedef pcl::PointCloud<PointXYZIN> CloudXYZIN;
  typedef pcl::PointCloud<PointXYZIN>::Ptr CloudXYZINPtr;

  PERFECT_VELODYNE_NORMAL_ESTIMATOR_PUBLIC
  explicit NormalEstimatorComponent(const rclcpp::NodeOptions &);

  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr);
  void estimate_normal(CloudXYZINPtr &);
  bool validate_range(double);
  int get_ring_index_from_firing_order(int);
  int get_firing_order_from_ring_index(int);
  bool validate_ring(int);
  void get_gaussian_sphere(const CloudXYZINPtr &, CloudXYZINPtr &);
  void get_d_gaussian_sphere(const CloudXYZINPtr &, CloudXYZINPtr &);
  void publish_normal_marker(const CloudXYZINPtr &);
  void remove_invalid_points(CloudXYZINPtr &);
  void filter_curvature(CloudXYZINPtr &);

private:
  double max_range_;
  double min_range_;
  int skip_;
  int vertical_points_;
  int horizontal_points_;
  int layer_num_;
  double query_radius_;
  double density_;
  double gaussian_sphere_radius_;
  double max_curvature_threshold_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr normal_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gaussian_sphere_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gaussian_sphere_filtered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr d_gaussian_sphere_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr d_gaussian_sphere_filtered_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr normal_marker_pub_;
};
}  //  namespace perfect_velodyne

#endif  // PERFECT_VELODYNE__NORMAL_ESTIMATOR_COMPONENT_HPP_
