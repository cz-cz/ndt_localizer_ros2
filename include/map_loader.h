#ifndef _MAP_LOADER_H_
#define _MAP_LOADER_H_
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
class MapLoaderNode : public rclcpp::Node
{
public:
  MapLoaderNode();
private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_map_pub_;
  std::vector<std::string> file_list_;
  float tf_x_, tf_y_, tf_z_, tf_roll_, tf_pitch_, tf_yaw_; 
  void init_tf_params();
  sensor_msgs::msg::PointCloud2 CreatePcd();
  sensor_msgs::msg::PointCloud2 TransformMap(sensor_msgs::msg::PointCloud2 & in);
  void SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr);

  void timer_callback(); // 新增
                         // Timer  // 新增
  rclcpp::TimerBase::SharedPtr timer_;
  int publish_count_;        // 新增
  int publish_interval_sec_; // 新增
  int publish_counter_;      // 新增
  sensor_msgs::msg::PointCloud2 cached_map_msg_;
};
#endif
