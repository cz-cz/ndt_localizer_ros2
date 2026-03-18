#include "map_loader.h"
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

MapLoaderNode::MapLoaderNode() : rclcpp::Node("map_loader")
{
    // Declare parameters
    this->declare_parameter<std::string>("pcd_path", "");
    this->declare_parameter<std::string>("map_topic", "point_map");
    this->declare_parameter<float>("x", 0.0f);
    this->declare_parameter<float>("y", 0.0f);
    this->declare_parameter<float>("z", 0.0f);
    this->declare_parameter<float>("roll", 0.0f);
    this->declare_parameter<float>("pitch", 0.0f);
    this->declare_parameter<float>("yaw", 0.0f);
    this->declare_parameter<int>("publish_count", 1);        // 新增：发布次数
    this->declare_parameter<int>("publish_interval_sec", 1);  // 新增：发布间隔（秒）

    // Get parameters
    std::string pcd_file_path;
    std::string map_topic;
    this->get_parameter("pcd_path", pcd_file_path);
    this->get_parameter("map_topic", map_topic);
    this->get_parameter("x", tf_x_);
    this->get_parameter("y", tf_y_);
    this->get_parameter("z", tf_z_);
    this->get_parameter("roll", tf_roll_);
    this->get_parameter("pitch", tf_pitch_);
    this->get_parameter("yaw", tf_yaw_);
    this->get_parameter("publish_count", publish_count_);        // 新增
    this->get_parameter("publish_interval_sec", publish_interval_sec_);  // 新增

    RCLCPP_INFO_STREAM(this->get_logger(), "x: " << tf_x_ << " y: " << tf_y_ << " z: " << tf_z_ 
                        << " roll: " << tf_roll_ << " pitch: " << tf_pitch_ << " yaw: " << tf_yaw_);
    RCLCPP_INFO(this->get_logger(), "Publish count: %d, Interval: %d sec", 
                publish_count_, publish_interval_sec_);

    // Create publisher
    pc_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(map_topic, 10);

    // Load map once (在内存中保存变换后的地图)
    file_list_.push_back(pcd_file_path);
    auto pc_msg = CreatePcd();
    cached_map_msg_ = TransformMap(pc_msg);  // 缓存变换后的地图
    
    if (cached_map_msg_.width != 0) {
        RCLCPP_INFO(this->get_logger(), "Map loaded: %d points", cached_map_msg_.width);
        
        // 创建定时器，每秒发布一次
        timer_ = this->create_wall_timer(
            std::chrono::seconds(publish_interval_sec_),
            std::bind(&MapLoaderNode::timer_callback, this)
        );
        
        publish_counter_ = 0;  // 初始化计数器
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map!");
    }
}

void MapLoaderNode::timer_callback()
{
    publish_counter_++;
    
    if (publish_counter_ > publish_count_) {
        RCLCPP_INFO(this->get_logger(), "Finished publishing %d times. Stopping timer.", publish_count_);
        timer_->cancel();  // 停止定时器
        return;
    }
    
    // 发布缓存的地图消息
    auto out_msg = cached_map_msg_;
    out_msg.header.frame_id = "map";
    out_msg.header.stamp = this->now();  // 更新时间戳
    
    pc_map_pub_->publish(out_msg);
    
    RCLCPP_INFO(this->get_logger(), "[%d/%d] Map published", publish_counter_, publish_count_);
}

sensor_msgs::msg::PointCloud2 MapLoaderNode::TransformMap(sensor_msgs::msg::PointCloud2 &in)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(in, *in_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    Eigen::Translation3f tl_m2w(tf_x_, tf_y_, tf_z_);
    Eigen::AngleAxisf rot_x_m2w(tf_roll_, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_m2w(tf_pitch_, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_m2w(tf_yaw_, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f tf_m2w = (tl_m2w * rot_z_m2w * rot_y_m2w * rot_x_m2w).matrix();

    pcl::transformPointCloud(*in_pc, *transformed_pc_ptr, tf_m2w);
    // SaveMap(transformed_pc_ptr);  // 可选：是否每次保存

    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*transformed_pc_ptr, output_msg);
    return output_msg;
}

void MapLoaderNode::SaveMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr map_pc_ptr)
{
    pcl::io::savePCDFile("/tmp/transformed_map.pcd", *map_pc_ptr);
}

sensor_msgs::msg::PointCloud2 MapLoaderNode::CreatePcd()
{
    sensor_msgs::msg::PointCloud2 pcd, part;
    for (const std::string& path : file_list_) {
      if (pcd.width == 0) {
        if (pcl::io::loadPCDFile(path.c_str(), pcd) == -1) {
          RCLCPP_ERROR(this->get_logger(), "load failed %s", path.c_str());
        }
      } else {
        if (pcl::io::loadPCDFile(path.c_str(), part) == -1) {
          RCLCPP_ERROR(this->get_logger(), "load failed %s", path.c_str());
        }
        pcd.width += part.width;
        pcd.row_step += part.row_step;
        pcd.data.insert(pcd.data.end(), part.data.begin(), part.data.end());
      }
      RCLCPP_INFO(this->get_logger(), "load %s", path.c_str());
      if (!rclcpp::ok()) break;
    }
    return pcd;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("map_loader"), "\033[1;32m---->\033[0m Map Loader Started.");
  auto node = std::make_shared<MapLoaderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}