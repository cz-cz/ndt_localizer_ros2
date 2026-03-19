#ifndef NDT_LOCALIZER_H
#define NDT_LOCALIZER_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <cstdlib>
#include <ctime>

class NdtLocalizerNode : public rclcpp::Node {
public:
    explicit NdtLocalizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~NdtLocalizerNode();

private:
    void callback_init_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg_ptr);
    void callback_pointsmap(const sensor_msgs::msg::PointCloud2::SharedPtr map_points_msg_ptr);
    void callback_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr sensor_points_msg_ptr);
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg_ptr);
    void timer_diagnostic();
    void init_params();
    bool get_transform(const std::string & target_frame, const std::string & source_frame,
                       geometry_msgs::msg::TransformStamped & transform_stamped);
    void publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::msg::PoseStamped & pose_msg);

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_aligned_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr exe_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr transform_probability_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr iteration_num_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_points_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_points_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Timer
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;

    // TF2
    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;
    tf2_ros::TransformBroadcaster tf2_broadcaster_;

    // PCL NDT
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
    std::mutex ndt_map_mtx_;

    // Parameters
    std::string base_frame_;
    std::string odom_frame_;
    std::string map_frame_;
    double converged_param_transform_probability_;

    // State
    bool init_pose = false;
    Eigen::Matrix4f pre_trans;
    Eigen::Matrix4f delta_trans;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_cov_msg_;
    std::map<std::string, std::string> key_value_stdmap_;

    // Odometry related
    Eigen::Matrix4f odom_pose_matrix_;
    bool has_odom_;
    int fail_count_;

    // Map bounds for random initialization
    Eigen::Vector4f map_min_;
    Eigen::Vector4f map_max_;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr convertLivoxPointCloud(
    // const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg);
};

#endif // NDT_LOCALIZER_H