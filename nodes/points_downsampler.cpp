#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#define MAX_MEASUREMENT_RANGE 120.0
using std::placeholders::_1;

class VoxelGridFilterNode : public rclcpp::Node
{
public:
  VoxelGridFilterNode() : Node("voxel_grid_filter")
  {
    // Declare parameters
    this->declare_parameter<std::string>("points_topic", "points");
    this->declare_parameter<bool>("output_log", false);
    this->declare_parameter<double>("leaf_size", 2.0);

    // Get parameters
    this->get_parameter("points_topic", points_topic_);
    this->get_parameter("output_log", output_log_);
    this->get_parameter("leaf_size", voxel_leaf_size_);

    RCLCPP_INFO_STREAM(this->get_logger(), "Voxel leaf size is: " << voxel_leaf_size_);

    if (output_log_) {
      char buffer[80];
      std::time_t now = std::time(NULL);
      std::tm *pnow = std::localtime(&now);
      std::strftime(buffer, 80, "%Y%m%d_%H%M%S", pnow);
      filename_ = "voxel_grid_filter_" + std::string(buffer) + ".csv";
      ofs_.open(filename_.c_str(), std::ios::app);
    }

    // Create publisher
    filtered_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_points", 10);

    // Create subscriber
    scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, 10, std::bind(&VoxelGridFilterNode::scan_callback, this, _1)
    );
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_points_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  double voxel_leaf_size_;
  bool output_log_;
  std::ofstream ofs_;
  std::string filename_;
  std::string points_topic_;

  pcl::PointCloud<pcl::PointXYZ> removePointsByRange(pcl::PointCloud<pcl::PointXYZ> scan, double min_range, double max_range)
  {
    pcl::PointCloud<pcl::PointXYZ> narrowed_scan;
    narrowed_scan.header = scan.header;
    if (min_range >= max_range) {
      RCLCPP_ERROR_ONCE(this->get_logger(), "min_range>=max_range @(%lf, %lf)", min_range, max_range);
      return scan;
    }
    double square_min_range = min_range * min_range;
    double square_max_range = max_range * max_range;
    for (const auto& p : scan) {
      double square_distance = p.x * p.x + p.y * p.y;
      if (square_min_range <= square_distance && square_distance <= square_max_range) {
        narrowed_scan.points.push_back(p);
      }
    }
    return narrowed_scan;
  }

  void scan_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZ> scan;
    pcl::fromROSMsg(*input, scan);
    scan = removePointsByRange(scan, 0, MAX_MEASUREMENT_RANGE);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_ptr(new pcl::PointCloud<pcl::PointXYZ>(scan));
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan_ptr(new pcl::PointCloud<pcl::PointXYZ>());
    sensor_msgs::msg::PointCloud2 filtered_msg;

    if (voxel_leaf_size_ >= 0.1) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_filter;
      voxel_grid_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_grid_filter.setInputCloud(scan_ptr);
      voxel_grid_filter.filter(*filtered_scan_ptr);
      pcl::toROSMsg(*filtered_scan_ptr, filtered_msg);
    } else {
      pcl::toROSMsg(*scan_ptr, filtered_msg);
    }

    filtered_msg.header = input->header;
    filtered_points_pub_->publish(filtered_msg);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VoxelGridFilterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
