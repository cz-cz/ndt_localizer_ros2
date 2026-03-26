#include "ndt.h"
NdtLocalizerNode::NdtLocalizerNode(const rclcpp::NodeOptions & options) : Node("ndt_localizer", options),
    tf2_buffer_(this->get_clock()),
    tf2_listener_(tf2_buffer_),
    tf2_broadcaster_(this) {
    key_value_stdmap_["state"] = "Initializing";
    // 初始化随机种子
    srand(time(NULL));
    // 初始化 Small-GICP (替换了原来的PCL GICP)
    ndt_ = small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ>::Ptr(
        new small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ>());
    init_params();
    // Publishers
    sensor_aligned_pose_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
    ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
    // Subscribers
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 100,
        std::bind(&NdtLocalizerNode::callback_init_pose, this, std::placeholders::_1));
    map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_map", 1,
        std::bind(&NdtLocalizerNode::callback_pointsmap, this, std::placeholders::_1));
    sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filtered_points", rclcpp::SensorDataQoS(),
        std::bind(&NdtLocalizerNode::callback_pointcloud, this, std::placeholders::_1));
    // 订阅里程计消息
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/odometry", 10,
        std::bind(&NdtLocalizerNode::callback_odom, this, std::placeholders::_1));
    // Timer 替代线程
    diagnostic_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), // 100Hz
        std::bind(&NdtLocalizerNode::timer_diagnostic, this));
    // 初始化里程计相关变量
    fail_count_ = 0;
    has_odom_ = false;
    odom_pose_matrix_ = Eigen::Matrix4f::Identity();
    T_map_odom_good_save_ = Eigen::Matrix4f::Identity();
    RCLCPP_INFO(this->get_logger(), "Small-GICP Localizer initialized (replaced PCL GICP)");
}
NdtLocalizerNode::~NdtLocalizerNode() {}
void NdtLocalizerNode::timer_diagnostic() {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "small_gicp_scan_matcher";
    diag_status_msg.hardware_id = "";
    for (const auto & key_value : key_value_stdmap_) {
        diagnostic_msgs::msg::KeyValue key_value_msg;
        key_value_msg.key = key_value.first;
        key_value_msg.value = key_value.second;
        diag_status_msg.values.push_back(key_value_msg);
    }
    diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {
        diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diag_status_msg.message += "Initializing State. ";
    }
    if (
        key_value_stdmap_.count("skipping_publish_num") &&
        std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
        diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (
        key_value_stdmap_.count("skipping_publish_num") &&
        std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
        diag_status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }
}
void NdtLocalizerNode::callback_init_pose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial_pose_msg_ptr) {
    if (initial_pose_msg_ptr->header.frame_id == map_frame_) {
        initial_pose_cov_msg_ = *initial_pose_msg_ptr;
    } else {
        // get TF from pose_frame to map_frame
        geometry_msgs::msg::TransformStamped TF_pose_to_map;
        if (get_transform(map_frame_, initial_pose_msg_ptr->header.frame_id, TF_pose_to_map)) {
            // transform pose_frame to map_frame
            geometry_msgs::msg::PoseWithCovarianceStamped mapTF_initial_pose_msg;
            tf2::doTransform(*initial_pose_msg_ptr, mapTF_initial_pose_msg, TF_pose_to_map);
            initial_pose_cov_msg_ = mapTF_initial_pose_msg;
        }
    }
    // if click the initpose again, re init!
    init_pose = false;
    RCLCPP_INFO(this->get_logger(), "Received initial pose, will use it for Small-GICP matching");
}
void NdtLocalizerNode::callback_pointsmap(
    const sensor_msgs::msg::PointCloud2::SharedPtr map_points_msg_ptr) {
    // 获取当前GICP的参数
    const auto trans_epsilon = ndt_->getTransformationEpsilon();
    const auto max_corr_dist = ndt_->getMaxCorrespondenceDistance();
    const auto fitness_epsilon = ndt_->getEuclideanFitnessEpsilon();
    const auto max_iterations = ndt_->getMaximumIterations();
    // 创建新的GICP实例（使用指针避免PCL 1.12的拷贝赋值问题）
    small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_new(
        new small_gicp::RegistrationPCL<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_new->setTransformationEpsilon(trans_epsilon);
    ndt_new->setMaxCorrespondenceDistance(max_corr_dist);
    ndt_new->setEuclideanFitnessEpsilon(fitness_epsilon);
    ndt_new->setMaximumIterations(max_iterations);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    ndt_new->setInputTarget(map_points_ptr);
    // 计算地图的边界
    pcl::getMinMax3D<pcl::PointXYZ>(*map_points_ptr, map_min_, map_max_);
    RCLCPP_INFO(this->get_logger(), "地图范围：x[%f, %f], y[%f, %f], z[%f, %f]", 
                map_min_[0], map_max_[0], map_min_[1], map_max_[1], map_min_[2], map_max_[2]);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_new->align(*output_cloud, Eigen::Matrix4f::Identity());
    // swap，直接替换指针，无需拷贝，完全兼容PCL 1.12
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);
    ndt_ = ndt_new;
    RCLCPP_INFO(this->get_logger(), "Map loaded: %zu points, ready for Small-GICP matching", map_points_ptr->size());
}
void NdtLocalizerNode::callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg_ptr) {
    // 将里程计的位姿转换为Eigen矩阵，作为GICP的初始位姿
    Eigen::Affine3d odom_affine;
    tf2::fromMsg(odom_msg_ptr->pose.pose, odom_affine);
    odom_pose_matrix_ = odom_affine.matrix().cast<float>();
    has_odom_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received odometry pose, will use as Small-GICP initial pose");
}
// pcl::PointCloud<pcl::PointXYZ>::Ptr NdtLocalizerNode::convertLivoxPointCloud(
//     const sensor_msgs::msg::PointCloud2::SharedPtr point_cloud_msg)
// {
//     // 检查点云是否为空
//     if (point_cloud_msg->data.empty()) {
//         RCLCPP_ERROR(rclcpp::get_logger("ndt_localizer"), "PointCloud2 data is empty");
//         return pcl::PointCloud<pcl::PointXYZ>::Ptr();
//     }
//     // 创建 PCL 点云
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    
//     cloud->header.frame_id = point_cloud_msg->header.frame_id;
    
//     cloud->height = point_cloud_msg->height;
//     cloud->width = point_cloud_msg->width;
//     cloud->is_dense = point_cloud_msg->is_dense;
//     cloud->points.resize(point_cloud_msg->width * point_cloud_msg->height);
//     // 获取原始数据指针
//     const uint8_t* data = point_cloud_msg->data.data();
//     const size_t point_step = point_cloud_msg->point_step;
//     const size_t num_points = point_cloud_msg->width * point_cloud_msg->height;
//     // 遍历每个点（保持原有逻辑）
//     for (size_t i = 0; i < num_points; ++i) {
//         float x = *reinterpret_cast<const float*>(data + i * point_step + 0);
//         float y = *reinterpret_cast<const float*>(data + i * point_step + 4);
//         float z = *reinterpret_cast<const float*>(data + i * point_step + 8);
        
//         cloud->points[i].x = x;
//         cloud->points[i].y = y;
//         cloud->points[i].z = z;
//     }
//     return cloud;
// }
void NdtLocalizerNode::callback_pointcloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr sensor_points_sensorTF_msg_ptr) {
    const auto exe_start_time = std::chrono::system_clock::now();
    // mutex Map
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);
    const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
    const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_sensorTF_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
    
    // sensor_points_sensorTF_ptr = convertLivoxPointCloud(sensor_points_sensorTF_msg_ptr);
    // get TF base to sensor
    geometry_msgs::msg::TransformStamped TF_base_to_sensor;
    if (!get_transform(base_frame_, sensor_frame, TF_base_to_sensor)) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to get transform");
        return;
    }
    const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(TF_base_to_sensor);
    const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_baselinkTF_ptr(
        new pcl::PointCloud<pcl::PointXYZ>);
    // 使用 Eigen 变换点云
    for (auto & point : sensor_points_sensorTF_ptr->points) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f transformed_p = base_to_sensor_matrix * p;
        point.x = transformed_p.x();
        point.y = transformed_p.y();
        point.z = transformed_p.z();
    }
    *sensor_points_baselinkTF_ptr = *sensor_points_sensorTF_ptr;
    // set input point cloud
    ndt_->setInputSource(sensor_points_baselinkTF_ptr);
    if (ndt_->getInputTarget() == nullptr) {
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No MAP!");
        return;
    }
    // 准备初始位姿：优先使用里程计，然后是初始位姿话题，然后是上一帧的位姿
    Eigen::Matrix4f initial_pose_matrix;
    if (has_odom_) {
        initial_pose_matrix = odom_pose_matrix_;
        RCLCPP_INFO(this->get_logger(), "---Using odometry as Small-GICP initial pose---------");
        if (!init_pose)
        {
          Eigen::Affine3d initial_pose_affine;
          tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
          initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
          pre_trans = initial_pose_matrix;
          init_pose = true;
          RCLCPP_INFO(this->get_logger(), "-----Using initial pose from topic as Small-GICP initial pose----------");
        }
    }
    else
    {
      if (!init_pose)
      {
        Eigen::Affine3d initial_pose_affine;
        tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
        initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
        pre_trans = initial_pose_matrix;
        init_pose = true;
        RCLCPP_INFO(this->get_logger(), "-----Using initial pose from topic as Small-GICP initial pose----------");
      }
      else
      {
        initial_pose_matrix = pre_trans * delta_trans;
        RCLCPP_INFO(this->get_logger(), "------Using previous pose as Small-GICP initial pose-------");
      }
    }
    std::cout<<"-------------Small-GICP initial_pose_matrix-------"<<std::endl<<initial_pose_matrix<<std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();
    key_value_stdmap_["state"] = "Aligning";
    // 使用初始位姿进行GICP匹配，这正是需求的功能
    ndt_->align(*output_cloud, initial_pose_matrix);
    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time = std::chrono::system_clock::now();
    const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;
    Eigen::Matrix4f result_pose_matrix = ndt_->getFinalTransformation();
    // 使用fitness score代替原来的transform probability，GICP不支持后者
    float transform_probability = ndt_->getFitnessScore();
    int iteration_num = 0;
    bool is_converged = true;
    static size_t skipping_publish_num = 0;
    std::cout<<ndt_->getFitnessScore()<<" getFitnessScore"<<std::endl;
    std::cout<<iteration_num<<" iteration_num"<<std::endl;
    std::cout<<ndt_->getMaximumIterations() + 2<<" max_iterations + 2"<<std::endl;
    std::cout<<transform_probability<<" fitness_score"<<std::endl;
    std::cout<<converged_param_transform_probability_ <<" converged_param_fitness_score"<<std::endl;
    // 收敛判断：fitness score小于阈值认为收敛，和原来的判断逻辑相反
    if (ndt_->getFitnessScore()>0.5) {
        is_converged = false;
        ++skipping_publish_num;
        fail_count_++;
        RCLCPP_INFO(this->get_logger(), "Small-GICP Not Converged, fail count: %d", fail_count_);
    } else {
        skipping_publish_num = 0;
        fail_count_ = 0; // 成功，重置失败计数器
    }

    // calculate the delta tf from pre_trans to current_trans
    delta_trans = pre_trans.inverse() * result_pose_matrix;
    Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
    RCLCPP_INFO(this->get_logger(), "delta x: %f y: %f z: %f",
                delta_translation(0), delta_translation(1), delta_translation(2));
    Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
    Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2, 1, 0);
    RCLCPP_INFO(this->get_logger(), "delta yaw: %f pitch: %f roll: %f",
                delta_euler(0), delta_euler(1), delta_euler(2));
    pre_trans = result_pose_matrix;
    if (transform_probability < 0.5)
    {    
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

    // publish
    geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
    result_pose_stamped_msg.header.stamp = sensor_ros_time;
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_msg.pose = result_pose_msg;
    ndt_pose_pub_->publish(result_pose_stamped_msg);
    
    // 计算map到odom的变换
    Eigen::Matrix4f T_map_odom;
    if (has_odom_) {
        // 已知：T_map_base（定位结果，map到base_link的变换）、T_base_odom（里程计，base_link到odom的变换）
        // 目标：计算map到odom的变换，直接相乘即可（无需求逆，适配当前里程计frame配置）
        T_map_odom = result_pose_matrix * odom_pose_matrix_.inverse();
    } else {
        // 没有里程计时，odom和base_link重合，所以T_map_odom = T_map_base
        T_map_odom = Eigen::Matrix4f::Identity();
    }
    T_map_odom_good_save_ = T_map_odom;
    // 转换为PoseStamped
    Eigen::Affine3d T_map_odom_affine;
    T_map_odom_affine.matrix() = T_map_odom.cast<double>();
    geometry_msgs::msg::Pose T_map_odom_pose = tf2::toMsg(T_map_odom_affine);

    geometry_msgs::msg::PoseStamped map_odom_pose_stamped;
    map_odom_pose_stamped.header.stamp = sensor_ros_time;
    map_odom_pose_stamped.header.frame_id = map_frame_;
    map_odom_pose_stamped.pose = T_map_odom_pose;

    // 发布map到odom的TF
    publish_tf(map_frame_, odom_frame_, map_odom_pose_stamped);
        // publish aligned point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // 使用 Eigen 变换点云
    for (auto & point : sensor_points_baselinkTF_ptr->points) {
        Eigen::Vector4f p(point.x, point.y, point.z, 1.0);
        Eigen::Vector4f transformed_p = result_pose_matrix * p;
        point.x = transformed_p.x();
        point.y = transformed_p.y();
        point.z = transformed_p.z();
    }
    *sensor_points_mapTF_ptr = *sensor_points_baselinkTF_ptr;

    sensor_msgs::msg::PointCloud2 sensor_points_mapTF_msg;
    pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
    sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
    sensor_points_mapTF_msg.header.frame_id = map_frame_;
    sensor_aligned_pose_pub_->publish(sensor_points_mapTF_msg);  
    }  

}
bool NdtLocalizerNode::get_transform(const std::string & target_frame, const std::string & source_frame,
                       geometry_msgs::msg::TransformStamped & transform_stamped) {
    if (target_frame == source_frame) {
        transform_stamped.header.stamp = this->now();
        transform_stamped.header.frame_id = target_frame;
        transform_stamped.child_frame_id = source_frame;
        transform_stamped.transform.translation.x = 0.0;
        transform_stamped.transform.translation.y = 0.0;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = 0.0;
        transform_stamped.transform.rotation.y = 0.0;
        transform_stamped.transform.rotation.z = 0.0;
        transform_stamped.transform.rotation.w = 1.0;
        return true;
    }
    try {
        transform_stamped = tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
        return true;
    } catch (tf2::TransformException & ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", source_frame.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

void NdtLocalizerNode::publish_tf(const std::string & frame_id, const std::string & child_frame_id,
                    const geometry_msgs::msg::PoseStamped & pose_msg) {
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = pose_msg.header.stamp;
    transform_stamped.header.frame_id = frame_id;
    transform_stamped.child_frame_id = child_frame_id;
    transform_stamped.transform.translation.x = pose_msg.pose.position.x;
    transform_stamped.transform.translation.y = pose_msg.pose.position.y;
    transform_stamped.transform.translation.z = pose_msg.pose.position.z;
    transform_stamped.transform.rotation = pose_msg.pose.orientation;
    tf2_broadcaster_.sendTransform(transform_stamped);
}
void NdtLocalizerNode::init_params() {
    // 帧ID参数
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->get_parameter("base_frame", base_frame_);
    this->declare_parameter<std::string>("map_frame", "map");
    this->get_parameter("map_frame", map_frame_);
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->get_parameter("odom_frame", odom_frame_); 
    // GICP 配准参数
    this->declare_parameter<double>("trans_epsilon", 0.01);
    double trans_epsilon;
    this->get_parameter("trans_epsilon", trans_epsilon);
    ndt_->setTransformationEpsilon(trans_epsilon);
    this->declare_parameter<double>("max_correspondence_distance", 1.0);
    double max_corr_dist;
    this->get_parameter("max_correspondence_distance", max_corr_dist);
    ndt_->setMaxCorrespondenceDistance(max_corr_dist);
    this->declare_parameter<double>("euclidean_fitness_epsilon", 0.01);
    double fitness_epsilon;
    this->get_parameter("euclidean_fitness_epsilon", fitness_epsilon);
    ndt_->setEuclideanFitnessEpsilon(fitness_epsilon);
    this->declare_parameter<int>("max_iterations", 30);
    int max_iterations;
    this->get_parameter("max_iterations", max_iterations);
    ndt_->setMaximumIterations(max_iterations);
    // 收敛判断参数：fitness score阈值，小于该值认为收敛
    this->declare_parameter<double>("converged_param_fitness_score", 1.0);
    this->get_parameter("converged_param_fitness_score", converged_param_transform_probability_);
}
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<NdtLocalizerNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
