#include "ndt.h"

NdtLocalizerNode::NdtLocalizerNode(const rclcpp::NodeOptions & options) : Node("ndt_localizer", options),
    tf2_buffer_(this->get_clock()),
    tf2_listener_(tf2_buffer_),
    tf2_broadcaster_(this) {
    key_value_stdmap_["state"] = "Initializing";

    // 初始化随机种子
    srand(time(NULL));

    // 初始化 NDT
    ndt_ = pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr(
        new pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());

    init_params();

    // Publishers
    sensor_aligned_pose_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points_aligned", 10);
    ndt_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ndt_pose", 10);
    exe_time_pub_ = this->create_publisher<std_msgs::msg::Float32>("exe_time_ms", 10);
    transform_probability_pub_ = this->create_publisher<std_msgs::msg::Float32>("transform_probability", 10);
    iteration_num_pub_ = this->create_publisher<std_msgs::msg::Float32>("iteration_num", 10);
    diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);

    // Subscribers
    initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 100,
        std::bind(&NdtLocalizerNode::callback_init_pose, this, std::placeholders::_1));

    map_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "points_map", 1,
        std::bind(&NdtLocalizerNode::callback_pointsmap, this, std::placeholders::_1));

    sensor_points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "filtered_points", 1,
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

    RCLCPP_INFO(this->get_logger(), "NdtLocalizerNode initialized");
}

NdtLocalizerNode::~NdtLocalizerNode() {}

void NdtLocalizerNode::timer_diagnostic() {
    diagnostic_msgs::msg::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "ndt_scan_matcher";
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
    diagnostic_msgs::msg::DiagnosticArray diag_msg;
    diag_msg.header.stamp = this->now();
    diag_msg.status.push_back(diag_status_msg);
    diagnostics_pub_->publish(diag_msg);
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

    RCLCPP_INFO(this->get_logger(), "Received initial pose");
}

void NdtLocalizerNode::callback_pointsmap(
    const sensor_msgs::msg::PointCloud2::SharedPtr map_points_msg_ptr) {
    const auto trans_epsilon = ndt_->getTransformationEpsilon();
    const auto step_size = ndt_->getStepSize();
    const auto resolution = ndt_->getResolution();
    const auto max_iterations = ndt_->getMaximumIterations();
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_new;
    ndt_new.setTransformationEpsilon(trans_epsilon);
    ndt_new.setStepSize(step_size);
    ndt_new.setResolution(resolution);
    ndt_new.setMaximumIterations(max_iterations);
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
    ndt_new.setInputTarget(map_points_ptr);

    // 计算地图的边界
    // pcl::getMinMax3D(*map_points_ptr, map_min_, map_max_);
    pcl::getMinMax3D<pcl::PointXYZ>(*map_points_ptr, map_min_, map_max_);
    RCLCPP_INFO(this->get_logger(), "地图范围：x[%f, %f], y[%f, %f], z[%f, %f]", 
                map_min_[0], map_max_[0], map_min_[1], map_max_[1], map_min_[2], map_max_[2]);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ndt_new.align(*output_cloud, Eigen::Matrix4f::Identity());
    // swap
    std::lock_guard<std::mutex> lock(ndt_map_mtx_);
    *ndt_ = ndt_new;

    RCLCPP_INFO(this->get_logger(), "Map loaded: %zu points", map_points_ptr->size());
}

void NdtLocalizerNode::callback_odom(const nav_msgs::msg::Odometry::SharedPtr odom_msg_ptr) {
    // 将里程计的位姿转换为Eigen矩阵
    Eigen::Affine3d odom_affine;
    tf2::fromMsg(odom_msg_ptr->pose.pose, odom_affine);
    odom_pose_matrix_ = odom_affine.matrix().cast<float>();
    has_odom_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received odometry pose");
}

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

    // align
    Eigen::Matrix4f initial_pose_matrix;
    // 优先使用里程计的位姿作为初始矩阵
    if (has_odom_) {
        initial_pose_matrix = odom_pose_matrix_;
        RCLCPP_DEBUG(this->get_logger(), "Using odometry as initial pose");
    } else {
        if (!init_pose){
            Eigen::Affine3d initial_pose_affine;
            tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
            initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
            pre_trans = initial_pose_matrix;
            init_pose = true;
            RCLCPP_INFO(this->get_logger(), "Using initial pose from topic");
        } else {
            initial_pose_matrix = pre_trans * delta_trans;
            RCLCPP_DEBUG(this->get_logger(), "Using previous pose as initial pose");
        }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    const auto align_start_time = std::chrono::system_clock::now();
    key_value_stdmap_["state"] = "Aligning";
    ndt_->align(*output_cloud, initial_pose_matrix);
    key_value_stdmap_["state"] = "Sleeping";
    const auto align_end_time = std::chrono::system_clock::now();
    const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;
    Eigen::Matrix4f result_pose_matrix = ndt_->getFinalTransformation();
    Eigen::Affine3d result_pose_affine;
    result_pose_affine.matrix() = result_pose_matrix.cast<double>();
    const geometry_msgs::msg::Pose result_pose_msg = tf2::toMsg(result_pose_affine);
    const auto exe_end_time = std::chrono::system_clock::now();
    const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;
    float transform_probability = ndt_->getTransformationProbability();
    int iteration_num = ndt_->getFinalNumIteration();
    bool is_converged = true;
    static size_t skipping_publish_num = 0;

    // 判断是否收敛
    if (
        iteration_num >= ndt_->getMaximumIterations() + 2 ||
        transform_probability < converged_param_transform_probability_) {
        is_converged = false;
        ++skipping_publish_num;
        fail_count_++;
        RCLCPP_INFO(this->get_logger(), "Not Converged, fail count: %d", fail_count_);
    } else {
        skipping_publish_num = 0;
        fail_count_ = 0; // 成功，重置失败计数器
    }

    // 如果多次配准失败，尝试10个随机初始位姿
    if (fail_count_ >= 3) {
        RCLCPP_INFO(this->get_logger(), "多次配准失败，尝试10个随机初始位姿");
        float best_prob = transform_probability;
        Eigen::Matrix4f best_pose = result_pose_matrix;
        pcl::PointCloud<pcl::PointXYZ>::Ptr best_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        *best_cloud = *output_cloud;

        // 生成10个随机初始位姿
        for (int i = 0; i < 10; i++) {
            // 在地图范围内随机生成位置
            float x = map_min_[0] + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (map_max_[0] - map_min_[0]);
            float y = map_min_[1] + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (map_max_[1] - map_min_[1]);
            float z = map_min_[2] + static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * (map_max_[2] - map_min_[2]);
            // 随机生成旋转（绕z轴的yaw）
            float yaw = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 2 * M_PI;
            Eigen::AngleAxisf roll_angle(0.0, Eigen::Vector3f::UnitX());
            Eigen::AngleAxisf pitch_angle(0.0, Eigen::Vector3f::UnitY());
            Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitZ());
            Eigen::Affine3f pose_affine;
            pose_affine.linear() = (yaw_angle * pitch_angle * roll_angle).toRotationMatrix();
            pose_affine.translation() = Eigen::Vector3f(x, y, z);
            Eigen::Matrix4f test_pose = pose_affine.matrix();

            pcl::PointCloud<pcl::PointXYZ>::Ptr test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            ndt_->align(*test_cloud, test_pose);
            float prob = ndt_->getTransformationProbability();
            RCLCPP_INFO(this->get_logger(), "尝试第%d个初始位姿，概率：%f", i+1, prob);

            if (prob > best_prob) {
                best_prob = prob;
                best_pose = ndt_->getFinalTransformation();
                *best_cloud = *test_cloud;
            }
        }

        // 使用最优的结果
        result_pose_matrix = best_pose;
        *output_cloud = *best_cloud;
        transform_probability = best_prob;
        iteration_num = ndt_->getFinalNumIteration();
        is_converged = (best_prob >= converged_param_transform_probability_);
        if (is_converged) {
            fail_count_ = 0; // 成功，重置失败计数器
            RCLCPP_INFO(this->get_logger(), "随机初始位姿配准成功，最优概率：%f", best_prob);
        } else {
            RCLCPP_INFO(this->get_logger(), "随机初始位姿配准仍失败，最优概率：%f", best_prob);
        }
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

    // publish
    geometry_msgs::msg::PoseStamped result_pose_stamped_msg;
    result_pose_stamped_msg.header.stamp = sensor_ros_time;
    result_pose_stamped_msg.header.frame_id = map_frame_;
    result_pose_stamped_msg.pose = result_pose_msg;
    if (is_converged) {
        ndt_pose_pub_->publish(result_pose_stamped_msg);
    }
    // publish tf(map frame to base frame)
    publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);
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
    std_msgs::msg::Float32 exe_time_msg;
    exe_time_msg.data = exe_time;
    exe_time_pub_->publish(exe_time_msg);
    std_msgs::msg::Float32 transform_probability_msg;
    transform_probability_msg.data = transform_probability;
    transform_probability_pub_->publish(transform_probability_msg);
    std_msgs::msg::Float32 iteration_num_msg;
    iteration_num_msg.data = iteration_num;
    iteration_num_pub_->publish(iteration_num_msg);
    key_value_stdmap_["transform_probability"] = std::to_string(transform_probability);
    key_value_stdmap_["iteration_num"] = std::to_string(iteration_num);
    key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);
    key_value_stdmap_["fail_count"] = std::to_string(fail_count_);
    RCLCPP_INFO(this->get_logger(), "------------------------------------------------");
    RCLCPP_INFO(this->get_logger(), "align_time: %fms", align_time);
    RCLCPP_INFO(this->get_logger(), "exe_time: %fms", exe_time);
    RCLCPP_INFO(this->get_logger(), "trans_prob: %f", transform_probability);
    RCLCPP_INFO(this->get_logger(), "iter_num: %d", iteration_num);
    RCLCPP_INFO(this->get_logger(), "skipping_publish_num: %zu", skipping_publish_num);
    RCLCPP_INFO(this->get_logger(), "fail_count: %d", fail_count_);
}

void NdtLocalizerNode::init_params(){
    // 声明参数
    this->declare_parameter<std::string>("base_frame", "base_link");
    this->declare_parameter<double>("trans_epsilon", 0.01);
    this->declare_parameter<double>("step_size", 0.1);
    this->declare_parameter<double>("resolution", 1.0);
    this->declare_parameter<int>("max_iterations", 30);
    this->declare_parameter<double>("converged_param_transform_probability", 0.95);
    base_frame_ = this->get_parameter("base_frame").as_string();
    RCLCPP_INFO(this->get_logger(), "base_frame_id: %s", base_frame_.c_str());
    double trans_epsilon = this->get_parameter("trans_epsilon").as_double();
    double step_size = this->get_parameter("step_size").as_double();
    double resolution = this->get_parameter("resolution").as_double();
    int max_iterations = this->get_parameter("max_iterations").as_int();
    map_frame_ = "map";
    ndt_->setTransformationEpsilon(trans_epsilon);
    ndt_->setStepSize(step_size);
    ndt_->setResolution(resolution);
    ndt_->setMaximumIterations(max_iterations);
    RCLCPP_INFO(this->get_logger(),
                "trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
                trans_epsilon, step_size, resolution, max_iterations);
    converged_param_transform_probability_ = this->get_parameter("converged_param_transform_probability").as_double();
}

bool NdtLocalizerNode::get_transform(
    const std::string & target_frame, const std::string & source_frame,
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


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NdtLocalizerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}