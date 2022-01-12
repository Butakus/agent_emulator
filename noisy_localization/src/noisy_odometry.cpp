#include "noisy_localization/noisy_odometry.hpp"

namespace noisy_localization
{

NoisyOdometry::NoisyOdometry() :
    rclcpp::Node("noisy_localization"),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyOdometry::NoisyOdometry(const rclcpp::NodeOptions& options) :
    rclcpp::Node("noisy_localization", options),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyOdometry::~NoisyOdometry()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void NoisyOdometry::init()
{
    // Initialize and declare parameters
    this->position_drift_noise_ = this->declare_parameter("position_drift_noise", 0.01);
    this->position_drift_vel_rate_ = this->declare_parameter("position_drift_vel_rate", 0.02);
    this->orientation_drift_noise_ = this->declare_parameter("orientation_drift_noise", 0.05);
    this->orientation_drift_vel_rate_ = this->declare_parameter("orientation_drift_vel_rate", 0.02);
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Publisher / Subscribers
    this->odom_pub_ = this->create_publisher<Odometry>("noisy_odom", rclcpp::SensorDataQoS());
    using std::placeholders::_1;
    this->odom_sub_ = this->create_subscription<Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&NoisyOdometry::odom_callback, this, _1));

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&NoisyOdometry::run, this);
}

void NoisyOdometry::run()
{
    this->running_ = true;
    std::unique_lock<std::mutex> state_lock(this->state_mutex_, std::defer_lock);

    rclcpp::Rate rate(this->rate_);
    // Wait until 2 odometries are received
    while (rclcpp::ok() && this->running_ && this->last_odom_ == nullptr && this->current_odom_ == nullptr)
    {
        rate.sleep();
    }

    while (rclcpp::ok() && this->running_)
    {
        Odometry::UniquePtr odom_msg = std::make_unique<Odometry>();
        // Lock mutex while accessing current_odom_ and last_odom_
        state_lock.lock();
        // Set header from current odom
        odom_msg->header = this->current_odom_->header;
        odom_msg->child_frame_id = this->current_odom_->child_frame_id;
        // Set position data from previous reading and velocity data from current reading
        odom_msg->pose = last_odom_->pose;
        odom_msg->twist = current_odom_->twist;
        // Compute the pose delta between the last 2 odometries
        Pose odom_delta = relative_transform(this->last_odom_->pose.pose, this->current_odom_->pose.pose);
        state_lock.unlock();

        // Add noise to odometry delta and reconstruct the current odometry by adding the noisy delta to the last position
        this->add_noise(odom_msg, odom_delta);

        this->odom_pub_->publish(std::move(odom_msg));

        rate.sleep();
    }
}

Eigen::Matrix<double, 2, 2> NoisyOdometry::get_noise_covariance(const Pose& odom_delta)
{
    // Compute noise std based on current velocity and distance covered
    double velocity = std::hypot(this->current_odom_->twist.twist.linear.x, this->current_odom_->twist.twist.linear.y);
    double delta_distance = std::hypot(odom_delta.position.x, odom_delta.position.y);

    double base_noise_std = this->position_drift_noise_ * delta_distance;
    double base_noise_var = base_noise_std * base_noise_std;

    double velocity_noise_std = this->position_drift_vel_rate_ * velocity;
    double velocity_noise_var = velocity_noise_std * velocity_noise_std;

    double forward_noise_var = base_noise_var + velocity_noise_var;

    Eigen::Matrix<double, 2, 2> noise_covariance;
    noise_covariance << forward_noise_var, 0,
                         0, base_noise_var;

    // Rotate covariance
    double angle = std::atan2(odom_delta.position.y, odom_delta.position.x);
    Eigen::Matrix<double, 2, 2> R;
    R << std::cos(angle), -std::sin(angle),
         std::sin(angle), std::cos(angle);

    Eigen::Matrix<double, 2, 2> rotated_cov = R * noise_covariance * R.transpose();

    // return noise_covariance;
    return rotated_cov;
}

void NoisyOdometry::add_noise(const Odometry::UniquePtr& odom_msg, Pose& odom_delta)
{
    // Compute noise std based on current velocity and distance covered
    double velocity = std::hypot(this->current_odom_->twist.twist.linear.x, this->current_odom_->twist.twist.linear.y);
    double delta_distance = std::hypot(odom_delta.position.x, odom_delta.position.y);

    double orientation_std = this->orientation_drift_noise_ * delta_distance + this->orientation_drift_vel_rate_ * velocity;

    // Get translation noise covariance based and align covariance ellipse with direction of movement
    Eigen::Matrix<double, 2, 2> noise_covariance = this->get_noise_covariance(odom_delta);

    // Compute and add translation noise
    Eigen::Vector2d noise_mean {0.0, 0.0};
    MultivariateNormal<double, 2> mvn(noise_mean, noise_covariance);

    Eigen::Vector2d position_noise = mvn.sample(this->random_generator_);

    odom_delta.position.x += position_noise(0);
    odom_delta.position.y += position_noise(1);

    // Compute and add orientation noise
    std::normal_distribution<> orientation_gauss_distribution = std::normal_distribution<>(0.0, orientation_std);
    double yaw = yaw_from_quaternion<double>(odom_delta.orientation) + orientation_gauss_distribution(this->random_generator_);
    odom_delta.orientation = quaternion_msg_from_yaw(yaw);
    
    // Reconstruct noisy odometry by adding the noisy delta to the last position
    odom_msg->pose.pose = composed_pose(odom_msg->pose.pose, odom_delta);

    // Set msg covariance
    // x/y variance and covariances
    odom_msg->pose.covariance[0] = noise_covariance(0, 0);
    odom_msg->pose.covariance[1] = noise_covariance(0, 1);
    odom_msg->pose.covariance[6] = noise_covariance(1, 0);
    odom_msg->pose.covariance[7] = noise_covariance(1, 1);
    // yaw variance
    odom_msg->pose.covariance[35] = orientation_std * orientation_std;
}

void NoisyOdometry::odom_callback(const Odometry::SharedPtr odom_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->last_odom_ = this->current_odom_;
    this->current_odom_ = odom_msg;
}


}  // namespace noisy_localization

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(noisy_localization::NoisyOdometry)
