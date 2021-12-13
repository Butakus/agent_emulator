#include "noisy_localization/noisy_localization.hpp"

namespace noisy_localization
{

NoisyLocalization::NoisyLocalization() :
    rclcpp::Node("noisy_localization"),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyLocalization::NoisyLocalization(const rclcpp::NodeOptions& options) :
    rclcpp::Node("noisy_localization", options),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyLocalization::~NoisyLocalization()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void NoisyLocalization::init()
{
    // Initialize and declare parameters
    this->position_stddev_ = this->declare_parameter("position_stddev", 2.0);
    this->orientation_stddev_ = this->declare_parameter("orientation_stddev", deg_to_rad(0.5));
    this->velocity_linear_stddev_ = this->declare_parameter("velocity_linear_stddev", 1.0);
    this->velocity_angular_stddev_ = this->declare_parameter("velocity_angular_stddev", deg_to_rad(0.1));
    this->rate_ = this->declare_parameter("rate", 20.0);

    // Initialize gaussian random distributions
    this->position_gauss_distribution_ = std::normal_distribution<>(0.0, this->position_stddev_);
    this->orientation_gauss_distribution_ = std::normal_distribution<>(0.0, this->orientation_stddev_);
    this->velocity_linear_gauss_distribution_ = std::normal_distribution<>(0.0, this->velocity_linear_stddev_);
    this->velocity_angular_gauss_distribution_ = std::normal_distribution<>(0.0, this->velocity_angular_stddev_);

    // Publisher / Subscribers
    this->odom_pub_ = this->create_publisher<Odometry>("noisy_odom", rclcpp::SensorDataQoS());
    using std::placeholders::_1;
    this->pose_sub_ = this->create_subscription<Pose>(
        "pose", rclcpp::SensorDataQoS(),
        std::bind(&NoisyLocalization::pose_callback, this, _1));
    this->velocity_sub_ = this->create_subscription<Twist>(
        "velocity", rclcpp::SensorDataQoS(),
        std::bind(&NoisyLocalization::velocity_callback, this, _1));

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&NoisyLocalization::run, this);
}

void NoisyLocalization::run()
{
    this->running_ = true;
    std::unique_lock<std::mutex> state_lock(this->state_mutex_, std::defer_lock);

    rclcpp::Rate rate(this->rate_);
    // TODO: Wait until pose is received
    while (rclcpp::ok() && this->running_)
    {
        Odometry::UniquePtr odom_msg = std::make_unique<Odometry>();
        // Lock mutex while accessing current_pose_ and current_velocity_
        state_lock.lock();
        if (this->current_pose_ != nullptr)
        {
            // Set header and pose from input pose
            odom_msg->header = this->current_pose_->header;
            odom_msg->pose.pose = this->current_pose_->pose;
            if (this->current_velocity_ != nullptr)
            {
                // Set velocity and child_frame_id from input velocity
                odom_msg->child_frame_id = this->current_velocity_->header.frame_id;
                odom_msg->twist.twist = this->current_velocity_->twist;
            }
        }
        state_lock.unlock();

        // Add noise and publish
        odom_msg->pose.pose.position.x += this->position_gauss_distribution_(this->random_generator_);
        odom_msg->pose.pose.position.y += this->position_gauss_distribution_(this->random_generator_);
        double yaw = yaw_from_quaternion<double>(odom_msg->pose.pose.orientation) + this->orientation_gauss_distribution_(this->random_generator_);
        odom_msg->pose.pose.orientation = quaternion_msg_from_yaw(yaw);

        odom_msg->twist.twist.linear.x += this->velocity_linear_gauss_distribution_(this->random_generator_);
        odom_msg->twist.twist.angular.z += this->velocity_angular_gauss_distribution_(this->random_generator_);
        
        // X covariance
        odom_msg->pose.covariance[0] = this->position_stddev_ * this->position_stddev_;
        odom_msg->twist.covariance[0] = this->velocity_linear_stddev_ * this->velocity_linear_stddev_;
        // Y covariance
        odom_msg->pose.covariance[7] = odom_msg->pose.covariance[0];
        // Yaw covariance
        odom_msg->pose.covariance[35] = this->orientation_stddev_ * this->orientation_stddev_;
        odom_msg->twist.covariance[35] = this->velocity_angular_stddev_ * this->velocity_angular_stddev_;

        this->odom_pub_->publish(std::move(odom_msg));

        rate.sleep();
    }
}

void NoisyLocalization::pose_callback(const Pose::SharedPtr pose_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->current_pose_ = pose_msg;
}

void NoisyLocalization::velocity_callback(const Twist::SharedPtr twist_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->current_velocity_ = twist_msg;
}


}  // namespace noisy_localization

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(noisy_localization::NoisyLocalization)
