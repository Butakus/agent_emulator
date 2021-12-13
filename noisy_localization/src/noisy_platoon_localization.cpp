#include "noisy_localization/noisy_platoon_localization.hpp"

namespace noisy_localization
{

NoisyPlatoonLocalization::NoisyPlatoonLocalization() :
    rclcpp::Node("noisy_platoon_localization"),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyPlatoonLocalization::NoisyPlatoonLocalization(const rclcpp::NodeOptions& options) :
    rclcpp::Node("noisy_platoon_localization", options),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyPlatoonLocalization::~NoisyPlatoonLocalization()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void NoisyPlatoonLocalization::init()
{
    using std::placeholders::_1;
    // Initialize and declare parameters
    this->position_stddev_ = this->declare_parameter("position_stddev", 2.0);
    this->velocity_stddev_ = this->declare_parameter("velocity_stddev", 1.0);

    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Set callback to handle parameter setting
    this->set_param_callback_handler_ = this->add_on_set_parameters_callback(
                                                std::bind(&NoisyPlatoonLocalization::set_param_callback, this, _1));
    // Initialize gaussian random distributions
    this->position_gauss_distribution_ = std::normal_distribution<>(0.0, this->position_stddev_);
    this->velocity_gauss_distribution_ = std::normal_distribution<>(0.0, this->velocity_stddev_);

    // Publisher / Subscribers
    this->odom_pub_ = this->create_publisher<Odometry>("prior_estimation", rclcpp::SensorDataQoS());
    this->pose_sub_ = this->create_subscription<Pose>(
        "pose", rclcpp::SensorDataQoS(),
        std::bind(&NoisyPlatoonLocalization::pose_callback, this, _1));
    this->velocity_sub_ = this->create_subscription<Twist>(
        "velocity", rclcpp::SensorDataQoS(),
        std::bind(&NoisyPlatoonLocalization::velocity_callback, this, _1));

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&NoisyPlatoonLocalization::run, this);
}

void NoisyPlatoonLocalization::run()
{
    this->running_ = true;
    std::unique_lock<std::mutex> state_lock(this->state_mutex_, std::defer_lock);

    rclcpp::Rate rate(this->rate_);
    // Wait until pose is received
    while (rclcpp::ok() && this->running_ && this->current_pose_ == nullptr)
    {
        rate.sleep();
    }

    while (rclcpp::ok() && this->running_)
    {
        Odometry::UniquePtr odom_msg = std::make_unique<Odometry>();
        // Lock mutex while accessing current_pose_ and current_velocity_
        state_lock.lock();
        // Set header and pose from input pose
        odom_msg->header = this->current_pose_->header;
        odom_msg->pose.pose = this->current_pose_->pose;
        if (this->current_velocity_ != nullptr)
        {
            // Set velocity and child_frame_id from input velocity
            odom_msg->child_frame_id = this->current_velocity_->header.frame_id;
            odom_msg->twist.twist = this->current_velocity_->twist;
        }
        state_lock.unlock();

        // Add noise and publish
        odom_msg->pose.pose.position.x += this->position_gauss_distribution_(this->random_generator_);
        odom_msg->twist.twist.linear.x += this->velocity_gauss_distribution_(this->random_generator_);
        
        // X covariance
        odom_msg->pose.covariance[0] = this->position_stddev_ * this->position_stddev_;
        odom_msg->twist.covariance[0] = this->velocity_stddev_ * this->velocity_stddev_;

        this->odom_pub_->publish(std::move(odom_msg));

        rate.sleep();
    }
}

rcl_interfaces::msg::SetParametersResult NoisyPlatoonLocalization::set_param_callback(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Set param " << param.get_name() << ": " << param.value_to_string());
        if (param.get_name() == "position_stddev")
        {
            std::lock_guard<std::mutex> lock(this->state_mutex_);
            this->position_stddev_ = param.as_double();
            this->position_gauss_distribution_ = std::normal_distribution<>(0.0, this->position_stddev_);
        }
        else if (param.get_name() == "velocity_stddev")
        {
            std::lock_guard<std::mutex> lock(this->state_mutex_);
            this->velocity_stddev_ = param.as_double();
            this->velocity_gauss_distribution_ = std::normal_distribution<>(0.0, this->velocity_stddev_);
        }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unknown param \"" << param.get_name() << "\". Skipping.");
        }
    }
    return result;
}

void NoisyPlatoonLocalization::pose_callback(const Pose::SharedPtr pose_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->current_pose_ = pose_msg;
}

void NoisyPlatoonLocalization::velocity_callback(const Twist::SharedPtr twist_msg)
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
RCLCPP_COMPONENTS_REGISTER_NODE(noisy_localization::NoisyPlatoonLocalization)
