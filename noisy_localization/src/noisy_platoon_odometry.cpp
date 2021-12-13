#include "noisy_localization/noisy_platoon_odometry.hpp"

namespace noisy_localization
{

NoisyPlatoonOdometry::NoisyPlatoonOdometry() :
    rclcpp::Node("noisy_platoon_localization"),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyPlatoonOdometry::NoisyPlatoonOdometry(const rclcpp::NodeOptions& options) :
    rclcpp::Node("noisy_platoon_localization", options),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoisyPlatoonOdometry::~NoisyPlatoonOdometry()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}


void NoisyPlatoonOdometry::init()
{
    using std::placeholders::_1;
    // Initialize and declare parameters
    this->velocity_stddev_ = this->declare_parameter("velocity_stddev", 0.01);

    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    std::string odom_frame_id = this->declare_parameter("odom_frame_id", "odom", read_only_descriptor);
    this->publish_tf_ = this->declare_parameter("publish_tf", false, read_only_descriptor);
    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Set callback to handle parameter setting
    this->set_param_callback_handler_ = this->add_on_set_parameters_callback(
                                                std::bind(&NoisyPlatoonOdometry::set_param_callback, this, _1));
    // Initialize gaussian random distributions
    this->velocity_gauss_distribution_ = std::normal_distribution<>(0.0, this->velocity_stddev_);

    // Initialize current Odometry
    this->odom_ = std::make_shared<Odometry>();
    this->odom_->header.stamp.sec = 0;
    this->odom_->header.stamp.nanosec = 0;
    this->odom_->header.frame_id = odom_frame_id;

    // Initialize TF broadcaster
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Publisher / Subscriber
    this->odom_pub_ = this->create_publisher<Odometry>("noisy_odom", rclcpp::SensorDataQoS());
    this->velocity_sub_ = this->create_subscription<Twist>(
        "velocity", rclcpp::SensorDataQoS(),
        std::bind(&NoisyPlatoonOdometry::velocity_callback, this, _1));

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&NoisyPlatoonOdometry::run, this);
}

void NoisyPlatoonOdometry::run()
{
    this->running_ = true;

    rclcpp::Rate rate(this->rate_);
    while (rclcpp::ok() && this->running_)
    {
        if (this->current_velocity_ != nullptr)
        {
            // Lock velocity mutex
            std::lock_guard<std::mutex> lock(this->state_mutex_);

            // Compute velocity in current period (average between last and current velocity)
            // double velocity = (this->current_velocity_->twist.linear.x + this->odom_->twist.twist.linear.x) / 2.0;
            double velocity = this->current_velocity_->twist.linear.x;
            // Add noise to velocity
            velocity += this->velocity_gauss_distribution_(this->random_generator_);
            // Compute position delta
            double delta_t = (rclcpp::Time(this->current_velocity_->header.stamp) - rclcpp::Time(this->odom_->header.stamp)).seconds();
            this->odom_->pose.pose.position.x += velocity * delta_t;
            this->odom_->twist.twist.linear.x = velocity;
            // TODO: Figure out position stddev. It should not be constant, and depend on velocity and time rate
            this->odom_->pose.covariance[0] = this->velocity_stddev_ * this->velocity_stddev_;
            this->odom_->twist.covariance[0] = this->velocity_stddev_ * this->velocity_stddev_;
            this->odom_->header.stamp = this->current_velocity_->header.stamp;
            // Publish new odometry
            this->odom_pub_->publish(*this->odom_);

            // Broadcast odom -> base_link transform
            if (this->publish_tf_)
            {
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = this->odom_->header.stamp;
                tf.header.frame_id = this->odom_->header.frame_id;
                tf.child_frame_id = this->odom_->child_frame_id;
                tf.transform.translation.x = this->odom_->pose.pose.position.x;
                tf.transform.rotation = this->odom_->pose.pose.orientation;
                this->tf_broadcaster_->sendTransform(tf);
            }
        }
        rate.sleep();
    }
}

rcl_interfaces::msg::SetParametersResult NoisyPlatoonOdometry::set_param_callback(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Set param " << param.get_name() << ": " << param.value_to_string());
        if (param.get_name() == "velocity_stddev")
        {
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

void NoisyPlatoonOdometry::velocity_callback(const Twist::SharedPtr twist_msg)
{
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    // If first time, init current odom
    if (this->current_velocity_ == nullptr)
    {
        this->odom_->header.stamp = twist_msg->header.stamp;
        this->odom_->child_frame_id = twist_msg->header.frame_id;
        this->odom_->twist.twist.linear.x = twist_msg->twist.linear.x;
    }
    this->current_velocity_ = twist_msg;
}

}  // namespace noisy_localization

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(noisy_localization::NoisyPlatoonOdometry)
