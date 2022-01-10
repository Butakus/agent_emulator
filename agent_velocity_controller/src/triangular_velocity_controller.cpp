#include "agent_velocity_controller/triangular_velocity_controller.hpp"

namespace agent_velocity_controller
{

TriangularVelocityController::TriangularVelocityController() : rclcpp::Node("agent_velocity_controller")
{
    this->init();
}

TriangularVelocityController::TriangularVelocityController(const rclcpp::NodeOptions& options) : rclcpp::Node("triangular_velocity_controller", options)
{
    this->init();
}



TriangularVelocityController::~TriangularVelocityController()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void TriangularVelocityController::init()
{
    // Initialize and declare parameters
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);
    this->acceleration_ = this->declare_parameter("acceleration", 1.0, read_only_descriptor);
    this->period_ = this->declare_parameter("period", 10.0, read_only_descriptor);
    this->offset_ = this->declare_parameter("offset", 4.0, read_only_descriptor);

    // Publisher
    this->velocity_pub_ = this->create_publisher<Twist>("target_velocity", 5);

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&TriangularVelocityController::run, this);
}

void TriangularVelocityController::run()
{
    this->running_ = true;

    rclcpp::Rate rate(this->rate_);
    while (rclcpp::ok() && this->running_)
    {
        auto now = this->now();
        Twist::UniquePtr velocity = std::make_unique<Twist>();
        double signal_dt = std::fmod(now.seconds(), this->period_);
        if (signal_dt < this->period_ / 2.0)
        {
            // Acceleration
            velocity->linear.x = this->offset_ + (this->acceleration_ * signal_dt);
        }
        else
        {
            // Deceleration
            velocity->linear.x = this->offset_ + (this->acceleration_ * (this->period_ - signal_dt));
        }
        // Publish the velocity command
        this->velocity_pub_->publish(std::move(velocity));
        rate.sleep();
    }
}

}  // namespace agent_velocity_controller

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(agent_velocity_controller::TriangularVelocityController)
