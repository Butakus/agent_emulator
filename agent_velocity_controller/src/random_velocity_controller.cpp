#include "agent_velocity_controller/random_velocity_controller.hpp"

namespace agent_velocity_controller
{

RandomVelocityController::RandomVelocityController() :
    rclcpp::Node("agent_velocity_controller"),
    random_generator_(rd_())
{
    this->init();
}

RandomVelocityController::RandomVelocityController(const rclcpp::NodeOptions& options) :
    rclcpp::Node("random_velocity_controller", options),
    random_generator_(rd_())
{
    this->init();
}



RandomVelocityController::~RandomVelocityController()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void RandomVelocityController::init()
{
    // Initialize and declare parameters
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    double velocity_mean = this->declare_parameter("velocity_mean", 1.0, read_only_descriptor);
    double velocity_stddev = this->declare_parameter("velocity_stddev", 1.0, read_only_descriptor);
    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Initialize gaussian random distributions
    this->velocity_gauss_distribution_ = std::normal_distribution<>(velocity_mean, velocity_stddev);

    // Publisher
    this->velocity_pub_ = this->create_publisher<Twist>("target_velocity", 5);

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&RandomVelocityController::run, this);
}

void RandomVelocityController::run()
{
    this->running_ = true;


    rclcpp::Rate rate(this->rate_);
    while (rclcpp::ok() && this->running_)
    {
    	// Compute a new random velocity
        Twist::UniquePtr velocity = std::make_unique<Twist>();
        velocity->header.stamp = this->now();
        velocity->twist.linear.x = this->velocity_gauss_distribution_(this->random_generator_);
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
RCLCPP_COMPONENTS_REGISTER_NODE(agent_velocity_controller::RandomVelocityController)
