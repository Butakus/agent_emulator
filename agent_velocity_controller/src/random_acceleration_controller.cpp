#include "agent_velocity_controller/random_acceleration_controller.hpp"

namespace agent_velocity_controller
{

RandomAccelerationController::RandomAccelerationController() :
    rclcpp::Node("agent_velocity_controller"),
    random_generator_(rd_())
{
    this->init();
}

RandomAccelerationController::RandomAccelerationController(const rclcpp::NodeOptions& options) :
    rclcpp::Node("random_velocity_controller", options),
    random_generator_(rd_())
{
    this->init();
}

RandomAccelerationController::~RandomAccelerationController()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void RandomAccelerationController::init()
{
    // Initialize and declare parameters
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    double min_velocity = this->declare_parameter("min_velocity", 3.0, read_only_descriptor);
    double max_velocity = this->declare_parameter("max_velocity", 5.0, read_only_descriptor);
    double acceleration_stddev = this->declare_parameter("acceleration_stddev", 1.0, read_only_descriptor);
    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Adjust acceleration sign depending on the position of the current velocity inside the permitted velocity range
    double velocity_range = max_velocity - min_velocity;
    this->min_velocity_threshold_ = min_velocity + (0.1 * velocity_range);
    this->max_velocity_threshold_ = max_velocity - (0.1 * velocity_range);

    // Initialize gaussian random distributions
    this->acceleration_gauss_distribution_ = std::normal_distribution<>(0.0, acceleration_stddev);

    // Publisher
    this->acceleration_pub_ = this->create_publisher<Accel>("target_acceleration", 5);
    // Subscriber
    using std::placeholders::_1;
    this->velocity_sub_ = this->create_subscription<TwistStamped>(
        "velocity", rclcpp::SensorDataQoS(),
        std::bind(&RandomAccelerationController::velocity_callback, this, _1));

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&RandomAccelerationController::run, this);
}

void RandomAccelerationController::run()
{
    this->running_ = true;
    rclcpp::Rate rate(this->rate_);

    // First, wait until velocity is received
    while (rclcpp::ok() && this->running_ && this->velocity_ == nullptr)
    {
        rate.sleep();
    }

    while (rclcpp::ok() && this->running_)
    {
        // Compute a new random velocity
        auto now = this->now();
        // compute new acceleration
        double acceleration = this->compute_acceleration();
        Accel::UniquePtr a = std::make_unique<Accel>();
        a->linear.x = acceleration;
        this->acceleration_pub_->publish(std::move(a));
        rate.sleep();
    }
}

/* Compute acceleration values based on current velocity and velocity ranges */
double RandomAccelerationController::compute_acceleration()
{
    // Initial random acceleration value
    double acceleration = this->acceleration_gauss_distribution_(this->random_generator_);

    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);

    // If velocity is close to the range limit, force the acceleration command to move it towards the center
    if ((this->velocity_->twist.linear.x < this->min_velocity_threshold_ && acceleration < 0.0) ||
        (this->velocity_->twist.linear.x > this->max_velocity_threshold_ && acceleration > 0.0))
    {
        acceleration = - acceleration;
    }

    return acceleration;
}

void RandomAccelerationController::velocity_callback(const TwistStamped::SharedPtr twist_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    this->velocity_ = twist_msg;
}

}  // namespace agent_velocity_controller

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(agent_velocity_controller::RandomAccelerationController)
