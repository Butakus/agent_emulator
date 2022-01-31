#include "noisy_localization/noise_variator.hpp"

namespace noisy_localization
{

NoiseVariator::NoiseVariator() :
    rclcpp::Node("noise_variator"),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoiseVariator::NoiseVariator(const rclcpp::NodeOptions& options) :
    rclcpp::Node("noise_variator", options),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

NoiseVariator::~NoiseVariator()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void NoiseVariator::init()
{
    // Initialize and declare parameters
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    this->target_localization_node_ = this->declare_parameter("target_localization_node", "", read_only_descriptor);
    this->position_noise_parameter_name_ = this->declare_parameter("position_noise_parameter_name", "", read_only_descriptor);
    this->orientation_noise_parameter_name_ = this->declare_parameter("orientation_noise_parameter_name", "", read_only_descriptor);
    this->position_noise_stddev_ = this->declare_parameter("position_noise_stddev", 1.0, read_only_descriptor);
    this->orientation_noise_stddev_ = this->declare_parameter("orientation_noise_stddev", 1.0, read_only_descriptor);
    this->variation_period_ = this->declare_parameter("variation_period", 2.0, read_only_descriptor);
    this->variation_probability_ = this->declare_parameter("variation_probability", 0.2, read_only_descriptor);
    this->tunnel_mean_stddev_ = this->declare_parameter("tunnel_mean_stddev_", 0.0, read_only_descriptor);
    this->tunnel_start_x_ = this->declare_parameter("tunnel_start_x", 0.0, read_only_descriptor);
    this->tunnel_start_y_ = this->declare_parameter("tunnel_start_y", 0.0, read_only_descriptor);
    this->tunnel_end_x_ = this->declare_parameter("tunnel_end_x", 0.0, read_only_descriptor);
    this->tunnel_end_y_ = this->declare_parameter("tunnel_end_y", 0.0, read_only_descriptor);
    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Check target node parameter
    if (this->target_localization_node_ == "")
    {
        RCLCPP_ERROR(this->get_logger(), "target_localization_node parameter not set!");
        rclcpp::shutdown();
        return;
    }

    // Initialize gaussian and uniform random distributions
    this->position_noise_gauss_distribution_ = std::normal_distribution<>(0.0, this->position_noise_stddev_);
    this->orientation_noise_gauss_distribution_ = std::normal_distribution<>(0.0, this->orientation_noise_stddev_);
    this->uniform_distribution_ = std::uniform_real_distribution<>(0.0, 1.0);

    // Parameter client
    this->parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->target_localization_node_);
    // Subscriber
    using std::placeholders::_1;
    this->pose_sub_ = this->create_subscription<Pose>(
        "pose", rclcpp::SensorDataQoS(),
        std::bind(&NoiseVariator::pose_callback, this, _1));

    // Start the main execution thread
    this->executor_thread_ = std::thread(&NoiseVariator::run, this);
}

void NoiseVariator::run()
{
    this->running_ = true;
    std::unique_lock<std::mutex> state_lock(this->state_mutex_, std::defer_lock);

    // Wait until parameter service is available
    rclcpp::Rate rate(this->rate_);
    while (rclcpp::ok() && this->running_ &&
           !this->parameters_client_->service_is_ready())
    {
        rate.sleep();
    }

    // Read initial parameter value
    this->initial_position_stddev_ = -1.0;
    this->initial_orientation_stddev_ = -1.0;
    this->get_noise_param(this->position_noise_parameter_name_, this->initial_position_stddev_);
    this->get_noise_param(this->orientation_noise_parameter_name_, this->initial_orientation_stddev_);

    // Wait until pose is received and initial noise params are received
    while (rclcpp::ok() && this->running_ &&
           this->current_pose_ != nullptr &&
           this->initial_position_stddev_ < 0.0 &&
           this->initial_orientation_stddev_ < 0.0)
    {
        rate.sleep();
    }

    if (this->tunnel_mean_stddev_ == 0.0) this->tunnel_mean_stddev_ = this->initial_position_stddev_;

    auto last_update = this->now();

    while (rclcpp::ok() && this->running_)
    {
        auto now = this->now();
        // Check if it is time to update the noise
        if ((now - last_update).seconds() > this->variation_period_ || this->force_tunnel_update_)
        {
            // If there is a tunnel event, update the stddev right away
            // Otherwise, check if the RNG gods allow the update
            if (this->force_tunnel_update_ || this->uniform_distribution_(this->random_generator_) < this->variation_probability_)
            {
                // Update noise parameter with a new random stddev
                state_lock.lock();
                double base_position_noise = this->initial_position_stddev_;
                if (this->tunnel_active_)
                {
                    // If inside the tunnel, use the tunnel stddev as noise mean
                    base_position_noise = this->tunnel_mean_stddev_;
                }
                // If there was a tunnel event, reset the flag
                if (this->force_tunnel_update_) this->force_tunnel_update_ = false;
                state_lock.unlock();
                // Compute the new stds and set the noise parameters
                double position_stddev = base_position_noise + this->position_noise_gauss_distribution_(this->random_generator_);
                this->set_noise_param(this->position_noise_parameter_name_, position_stddev);
                // Orientation noise is not affected by the tunnel
                double orientation_stddev = this->initial_orientation_stddev_ + this->orientation_noise_gauss_distribution_(this->random_generator_);
                this->set_noise_param(this->orientation_noise_parameter_name_, orientation_stddev);
            }
            last_update = now;
        }
        rate.sleep();
    }
}

void NoiseVariator::get_noise_param(std::string param_name, double& param)
{
    using ParametersFuture = std::shared_future<std::vector<rclcpp::Parameter> >;
    std::function<void(ParametersFuture)> param_callback = [this, &param](ParametersFuture parameters_future)
    {
        // Read parameter from future result
        std::vector<rclcpp::Parameter> parameters = parameters_future.get();
        assert(parameters.size() == 1);
        assert(parameters[0].get_name() == param_name);
        param = parameters[0].as_double();
    };
    this->parameters_client_->get_parameters({param_name}, param_callback);
}

void NoiseVariator::set_noise_param(std::string param_name, double stddev_param)
{
    // Force stddev to be positive (in case initial value was very small)
    stddev_param = std::abs(stddev_param);
    // Set noise parameter
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter(param_name, stddev_param)};
    // Create lambda callback to process the parameter future results
    using ResultsFuture = std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult> >;
    std::function<void(ResultsFuture)> param_callback = [this](ResultsFuture results_future)
    {
        std::vector<rcl_interfaces::msg::SetParametersResult> results = results_future.get();
        assert(results.size() == 1);
        if (!results[0].successful)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not set noise parameter");
        }
    };

    // Set parameter
    if (!this->parameters_client_->service_is_ready())
    {
        RCLCPP_ERROR(this->get_logger(), "Could not set noise parameter. Service not available.");
        return;
    }
    this->parameters_client_->set_parameters(params, param_callback);
}

bool NoiseVariator::check_tunnel()
{
    return (
        this->current_pose_->pose.position.x > this->tunnel_start_x_ &&
        this->current_pose_->pose.position.x < this->tunnel_end_x_ &&
        this->current_pose_->pose.position.y > this->tunnel_start_y_ &&
        this->current_pose_->pose.position.y < this->tunnel_end_y_
    );
}

void NoiseVariator::pose_callback(const Pose::SharedPtr pose_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    // The first time, just update the pose
    if (this->current_pose_ == nullptr)
    {
        this->current_pose_ = pose_msg;
        return;
    }
    // Update pose
    this->current_pose_ = pose_msg;
    // Check tunnel event
    bool inside_tunnel = this->check_tunnel();

    if (this->tunnel_active_ != inside_tunnel)
    {
        this->force_tunnel_update_ = true;
        this->tunnel_active_ = inside_tunnel;
    }
}

}  // namespace noisy_localization

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(noisy_localization::NoiseVariator)
