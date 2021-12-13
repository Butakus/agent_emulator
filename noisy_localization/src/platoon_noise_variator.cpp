#include "noisy_localization/platoon_noise_variator.hpp"

namespace noisy_localization
{

PlatoonNoiseVariator::PlatoonNoiseVariator() :
    rclcpp::Node("platoon_noise_variator"),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

PlatoonNoiseVariator::PlatoonNoiseVariator(const rclcpp::NodeOptions& options) :
    rclcpp::Node("platoon_noise_variator", options),
    // random_generator_(rd_())
    random_generator_(42)
{
    this->init();
}

PlatoonNoiseVariator::~PlatoonNoiseVariator()
{
    this->running_ = false;
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void PlatoonNoiseVariator::init()
{
    // Initialize and declare parameters
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    this->target_localization_node_ = this->declare_parameter("target_localization_node", "", read_only_descriptor);
    this->noise_parameter_name_ = this->declare_parameter("noise_parameter_name", "", read_only_descriptor);
    this->noise_stddev_ = this->declare_parameter("noise_stddev", 1.0, read_only_descriptor);
    this->variation_period_ = this->declare_parameter("variation_period", 2.0, read_only_descriptor);
    this->variation_probability_ = this->declare_parameter("variation_probability", 0.2, read_only_descriptor);
    this->tunnel_mean_stddev_ = this->declare_parameter("tunnel_mean_stddev_", 0.0, read_only_descriptor);
    this->tunnel_start_ = this->declare_parameter("tunnel_start", 0.0, read_only_descriptor);
    this->tunnel_end_ = this->declare_parameter("tunnel_end", 0.0, read_only_descriptor);
    this->rate_ = this->declare_parameter("rate", 20.0, read_only_descriptor);

    // Check target node parameter
    if (this->target_localization_node_ == "")
    {
        RCLCPP_ERROR(this->get_logger(), "target_localization_node parameter not set!");
        rclcpp::shutdown();
        return;
    }

    // Initialize gaussian and uniform random distributions
    this->noise_gauss_distribution_ = std::normal_distribution<>(0.0, this->noise_stddev_);
    this->uniform_distribution_ = std::uniform_real_distribution<>(0.0, 1.0);

    // Parameter client
    this->parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, this->target_localization_node_);
    // Subscriber
    using std::placeholders::_1;
    this->pose_sub_ = this->create_subscription<Pose>(
        "pose", rclcpp::SensorDataQoS(),
        std::bind(&PlatoonNoiseVariator::pose_callback, this, _1));

    // Start the main execution thread
    this->executor_thread_ = std::thread(&PlatoonNoiseVariator::run, this);
}

void PlatoonNoiseVariator::run()
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
    this->initial_stddev_ = -1.0;
    this->get_noise_param();

    // Wait until pose is received and initial_stddev param is read
    while (rclcpp::ok() && this->running_ &&
           this->current_pose_ != nullptr &&
           this->initial_stddev_ < 0.0)
    {
        rate.sleep();
    }

    if (this->tunnel_mean_stddev_ == 0.0) this->tunnel_mean_stddev_ = this->initial_stddev_;

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
                double base_noise = this->initial_stddev_;
                if (this->current_pose_->pose.position.x > this->tunnel_start_ &&
                    this->current_pose_->pose.position.x < this->tunnel_end_ )
                {
                    // If inside the tunnel, use the tunnel stddev as noise mean
                    base_noise = this->tunnel_mean_stddev_;
                }
                // If there was a tunnel event, reset the flag
                if (this->force_tunnel_update_) this->force_tunnel_update_ = false;
                state_lock.unlock();
                // Compute the new std and set the noise parameter
                double stddev = base_noise + this->noise_gauss_distribution_(this->random_generator_);
                this->set_noise_param(stddev);
            }
            last_update = now;
        }
        rate.sleep();
    }
}

void PlatoonNoiseVariator::get_noise_param()
{
    using ParametersFuture = std::shared_future<std::vector<rclcpp::Parameter> >;
    std::function<void(ParametersFuture)> param_callback = [this](ParametersFuture parameters_future)
    {
        // Read parameter from future result
        std::vector<rclcpp::Parameter> parameters = parameters_future.get();
        assert(parameters.size() == 1);
        assert(parameters[0].get_name() == this->noise_parameter_name_);
        this->initial_stddev_ = parameters[0].as_double();
    };
    this->parameters_client_->get_parameters({this->noise_parameter_name_}, param_callback);
}

void PlatoonNoiseVariator::set_noise_param(double stddev)
{
    // Force stddev to be positive (in case initial value was very small)
    stddev = std::abs(stddev);
    // Set noise parameter
    std::vector<rclcpp::Parameter> params = {rclcpp::Parameter(this->noise_parameter_name_, stddev)};
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

void PlatoonNoiseVariator::pose_callback(const Pose::SharedPtr pose_msg)
{
    // Lock mutex
    std::lock_guard<std::mutex> lock(this->state_mutex_);
    // The first time, just update the pose
    if (this->current_pose_ == nullptr)
    {
        this->current_pose_ = pose_msg;
        return;
    }
    // Check tunnel event
    if ((this->current_pose_->pose.position.x < this->tunnel_start_ && pose_msg->pose.position.x > this->tunnel_start_) ||
        (this->current_pose_->pose.position.x < this->tunnel_end_ && pose_msg->pose.position.x > this->tunnel_end_)     ||
        (this->current_pose_->pose.position.x > this->tunnel_start_ && pose_msg->pose.position.x < this->tunnel_start_) ||
        (this->current_pose_->pose.position.x > this->tunnel_end_ && pose_msg->pose.position.x < this->tunnel_end_))
    {
        this->force_tunnel_update_ = true;
    }
    this->current_pose_ = pose_msg;
}

}  // namespace noisy_localization

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(noisy_localization::PlatoonNoiseVariator)
