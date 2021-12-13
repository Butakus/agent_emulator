#ifndef NOISY_LOCALIZATION__PLATOON_NOISE_VARIATOR_HPP_
#define NOISY_LOCALIZATION__PLATOON_NOISE_VARIATOR_HPP_

#include <random>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace noisy_localization
{

class PlatoonNoiseVariator : public rclcpp::Node
{
public:
    using Pose = geometry_msgs::msg::PoseStamped;

    // Smartpointer typedef
    typedef std::shared_ptr<PlatoonNoiseVariator> SharedPtr;
    typedef std::unique_ptr<PlatoonNoiseVariator> UniquePtr;

    PlatoonNoiseVariator();
    PlatoonNoiseVariator(const rclcpp::NodeOptions& options);
    virtual ~PlatoonNoiseVariator();

protected:
    // Parameters
    std::string target_localization_node_;
    std::string noise_parameter_name_;
    double noise_stddev_;
    double variation_period_;
    double variation_probability_;
    double tunnel_mean_stddev_;
    double tunnel_start_;
    double tunnel_end_;
    double rate_;

    // State
    Pose::SharedPtr current_pose_;
    double initial_stddev_;
    bool force_tunnel_update_;
    std::mutex state_mutex_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;

    // Random engine
    std::random_device rd_;
    std::mt19937 random_generator_;
    // Gaussian random distributions
    std::normal_distribution<> noise_gauss_distribution_;
    std::uniform_real_distribution<> uniform_distribution_;

    // Parameter client
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    // Publisher / Subscriber
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

    void init();
    void run();

    // Get/Set the value stored in the noise parameter
    void get_noise_param();
    void set_noise_param(double stddev);

    // Callbacks
    void pose_callback(const Pose::SharedPtr pose_msg);
};


}  // namespace noisy_localization

#endif  // NOISY_LOCALIZATION__PLATOON_NOISE_VARIATOR_HPP_
