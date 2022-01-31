#ifndef NOISY_LOCALIZATION__NOISY_LOCALIZATION_HPP_
#define NOISY_LOCALIZATION__NOISY_LOCALIZATION_HPP_

#include <random>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <noisy_localization/utils.hpp>

namespace noisy_localization
{

class NoisyLocalization : public rclcpp::Node
{
public:
    using Pose = geometry_msgs::msg::PoseStamped;
    using Twist = geometry_msgs::msg::TwistStamped;
    using Odometry = nav_msgs::msg::Odometry;

    // Smartpointer typedef
    typedef std::shared_ptr<NoisyLocalization> SharedPtr;
    typedef std::unique_ptr<NoisyLocalization> UniquePtr;

    NoisyLocalization();
    NoisyLocalization(const rclcpp::NodeOptions& options);
    virtual ~NoisyLocalization();

protected:
    // Parameters
    double position_stddev_;
    double orientation_stddev_;
    double velocity_linear_stddev_;
    double velocity_angular_stddev_;
    double rate_;
    OnSetParametersCallbackHandle::SharedPtr set_param_callback_handler_;

    // State
    Pose::SharedPtr current_pose_;
    Twist::SharedPtr current_velocity_;
    std::mutex state_mutex_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;

    // Random engine
    std::random_device rd_;
    std::mt19937 random_generator_;
    // Gaussian random distributions
    std::normal_distribution<> position_gauss_distribution_;
    std::normal_distribution<> orientation_gauss_distribution_;
    std::normal_distribution<> velocity_linear_gauss_distribution_;
    std::normal_distribution<> velocity_angular_gauss_distribution_;

    // Publishers / Subscriber
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_;
    rclcpp::Subscription<Twist>::SharedPtr velocity_sub_;

    void init();
    void run();

    // Param callback
    rcl_interfaces::msg::SetParametersResult set_param_callback(const std::vector<rclcpp::Parameter>& params);

    // Callbacks
    void pose_callback(const Pose::SharedPtr pose_msg);
    void velocity_callback(const Twist::SharedPtr twist_msg);
};


}  // namespace noisy_localization

#endif  // NOISY_LOCALIZATION__NOISY_LOCALIZATION_HPP_
