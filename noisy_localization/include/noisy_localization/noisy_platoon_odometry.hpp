#ifndef NOISY_LOCALIZATION__NOISY_PLATOON_ODOMETRY_HPP_
#define NOISY_LOCALIZATION__NOISY_PLATOON_ODOMETRY_HPP_

#include <random>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <noisy_localization/utils.hpp>

namespace noisy_localization
{

class NoisyPlatoonOdometry : public rclcpp::Node
{
public:
    using Twist = geometry_msgs::msg::TwistStamped;
    using Odometry = nav_msgs::msg::Odometry;

    // Smartpointer typedef
    typedef std::shared_ptr<NoisyPlatoonOdometry> SharedPtr;
    typedef std::unique_ptr<NoisyPlatoonOdometry> UniquePtr;

    NoisyPlatoonOdometry();
    NoisyPlatoonOdometry(const rclcpp::NodeOptions& options);
    virtual ~NoisyPlatoonOdometry();

protected:
    // Parameters
    double velocity_stddev_;
    double rate_;
    bool publish_tf_;
    OnSetParametersCallbackHandle::SharedPtr set_param_callback_handler_;

    // State
    Twist::SharedPtr current_velocity_;
    Odometry::SharedPtr odom_;
    std::mutex state_mutex_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;

    // TF2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Random engine
    std::random_device rd_;
    std::mt19937 random_generator_;
    // Gaussian random distributions
    std::normal_distribution<> velocity_gauss_distribution_;

    // Publisher / Subscriber
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<Twist>::SharedPtr velocity_sub_;

    void init();
    void run();

    // Param callback
    rcl_interfaces::msg::SetParametersResult set_param_callback(const std::vector<rclcpp::Parameter>& params);

    // Callbacks
    void velocity_callback(const Twist::SharedPtr twist_msg);
};


}  // namespace noisy_localization

#endif  // NOISY_LOCALIZATION__NOISY_PLATOON_ODOMETRY_HPP_
