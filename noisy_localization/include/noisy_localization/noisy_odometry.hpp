#ifndef NOISY_LOCALIZATION__NOISY_ODOMETRY_HPP_
#define NOISY_LOCALIZATION__NOISY_ODOMETRY_HPP_

#include <random>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <noisy_localization/utils.hpp>
#include <noisy_localization/mvn.hpp>

namespace noisy_localization
{

class NoisyOdometry : public rclcpp::Node
{
public:
    using Odometry = nav_msgs::msg::Odometry;
    using Pose = geometry_msgs::msg::Pose;

    // Smartpointer typedef
    typedef std::shared_ptr<NoisyOdometry> SharedPtr;
    typedef std::unique_ptr<NoisyOdometry> UniquePtr;

    NoisyOdometry();
    NoisyOdometry(const rclcpp::NodeOptions& options);
    virtual ~NoisyOdometry();

protected:
    // Parameters
    double position_drift_noise_; // Percentage (meters of std for each meter traveled)
    double position_drift_vel_rate_; // Seconds (meters of std for each meter/second of velocity)
    double orientation_drift_noise_;
    double orientation_drift_vel_rate_;
    double rate_;

    // State
    Odometry::SharedPtr current_odom_;
    Odometry::SharedPtr last_odom_;
    std::mutex state_mutex_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;

    // Random engine
    std::random_device rd_;
    std::mt19937 random_generator_;

    // Publishers / Subscriber
    rclcpp::Publisher<Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

    void init();
    void run();

    /* Get translation noise covariance based on velocity and
       align covariance ellipse with direction of movement
    */
    Eigen::Matrix<double, 2, 2> get_noise_covariance(const Pose& odom_delta);
    /* Add noise to odometry delta and reconstruct the current odometry
       by adding the noisy delta to the last position
    */
    void add_noise(const Odometry::UniquePtr& odom_msg, Pose& odom_delta);

    // Callbacks
    void odom_callback(const Odometry::SharedPtr odom_msg);
};


}  // namespace noisy_localization

#endif  // NOISY_LOCALIZATION__NOISY_ODOMETRY_HPP_
