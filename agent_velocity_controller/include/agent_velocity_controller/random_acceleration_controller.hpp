#ifndef AGENT_VELOCITY_CONTROLLER__RANDOM_VELOCITY_CONTROLLER_HPP_
#define AGENT_VELOCITY_CONTROLLER__RANDOM_VELOCITY_CONTROLLER_HPP_

#include <random>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel.hpp>


namespace agent_velocity_controller
{

class RandomAccelerationController : public rclcpp::Node
{
public:
    using Accel = geometry_msgs::msg::Accel;
    using TwistStamped = geometry_msgs::msg::TwistStamped;

    // Smartpointer typedef
    typedef std::shared_ptr<RandomAccelerationController> SharedPtr;
    typedef std::unique_ptr<RandomAccelerationController> UniquePtr;

    RandomAccelerationController();
    RandomAccelerationController(const rclcpp::NodeOptions& options);
    virtual ~RandomAccelerationController();

protected:
    // Parameters
    double rate_;
    double min_velocity_threshold_;
    double max_velocity_threshold_;

    // State
    TwistStamped::SharedPtr velocity_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;
    std::mutex state_mutex_;

    // Random engine
    std::random_device rd_;
    std::mt19937 random_generator_;
    // Gaussian random distribution
    std::normal_distribution<> acceleration_gauss_distribution_;

    // Publisher
    rclcpp::Publisher<Accel>::SharedPtr acceleration_pub_;

    // Subscriber
    rclcpp::Subscription<TwistStamped>::SharedPtr velocity_sub_;

    void init();
    void run();

    /* Compute acceleration values based on current velocity and velocity ranges */
    double compute_acceleration();

    // Callback
    void velocity_callback(const TwistStamped::SharedPtr twist_msg);
};

}  // namespace agent_velocity_controller

#endif  // AGENT_VELOCITY_CONTROLLER__RANDOM_VELOCITY_CONTROLLER_HPP_
