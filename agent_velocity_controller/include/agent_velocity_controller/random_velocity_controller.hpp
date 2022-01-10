#ifndef AGENT_VELOCITY_CONTROLLER__RANDOM_VELOCITY_CONTROLLER_HPP_
#define AGENT_VELOCITY_CONTROLLER__RANDOM_VELOCITY_CONTROLLER_HPP_

#include <random>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/accel.hpp>


namespace agent_velocity_controller
{

class RandomVelocityController : public rclcpp::Node
{
public:
    using Twist = geometry_msgs::msg::Twist;

    // Smartpointer typedef
    typedef std::shared_ptr<RandomVelocityController> SharedPtr;
    typedef std::unique_ptr<RandomVelocityController> UniquePtr;

    RandomVelocityController();
    RandomVelocityController(const rclcpp::NodeOptions& options);
    virtual ~RandomVelocityController();

protected:
    // Parameters
    double rate_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;

    // Random engine
    std::random_device rd_;
    std::mt19937 random_generator_;
    // Gaussian random distribution
    std::normal_distribution<> velocity_gauss_distribution_;

    // Publisher
    rclcpp::Publisher<Twist>::SharedPtr velocity_pub_;

    void init();
    void run();
};

}  // namespace agent_velocity_controller

#endif  // AGENT_VELOCITY_CONTROLLER__RANDOM_VELOCITY_CONTROLLER_HPP_
