#ifndef AGENT_VELOCITY_CONTROLLER__TRIANGULAR_VELOCITY_CONTROLLER_HPP_
#define AGENT_VELOCITY_CONTROLLER__TRIANGULAR_VELOCITY_CONTROLLER_HPP_

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>


namespace agent_velocity_controller
{

class TriangularVelocityController : public rclcpp::Node
{
public:
    using Twist = geometry_msgs::msg::TwistStamped;

    // Smartpointer typedef
    typedef std::shared_ptr<TriangularVelocityController> SharedPtr;
    typedef std::unique_ptr<TriangularVelocityController> UniquePtr;

    TriangularVelocityController();
    TriangularVelocityController(const rclcpp::NodeOptions& options);
    virtual ~TriangularVelocityController();

protected:
    // Parameters
    double rate_;
    // Triangle wave parameters
    double acceleration_;
    double period_;
    double offset_;

    // Threads
    bool running_ = false;
    std::thread executor_thread_;

    // Publisher
    rclcpp::Publisher<Twist>::SharedPtr velocity_pub_;

    void init();
    void run();
};

}  // namespace agent_velocity_controller

#endif  // AGENT_VELOCITY_CONTROLLER__TRIANGULAR_VELOCITY_CONTROLLER_HPP_
