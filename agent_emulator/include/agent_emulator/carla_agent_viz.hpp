#ifndef AGENT_EMULATOR__CARLA_AGENT_VIZ_HPP_
#define AGENT_EMULATOR__CARLA_AGENT_VIZ_HPP_

#include <functional>
#include <thread>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace agent_emulator
{


class CarlaAgentViz : public rclcpp::Node
{
public:
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Pose = geometry_msgs::msg::PoseStamped;
    using Odometry = nav_msgs::msg::Odometry;

    // Smartpointer typedef
    typedef std::shared_ptr<CarlaAgentViz> SharedPtr;
    typedef std::unique_ptr<CarlaAgentViz> UniquePtr;

    // Constructors
    CarlaAgentViz();
    CarlaAgentViz(const rclcpp::NodeOptions& options);
    virtual ~CarlaAgentViz();


private:
    // Map to store the latest pose received from each agent
    std::map<std::string, Pose> agent_poses_;

    // Publishers
    rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_pub_;

    // Subscribers
    std::vector<rclcpp::Subscription<Odometry>::SharedPtr> odom_subs_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    // Initialization
    void init();

    // Main execution thread
    std::thread executor_thread_;
    bool stop_thread_;
    void run();
};


} // Namespace agent_emulator

#endif // AGENT_EMULATOR__CARLA_AGENT_VIZ_HPP_
