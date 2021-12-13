#include <agent_emulator/agent_viz.hpp>

namespace agent_emulator
{


AgentViz::AgentViz() : rclcpp::Node("agent_viz")
{
    this->init();
}

AgentViz::AgentViz(const rclcpp::NodeOptions& options) : rclcpp::Node("agent_viz", options)
{
    this->init();
}

AgentViz::~AgentViz()
{
    // Wait until the execution thread finishes
    if (this->executor_thread_.joinable())
    {
        this->stop_thread_ = true;
        this->executor_thread_.join();
    }
}

void AgentViz::init()
{
    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    // Set agent list description file from parameter
    std::vector<std::string> pose_topic_list;
    pose_topic_list = this->declare_parameter("pose_topics", pose_topic_list, read_only_descriptor);

    if (pose_topic_list.size() == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Empty pose_topics parameter.");
        rclcpp::shutdown();
    }

    // Initialize marker publisher
    marker_array_pub_ = this->create_publisher<MarkerArray>("/agent_viz/agent_markers", 5);

    // Create a reentrant callback_group for callbacks, so they can be called concurrently
    this->callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant);
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = this->callback_group_;

    // Initialize subscribers
    for (const auto& topic : pose_topic_list)
    {
        // Create callback lambda with custom name argument
        auto callback = [topic, this](const Pose::SharedPtr msg) -> void
        {
            // Store the pose in the map by the topic name
            this->agent_poses_[topic] = *msg;
        };

        // Create and store subscriber
        rclcpp::Subscription<Pose>::SharedPtr sub =
            this->create_subscription<Pose>(topic, rclcpp::SensorDataQoS(), callback, sub_options);
        this->pose_subs_.push_back(sub);
    }

    this->stop_thread_ = false;
    this->executor_thread_ = std::thread(&AgentViz::run, this);
}

void AgentViz::run()
{
    rclcpp::Rate rate(20);
    while (rclcpp::ok() && !this->stop_thread_)
    {
        MarkerArray markers;
        for (const auto& p : this->agent_poses_)
        {
            std::string topic = p.first;
            Pose pose = p.second;

            visualization_msgs::msg::Marker marker;
            marker.header = pose.header;
            marker.ns = topic;
            marker.id = 1;
            // marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            marker.mesh_resource = "package://agent_emulator/meshes/agent_mesh.stl";
            // marker.mesh_resource = "package://agent_emulator/meshes/obstacle_marker.dae";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose.pose;
            marker.pose.position.z += 0.5; // For better visualization
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
            // Lightblue color
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            // Add marker to the list
            markers.markers.push_back(marker);
        }
        this->marker_array_pub_->publish(markers);

        rate.sleep();
    }
}

} // Namespace agent_emulator

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(agent_emulator::AgentViz)
