#ifndef AGENT_EMULATOR__AGENT_HPP_
#define AGENT_EMULATOR__AGENT_HPP_

#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <agent_emulator/utils.hpp>
#include <agent_emulator/srv/set_pose.hpp>
#include <agent_emulator/srv/set_velocity.hpp>
#include <agent_emulator/srv/set_acceleration.hpp>
#include <agent_emulator/action/go_to.hpp>

namespace agent_emulator
{

using Pose = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::TwistStamped;
using Accel = geometry_msgs::msg::AccelStamped;
using Odometry = nav_msgs::msg::Odometry;

class Agent : public rclcpp::Node
{
public:
    using SetPose = agent_emulator::srv::SetPose;
    using SetVelocity = agent_emulator::srv::SetVelocity;
    using SetAcceleration = agent_emulator::srv::SetAcceleration;
    using GoTo = agent_emulator::action::GoTo;
    using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;

    // Smartpointer typedef
    typedef std::shared_ptr<Agent> SharedPtr;
    typedef std::unique_ptr<Agent> UniquePtr;

    // Constructors
    Agent();
    Agent(const rclcpp::NodeOptions& options);
    virtual ~Agent();

    // Get current pose
    inline Pose get_pose() const {return this->pose_;}
    // Get current odometry
    inline Odometry get_odom() const {return this->odom_;}
    // Get x, y and yaw values
    inline double x() const {return this->pose_.pose.position.x;}
    inline double y() const {return this->pose_.pose.position.y;}
    inline double yaw() const {return yaw_from_quaternion<double>(this->pose_.pose.orientation);}
    // Get current velocity
    inline Twist velocity() const {return this->velocity_;};
    // Get current acceleration
    inline Accel acceleration() const {return this->acceleration_;};
    // Get frame_id
    inline std::string frame_id() const {return this->frame_id_;};

    // Set current pose
    void set_pose(const Pose& pose);
    // Set x, y and yaw values
    void set_x(const double x);
    void set_y(const double y);
    void set_yaw(const double yaw);
    // Reset odometry values
    void reset_odom();
    // Set current velocity 
    void set_velocity(const Twist& velocity);
    void set_linear_velocity(const double v_x);
    void set_angular_velocity(const double v_yaw);
    // Set current acceleration 
    void set_acceleration(const Accel& acceleration);
    void set_linear_acceleration(const double a_x);
    void set_angular_acceleration(const double a_yaw);
    // Set frame_id 
    void set_frame_id(const std::string frame_id);

    // Update state using a time_delta from last update call
    void update_state();

    // Publish current satate
    void publish_state() const;

protected:
    // State
    Pose pose_;
    Twist velocity_;
    Accel acceleration_;
    Odometry odom_;

    // Threads
    bool performing_goto_action_ = false;
    bool running_ = false;
    std::thread executor_thread_;
    std::thread goto_executor_thread_;

    // Parameters
    Twist goto_velocity_;
    double goto_goal_tolerance_;
    std::string frame_id_;
    double update_rate_;
    bool publish_tf_;
    OnSetParametersCallbackHandle::SharedPtr set_param_callback_handler_;

    // Time point since last state update (from updateState() or from setters)
    rclcpp::Time last_update_time_;

    // Mutex to avoid conflict in state updates
    std::recursive_mutex state_mutex_;

    // Publishers
    rclcpp::Publisher<Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<Accel>::SharedPtr acceleration_pub_;
    rclcpp::Publisher<Odometry>::SharedPtr odometry_pub_;

    // Subscribers
    rclcpp::Subscription<Twist>::SharedPtr velocity_sub_;
    rclcpp::Subscription<Accel>::SharedPtr acceleration_sub_;

    // Services
    rclcpp::Service<agent_emulator::srv::SetPose>::SharedPtr set_pose_service_;
    rclcpp::Service<agent_emulator::srv::SetVelocity>::SharedPtr set_velocity_service_;
    rclcpp::Service<agent_emulator::srv::SetAcceleration>::SharedPtr set_acceleration_service_;

    // Actions
    rclcpp_action::Server<GoTo>::SharedPtr goto_action_server_;

    // TF2 broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Initialization
    void init();
    void run();

    // Param callback
    rcl_interfaces::msg::SetParametersResult set_param_callback(const std::vector<rclcpp::Parameter>& params);

    // Velocity callback
    void velocity_callback(const Twist::SharedPtr twist_msg);
    // Acceleration callback
    void acceleration_callback(const Accel::SharedPtr twist_msg);

    // set_pose service callback
    void set_pose_callback(const std::shared_ptr<SetPose::Request> req,
                                std::shared_ptr<SetPose::Response> res);
    // set_velocity service callback
    void set_velocity_callback(const std::shared_ptr<SetVelocity::Request> req,
                                std::shared_ptr<SetVelocity::Response> res);
    // set_acceleration service callback
    void set_acceleration_callback(const std::shared_ptr<SetAcceleration::Request> req,
                                    std::shared_ptr<SetAcceleration::Response> res);

    // Action callbacks
    rclcpp_action::GoalResponse handle_goto_goal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const GoTo::Goal> goal);
    rclcpp_action::CancelResponse handle_goto_cancel(const std::shared_ptr<GoalHandleGoTo> goal_handle);
    void handle_goto_accepted(const std::shared_ptr<GoalHandleGoTo> goal_handle);
    void execute_goto(const std::shared_ptr<GoalHandleGoTo> goal_handle);
    Twist compute_velocity_cmd(const std::shared_ptr<GoalHandleGoTo> goal_handle) const;

    // Stop any movement
    void set_zero_velocity();

    // Update state velocity after dt
    void update_velocity(const rclcpp::Duration& dt_dur);
    // Update state pose after dt
    void update_pose(const rclcpp::Duration& dt_dur);
    // Update state odom after dt
    void update_odom(const rclcpp::Duration& dt_dur);
};


} // Namespace agent_emulator

#endif // AGENT_EMULATOR__AGENT_HPP_
