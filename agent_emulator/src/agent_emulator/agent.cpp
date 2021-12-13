#include <agent_emulator/agent.hpp>

namespace agent_emulator
{

Agent::Agent() : rclcpp::Node("agent")
{
    this->init();
}

Agent::Agent(const rclcpp::NodeOptions& options) : rclcpp::Node("agent", options)
{
    this->init();
}

Agent::~Agent()
{
    this->running_ = false;
    // Wait until the execution thread finishes
    if (this->goto_executor_thread_.joinable())
    {
        this->goto_executor_thread_.join();
    }
    // Stop main thread
    if (this->executor_thread_.joinable())
    {
        this->executor_thread_.join();
    }
}

void Agent::init()
{
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Initialize and declare parameters
    this->goto_velocity_.twist.linear.x = this->declare_parameter("goto_velocity_linear", 1.0);
    this->goto_velocity_.twist.angular.z = this->declare_parameter("goto_velocity_angular", 0.5);;
    this->goto_goal_tolerance_ = this->declare_parameter("goto_goal_tolerance", 0.05);

    // Descriptor for read_only parameters. These parameters cannot be changed (only overrided from yaml or launch args)
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    read_only_descriptor.read_only = true;

    // Set frame_id from parameter
    this->frame_id_ = this->declare_parameter("frame_id", "base_link", read_only_descriptor);

    // Set update_rate from parameter
    this->update_rate_ = this->declare_parameter("update_rate", 50.0, read_only_descriptor);

    // Set publish_tf from parameter
    this->publish_tf_ = this->declare_parameter("publish_tf", true, read_only_descriptor);

    // Set initial pose from parameter
    rclcpp::Time init_time = this->now();
    std::vector<double> initial_pose_param = {0.0, 0.0, 0.0};
    initial_pose_param = this->declare_parameter("initial_pose", initial_pose_param, read_only_descriptor);
    this->pose_.header.frame_id = "map";
    this->pose_.header.stamp = init_time;
    this->pose_.pose.position.x = initial_pose_param[0];
    this->pose_.pose.position.y = initial_pose_param[1];
    this->pose_.pose.orientation = quaternion_msg_from_yaw(deg_to_rad(initial_pose_param[2]));

    // Init velocity and acceleration
    this->velocity_.header.frame_id = this->frame_id_;
    this->velocity_.header.stamp = init_time;
    this->acceleration_.header.frame_id = this->frame_id_;
    this->acceleration_.header.stamp = init_time;

    // Set callback to handle parameter setting
    this->set_param_callback_handler_ = this->add_on_set_parameters_callback(
                                                std::bind(&Agent::set_param_callback, this, _1));

    // Publishers
    this->pose_pub_ = this->create_publisher<Pose>("pose", 5);
    this->velocity_pub_ = this->create_publisher<Twist>("velocity", 5);
    this->acceleration_pub_ = this->create_publisher<Accel>("acceleration", 5);

    // Subscribers
    this->velocity_sub_ = this->create_subscription<Twist>(
        "target_velocity", 5, std::bind(&Agent::velocity_callback, this, _1));
    this->acceleration_sub_ = this->create_subscription<Accel>(
        "target_acceleration", 5, std::bind(&Agent::acceleration_callback, this, _1));

    // Services
    this->set_pose_service_ = this->create_service<agent_emulator::srv::SetPose>(
        "set_pose", std::bind(&Agent::set_pose_callback, this, _1, _2));
    this->set_velocity_service_ = this->create_service<agent_emulator::srv::SetVelocity>(
        "set_velocity", std::bind(&Agent::set_velocity_callback, this, _1, _2));
    this->set_acceleration_service_ = this->create_service<agent_emulator::srv::SetAcceleration>(
        "set_acceleration", std::bind(&Agent::set_acceleration_callback, this, _1, _2));

    // Actions
    this->goto_action_server_ = rclcpp_action::create_server<GoTo>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "goto",
        std::bind(&Agent::handle_goto_goal, this, _1, _2),
        std::bind(&Agent::handle_goto_cancel, this, _1),
        std::bind(&Agent::handle_goto_accepted, this, _1));

    // TF broadcaster
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Publish state after initialization (to send initial pose and tf)
    this->last_update_time_ = init_time;
    this->publish_state();

    // Start the main execution thread to update and publish the state
    this->executor_thread_ = std::thread(&Agent::run, this);
}

void Agent::run()
{
    this->running_ = true;

    rclcpp::Rate rate(this->update_rate_);
    while (rclcpp::ok() && this->running_)
    {
        this->update_state();
        this->publish_state();
        rate.sleep();
    }
}

rcl_interfaces::msg::SetParametersResult Agent::set_param_callback(const std::vector<rclcpp::Parameter>& params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : params)
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Set param " << param.get_name() << ": " << param.value_to_string());
        if (param.get_name() == "goto_velocity_linear")
        {
            this->goto_velocity_.twist.linear.x = param.as_double();
        }
        else if (param.get_name() == "goto_velocity_angular")
        {
            this->goto_velocity_.twist.angular.z = param.as_double();
        }
        else if (param.get_name() == "goto_goal_tolerance")
        {
            this->goto_goal_tolerance_ = param.as_double();
        }
        else
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Unknown param \"" << param.get_name() << "\". Skipping.");
        }
    }

    return result;
}

void Agent::set_pose(const Pose& pose)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // Update state
    this->pose_.header = pose.header;
    this->pose_.pose = pose.pose;
    // Update time
    this->last_update_time_ = this->now();
}

void Agent::set_x(const double x)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // Update state
    this->pose_.pose.position.x = x;
    // Update time
    this->last_update_time_ = this->now();
}

void Agent::set_y(const double y)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // Update state
    this->pose_.pose.position.y = y;
    // Update time
    this->last_update_time_ = this->now();
}

void Agent::set_yaw(const double yaw)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // Update state
    this->pose_.pose.orientation = quaternion_msg_from_yaw(yaw);
    // Update time
    this->last_update_time_ = this->now();
}

void Agent::set_velocity(const Twist& velocity)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // When velocity or acceleration is changed, state must be updated
    this->update_state();
    // Update velocity
    this->velocity_.header = velocity.header;
    this->velocity_.twist = velocity.twist;
}

void Agent::set_linear_velocity(const double v_x)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // When velocity or acceleration is changed, state must be updated
    this->update_state();
    // Update velocity
    this->velocity_.twist.linear.x = v_x;
}

void Agent::set_angular_velocity(const double v_yaw)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // When velocity or acceleration is changed, state must be updated
    this->update_state();
    // Update velocity
    this->velocity_.twist.angular.z = v_yaw;
}

void Agent::set_acceleration(const Accel& acceleration)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // When velocity or acceleration is changed, state must be updated
    this->update_state();
    // Update acceleration
    this->acceleration_.header = acceleration.header;
    this->acceleration_.accel = acceleration.accel;
}

void Agent::set_linear_acceleration(const double a_x)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // When velocity or acceleration is changed, state must be updated
    this->update_state();
    // Update velocity
    this->acceleration_.accel.linear.x = a_x;
}

void Agent::set_angular_acceleration(const double a_yaw)
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    // When velocity or acceleration is changed, state must be updated
    this->update_state();
    // Update velocity
    this->acceleration_.accel.angular.z = a_yaw;
}

void Agent::set_frame_id(const std::string frame_id)
{
    this->frame_id_ = frame_id;
    this->velocity_.header.frame_id = frame_id;
    this->acceleration_.header.frame_id = frame_id;
}



void Agent::velocity_callback(const Twist::SharedPtr twist_msg)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Set velocity (x, yaw) to (" <<
                        twist_msg->twist.linear.x << ", " <<
                        twist_msg->twist.angular.z << ")");
    this->set_velocity(*twist_msg);
}

void Agent::acceleration_callback(const Accel::SharedPtr accel_msg)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Set acceleration (x, yaw) to (" <<
                        accel_msg->accel.linear.x << ", " << 
                        accel_msg->accel.angular.z << ")");
    this->set_acceleration(*accel_msg);
}

void Agent::set_pose_callback(const std::shared_ptr<SetPose::Request> req,
                               std::shared_ptr<SetPose::Response> res)
{
    // Check if the agent is performing an  action
    if (this->performing_goto_action_)
    {
        // Reject pose updates when performing an action
        RCLCPP_WARN(this->get_logger(), "Cannot set pose while performing an action!");
        res->success = false;
    }
    else
    {
        RCLCPP_DEBUG_STREAM(this->get_logger(),
                            "Set pose (x, y) to (" <<
                            req->pose.pose.position.x << ", " <<
                            req->pose.pose.position.y << ")");
        // Update the pose
        this->set_pose(req->pose);
        res->success = true;
    }
}

void Agent::set_velocity_callback(const std::shared_ptr<SetVelocity::Request> req,
                                   std::shared_ptr<SetVelocity::Response> res)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Set velocity (x, yaw) to (" <<
                        req->velocity.twist.linear.x << ", " <<
                        req->velocity.twist.angular.z << ")");
    // Update the velocity
    this->set_velocity(req->velocity);
    res->success = true;
}

void Agent::set_acceleration_callback(const std::shared_ptr<SetAcceleration::Request> req,
                                       std::shared_ptr<SetAcceleration::Response> res)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Set acceleration (x, yaw) to (" <<
                        req->acceleration.accel.linear.x << ", " <<
                        req->acceleration.accel.angular.z << ")");
    // Update the acceleration
    this->set_acceleration(req->acceleration);
    res->success = true;
}

rclcpp_action::GoalResponse Agent::handle_goto_goal(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const GoTo::Goal> goal)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Received GoTo goal request " << (int)uuid[0] << " - (" <<
                        goal->goal_pose.pose.position.x << ", " << 
                        goal->goal_pose.pose.position.y << ")");
    // Check if agent is already performing an action
    if (this->performing_goto_action_)
    {
        RCLCPP_WARN(this->get_logger(), "Already performing an action. GoTo goal REJECTED.");
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Since we are sure that no goto action is being performed, we can join the thread (needed to create a new one).
    if (this->goto_executor_thread_.joinable()) this->goto_executor_thread_.join();

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Agent::handle_goto_cancel(const std::shared_ptr<GoalHandleGoTo> goal_handle)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Received request to cancel GoTo goal " <<
                       (int)(goal_handle->get_goal_id()[0]));
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Agent::handle_goto_accepted(const std::shared_ptr<GoalHandleGoTo> goal_handle)
{
    using std::placeholders::_1;
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    this->goto_executor_thread_ = std::thread(std::bind(&Agent::execute_goto, this, _1), goal_handle);
}

void Agent::execute_goto(const std::shared_ptr<GoalHandleGoTo> goal_handle)
{
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Executing GoTo goal " << (int)(goal_handle->get_goal_id()[0]));
    this->performing_goto_action_ = true;

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoTo::Feedback>();
    auto result = std::make_shared<GoTo::Result>();

    rclcpp::Rate rate(100);
    while (rclcpp::ok() && this->performing_goto_action_)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            // Set zero velocity to stop any movement
            this->set_zero_velocity();

            goal_handle->canceled(result);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "GoTo goal " <<
                               (int)(goal_handle->get_goal_id()[0]) << " canceled");
            this->performing_goto_action_ = false;
            return;
        }

        // Update state
        this->update_state();

        // Check goal
        double trans_error = std::hypot(goal->goal_pose.pose.position.x - this->pose_.pose.position.x,
                                        goal->goal_pose.pose.position.y - this->pose_.pose.position.y);
        if (trans_error < goto_goal_tolerance_)
        {
            // Set zero velocity to stop any movement
            this->set_zero_velocity();

            goal_handle->succeed(result);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "GoTo goal " <<
                               (int)(goal_handle->get_goal_id()[0]) << " succeeded");
            break;
        }

        // Compute velocity command
        Twist goal_velocity = this->compute_velocity_cmd(goal_handle);
        // Update velocity
        this->set_velocity(goal_velocity);

        // Publish feedback
        feedback->current_pose = this->pose_;
        feedback->cmd_vel = goal_velocity;
        goal_handle->publish_feedback(feedback);

        rate.sleep();
    }
    this->performing_goto_action_ = false;
}

Twist Agent::compute_velocity_cmd(const std::shared_ptr<GoalHandleGoTo> goal_handle) const
{
    Twist velocity_cmd;
    velocity_cmd.header = this->velocity_.header;

    Pose goal_pose = goal_handle->get_goal()->goal_pose;

    double d_x = goal_pose.pose.position.x - this->pose_.pose.position.x;
    double d_y = goal_pose.pose.position.y - this->pose_.pose.position.y;

    double current_yaw = std::fmod(yaw_from_quaternion<double>(this->pose_.pose.orientation), deg_to_rad(360));
    current_yaw = norm_angle(current_yaw);
    double goal_yaw = std::fmod(std::atan2(d_y, d_x), deg_to_rad(360));
    goal_yaw = norm_angle(goal_yaw);

    // Get rotation error
    double yaw_error_pos = std::fmod(goal_yaw - current_yaw, deg_to_rad(360));
    yaw_error_pos = norm_angle(yaw_error_pos);
    double yaw_error_neg = std::fmod(current_yaw - goal_yaw, deg_to_rad(360));
    yaw_error_neg = norm_angle(yaw_error_neg);
    double yaw_error = yaw_error_pos < yaw_error_neg ? yaw_error_pos : - yaw_error_neg;

    // Get translation error
    double trans_error = std::hypot(d_x, d_y);

    // If rotation error is too high, do only rotation
    velocity_cmd.twist.linear.x = std::abs(yaw_error) > deg_to_rad(20) ? 0.0 :
                                this->goto_velocity_.twist.linear.x * smooth(trans_error, 0.3);

    velocity_cmd.twist.angular.z = this->goto_velocity_.twist.angular.z * smooth(yaw_error, 0.2);

    return velocity_cmd;
}

void Agent::set_zero_velocity()
{
    Twist zero_vel;
    zero_vel.header.frame_id = this->velocity_.header.frame_id;
    this->set_velocity(zero_vel);
}

void Agent::update_velocity(const rclcpp::Duration& dt_dur)
{
    // Convert Duration to seconds
    double dt = dt_dur.seconds();

    this->velocity_.twist.linear.x += dt * this->acceleration_.accel.linear.x;
    this->velocity_.twist.angular.z += dt * this->acceleration_.accel.angular.z;
}

void Agent::update_pose(const rclcpp::Duration& dt_dur)
{
    // Convert Duration to seconds
    double dt = dt_dur.seconds();

    double old_yaw = yaw_from_quaternion<double>(this->pose_.pose.orientation);
    this->pose_.pose.position.x += dt * this->velocity_.twist.linear.x * std::cos(old_yaw);
    this->pose_.pose.position.y += dt * this->velocity_.twist.linear.x * std::sin(old_yaw);
    this->pose_.pose.position.z += dt * this->velocity_.twist.linear.z;

    double new_yaw = old_yaw + dt * this->velocity_.twist.angular.z;
    this->pose_.pose.orientation = quaternion_msg_from_yaw(new_yaw);
}

void Agent::update_state()
{
    // Lock mutex
    std::lock_guard<std::recursive_mutex> lock(this->state_mutex_);
    rclcpp::Time update_time = this->now();
    // Get time delta
    rclcpp::Duration dt = update_time - this->last_update_time_;
    // Update velocity first so  it is precomuped for the pose update
    this->update_velocity(dt);
    // Update pose
    this->update_pose(dt);
    // Update stamps
    this->pose_.header.stamp = update_time;
    this->velocity_.header.stamp = update_time;
    this->acceleration_.header.stamp = update_time;
    // Update time
    this->last_update_time_ = update_time;
}

// Publish current satate
void Agent::publish_state() const
{
    // Publish state
    this->pose_pub_->publish(std::move(std::make_unique<Pose>(this->pose_)));
    this->velocity_pub_->publish(std::move(std::make_unique<Twist>(this->velocity_)));
    this->acceleration_pub_->publish(std::move(std::make_unique<Accel>(this->acceleration_)));

    // Broadcast map -> base_link transform
    if (this->publish_tf_)
    {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->pose_.header.stamp;
        tf.header.frame_id = this->pose_.header.frame_id;
        tf.child_frame_id = this->frame_id_;
        tf.transform.translation.x = this->pose_.pose.position.x;
        tf.transform.translation.y = this->pose_.pose.position.y;
        tf.transform.translation.z = this->pose_.pose.position.z;
        tf.transform.rotation = this->pose_.pose.orientation;
        this->tf_broadcaster_->sendTransform(tf);
    }
}

} // Namespace agent_emulator


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(agent_emulator::Agent)
