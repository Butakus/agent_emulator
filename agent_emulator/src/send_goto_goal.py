#!/usr/bin/python3

import sys
import time
from math import pi

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from agent_emulator.action import GoTo

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pynput import keyboard
import termios
import tty
import select


action_client = None
settings = None

class GoToClient(Node):

    def __init__(self, action_name, goal_x, goal_y):
        super().__init__('goto_client')
        self._action_client = ActionClient(self, GoTo, action_name)
        self.goal_x = goal_x
        self.goal_y = goal_y

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted')

    def feedback_callback(self, feedback):
        self.get_logger().info('--------------------------')
        self.get_logger().info('{x},{z}'.format(x=feedback.feedback.cmd_vel.twist.linear.x,
                                                z=180.0*feedback.feedback.cmd_vel.twist.angular.z/pi))

    def cancel_goal(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = GoTo.Goal()
        goal_msg.goal_pose = PoseStamped()
        goal_msg.goal_pose.pose.position.x = self.goal_x
        goal_msg.goal_pose.pose.position.y = self.goal_y

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        # self._send_goal_future = self._action_client.send_goal_async(
        #     goal_msg,
        #     feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


def on_press(key):
    global action_client
    try:
        if key.char == 'q':
            print("Cancel goal")
            action_client.cancel_goal()
    except AttributeError:
        print('special key {0} pressed'.format(
            key))
        if key == keyboard.Key.esc:
            print("Bye!")
            rclpy.shutdown()

def get_key():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    global action_client, settings
    settings = termios.tcgetattr(sys.stdin)

    action_name = '/goto'
    goal_x = 0.0
    goal_y = 0.0

    if len(sys.argv) > 1:
        action_name = sys.argv[1]
    if len(sys.argv) > 2:
        goal_x = float(sys.argv[2])
    if len(sys.argv) > 3:
        goal_y = float(sys.argv[3])

    rclpy.init(args=args)

    # Create GoTo action client
    action_client = GoToClient(action_name, goal_x, goal_y)

    # Init keyboard listener
    listener = keyboard.Listener(
        on_press=on_press)
    listener.start()
    print("Press 'q' to cancel the action")

    action_client.send_goal()

    while rclpy.ok():
        # key = get_key()
        # if key == 'c':
        #     print("Cancel goal")
        #     action_client.cancel_goal()
        # elif key == '\x03':
        #     print("Bye!")
        #     rclpy.shutdown()
        rclpy.spin_once(action_client)
        time.sleep(0.05)
    
    listener.stop()


if __name__ == '__main__':
    main()
