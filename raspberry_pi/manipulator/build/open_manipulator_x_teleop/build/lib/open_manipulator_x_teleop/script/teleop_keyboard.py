#!/usr/bin/env python
#
# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Will Son

from math import exp
import os
import rclpy
import select
import sys
import threading
import time

from open_manipulator_msgs.msg import KinematicsPose, OpenManipulatorState
from open_manipulator_msgs.srv import SetJointPosition, SetKinematicsPose
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
present_state = [0, 0, 0, 0, 0, 0, 0]
mission_state = [0, 0, 0]
timer = 0

task_position_delta = 0.01  # meter
joint_angle_delta = 0.05  # radian
path_time = 0.2 # second

usage = """
-------------------------------------------------------
  Waste Seperation OpenManipulator Controller (ﾉ'ω')ﾉ 
-------------------------------------------------------
Task Space Control:                                     
         (Forward, X+)
              W                   Q (Upward, Z+)
(Left, Y+) A     D (Right, Y-)    Z (Downward, Z-)
              X 
        (Backward, X-)

POINTS
- Glass :   START POINT (Y), END POINT (H), Mission Start (,)
- Plastic : START POINT (U), END POINT (J), Mission Start (.)
- Can :     START POINT (I), END POINT (K), Mission Start (/)

- INIT : (1)  |  PHOTO : (O)
- HOME : (2)  |   END  : (P)

- Gripper: Increase (F), Decrease (G) | Fully Open (V), Fully Close (B)

CONTINUOUS JOINT MOVE : (3), (4), (5), (6) | JOINT 1, 2, 3, 4
CONTINUOUS SPACE MOVE : (7), (8), (9) | x, y, z

CTRL-C to quit
"""

e = """
Communications Failed
"""


class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''

        # Create joint_states subscriber
        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            self.qos)
        self.joint_state_subscription

        # Create kinematics_pose subscriber
        self.kinematics_pose_subscription = self.create_subscription(
            KinematicsPose,
            'kinematics_pose',
            self.kinematics_pose_callback,
            self.qos)
        self.kinematics_pose_subscription

        # Create manipulator state subscriber
        self.open_manipulator_state_subscription = self.create_subscription(
            OpenManipulatorState,
            'states',
            self.open_manipulator_state_callback,
            self.qos)
        self.open_manipulator_state_subscription

        # Create Service Clients
        self.goal_joint_space = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_joint_space_req = SetJointPosition.Request()
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.kinematics_pose.pose.orientation.w = goal_kinematics_pose[3]
        self.goal_task_space_req.kinematics_pose.pose.orientation.x = goal_kinematics_pose[4]
        self.goal_task_space_req.kinematics_pose.pose.orientation.y = goal_kinematics_pose[5]
        self.goal_task_space_req.kinematics_pose.pose.orientation.z = goal_kinematics_pose[6]
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_goal_joint_space(self, path_time):
        self.goal_joint_space_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.goal_joint_space_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.goal_joint_space_req.path_time = path_time

        try:
            self.goal_joint_space.call_async(self.goal_joint_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Joint failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)

        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            for index in range(0, 5):
                goal_joint_angle[index] = present_joint_angle[index]

def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print_present_values()
    return key

def print_present_values():
    print(usage)
    print('Joint Angle(Rad) : [{:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}]'.format(
        present_joint_angle[0],
        present_joint_angle[1],
        present_joint_angle[2],
        present_joint_angle[3],
        present_joint_angle[4]))
    print('Kinematics Pose(Pose X, Y, Z | Orientation W, X, Y, Z): {:.3f}, {:.3f}, {:.3f} | {:.3f}, {:.3f}, {:.3f}, {:.3f}\n'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2],
        present_kinematics_pose[3],
        present_kinematics_pose[4],
        present_kinematics_pose[5],
        present_kinematics_pose[6]))
    print('Joint States : [{:}, {:}, {:}, {:}]  \nKinematics States : [{:}, {:}, {:}]\n'.format(
        present_state[0] % 4,
        present_state[1] % 4,
        present_state[2] % 4,
        present_state[3] % 4,
        present_state[4] % 4,
        present_state[5] % 4,
        present_state[6] % 4))
    print('Mission State : [{:}, {:}, {:}] | (Glass, Plastic, Can)'.format(
        mission_state[0],
        mission_state[1],
        mission_state[2]
    ))
    print('timer_counts : [  ', timer, '  ]')

def main():

    def arm_move(angle1, angle2, angle3, angle4):
        goal_joint_angle[0] = angle1
        goal_joint_angle[1] = angle2
        goal_joint_angle[2] = angle3
        goal_joint_angle[3] = angle4
        pathtime = 1.0
        teleop_keyboard.send_goal_joint_space(pathtime)

    def hand_open():
        goal_joint_angle[4] = 0.01
        teleop_keyboard.send_tool_control_request()

    def hand_close():
        goal_joint_angle[4] = -0.01
        teleop_keyboard.send_tool_control_request()

    settings = None

    global timer

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init()
    except Exception as e:
        print(e)

    try:
        teleop_keyboard = TeleopKeyboard()
    except Exception as e:
        print(e)

    try:
        while(rclpy.ok()):
            rclpy.spin_once(teleop_keyboard)
            key_value = get_key(settings)
            if key_value == 'w':
                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'x':
                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] - task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'a':
                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'd':
                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] - task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'q':
                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'z':
                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] - task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif key_value == 'y':
                arm_move(0.000, 0.225, 1.373, -1.641)
            elif key_value == 'u':
                arm_move(-1.580, 0.225, 1.373, -1.641)
            elif key_value == 'i':
                arm_move(1.580, 0.225, 1.373, -1.641)
            elif key_value == 'h':
                arm_move(0.000, 0.609, 0.218, -0.927)
            elif key_value == 'j':
                arm_move(-1.580, 0.609, 0.218, -0.927)
            elif key_value == 'k':
                arm_move(1.580, 0.609, 0.218, -0.927)
            elif key_value == 'o':
                arm_move(0.000, -1.379, 0.347, 1.095)
            elif key_value == 'p':
                arm_move(0.000, 0.301, 0.204, 1.100)
            elif key_value == 'f':
                goal_joint_angle[4] = prev_goal_joint_angle[4] + 0.002
                teleop_keyboard.send_tool_control_request()
            elif key_value == 'g':
                goal_joint_angle[4] = prev_goal_joint_angle[4] - 0.002
                teleop_keyboard.send_tool_control_request()
            elif key_value == 'v':
                goal_joint_angle[4] = 0.01
                teleop_keyboard.send_tool_control_request()
            elif key_value == 'b':
                goal_joint_angle[4] = -0.01
                teleop_keyboard.send_tool_control_request()
            elif key_value == '1':
                arm_move(0.0, 0.0, 0.0, 0.0)
            elif key_value == '2':
                arm_move(0.0, -1.05, 0.35, 0.70)
            elif key_value == '3':
                present_state[0] += 1
            elif key_value == '4':
                present_state[1] += 1
            elif key_value == '5':
                present_state[2] += 1
            elif key_value == '6':
                present_state[3] += 1
            elif key_value == '7':
                present_state[4] += 1
            elif key_value == '8':
                present_state[5] += 1
            elif key_value == '9':
                present_state[6] += 1
            elif key_value == ',':
                if mission_state[0] == 0 and mission_state[1] == 0 and mission_state[2] == 0:
                    mission_state[0] += 1
            elif key_value == '.':
                if mission_state[0] == 0 and mission_state[1] == 0 and mission_state[2] == 0:
                    mission_state[1] = 1
            elif key_value == '/':
                if mission_state[0] == 0 and mission_state[1] == 0 and mission_state[2] == 0:
                    mission_state[2] = 1
            else:
                if key_value == '\x03':
                    break
                else:
                    for index in range(0, 7):
                        prev_goal_kinematics_pose[index] = goal_kinematics_pose[index]
                    for index in range(0, 5):
                        prev_goal_joint_angle[index] = goal_joint_angle[index]

            if present_state[0] % 4 == 1:
                goal_joint_angle[0] = prev_goal_joint_angle[0] - joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)
            elif present_state[0] % 4 == 3:
                goal_joint_angle[0] = prev_goal_joint_angle[0] + joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)

            if present_state[1] % 4 == 1:
                goal_joint_angle[1] = prev_goal_joint_angle[1] - joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)
            elif present_state[1] % 4 == 3:
                goal_joint_angle[1] = prev_goal_joint_angle[1] + joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)

            if present_state[2] % 4 == 1:
                goal_joint_angle[2] = prev_goal_joint_angle[2] - joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)
            elif present_state[2] % 4 == 3:
                goal_joint_angle[2] = prev_goal_joint_angle[2] + joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)

            if present_state[3] % 4 == 1:
                goal_joint_angle[3] = prev_goal_joint_angle[3] - joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)
            elif present_state[3] % 4 == 3:
                goal_joint_angle[3] = prev_goal_joint_angle[3] + joint_angle_delta
                pathtime = path_time
                teleop_keyboard.send_goal_joint_space(pathtime)

            if present_state[4] % 4 == 1:
                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif present_state[4] % 4 == 3:
                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] - task_position_delta
                teleop_keyboard.send_goal_task_space()

            if present_state[5] % 4 == 1:
                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif present_state[5] % 4 == 3:
                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] - task_position_delta
                teleop_keyboard.send_goal_task_space()

            if present_state[6] % 4 == 1:
                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] + task_position_delta
                teleop_keyboard.send_goal_task_space()
            elif present_state[6] % 4 == 3:
                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] - task_position_delta
                teleop_keyboard.send_goal_task_space()

            if mission_state[0] == 1:
                if timer == 0 :
                    timer = 60
                elif timer != 0:
                    timer -= 1

                    if timer >= 45:
                        arm_move(0.000, -1.379, 0.347, 1.095)
                    elif timer >= 30:
                        arm_move(0.000, 0.225, 1.373, -1.641)
                    elif timer >= 15:
                        arm_move(0.000, 0.609, 0.218, -0.927)
                    elif timer > 1:
                        arm_move(0.000, -1.379, 0.347, 1.095)
                    elif timer == 1:
                        timer = 0
                        mission_state[0] = 0

            if mission_state[1] == 1:
                if timer == 0 :
                    timer = 70
                elif timer != 0:
                    timer -= 1

                    if timer >= 55:
                        arm_move(0.000, -1.379, 0.347, 1.095)
                    elif timer >= 40:
                        arm_move(0.000, 0.225, 1.373, -1.641)
                    elif timer >= 30:
                        arm_move(-1.580, 0.225, 1.373, -1.641)
                    elif timer >= 15:
                        arm_move(-1.580, 0.609, 0.218, -0.927)
                    elif timer > 1:
                        arm_move(0.000, -1.379, 0.347, 1.095)
                    elif timer == 1:
                        timer = 0
                        mission_state[1] = 0

            if mission_state[2] == 1:
                if timer == 0 :
                    timer = 70
                elif timer != 0:
                    timer -= 1

                    if timer >= 55:
                        arm_move(0.000, -1.379, 0.347, 1.095)
                    elif timer >= 40:
                        arm_move(0.000, 0.225, 1.373, -1.641)
                    elif timer >= 30:
                        arm_move(1.580, 0.225, 1.373, -1.641)
                    elif timer >= 15:
                        arm_move(1.580, 0.609, 0.218, -0.927)
                    elif timer > 1:
                        arm_move(0.000, -1.379, 0.347, 1.095)
                    elif timer == 1:
                        timer = 0
                        mission_state[2] = 0

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()