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

present_kinematics_pose = [0.0, 0.0, 0.0]
goal_kinematics_pose = [0.0, 0.0, 0.0]
prev_goal_kinematics_pose = [0.0, 0.0, 0.0]
timer = 0

task_position_delta = 0.01  # meter
task_position_x = 0.01  # meter
task_position_y = 0.01  # meter
task_position_z = 0.01  # meter
path_time = 2.0 # second

up_location = [0.0, 0.0, 0.0]   # 버튼중심점의 x, y, z가 저장됨(가정)(data1)
client_address = [0, 0]   # 목적지의 층수, 1호/2호가 저장됨(가정)(data1)
down_location = [0.0, 0.0, 0.0]   # 버튼중심점의 x, y, z가 저장됨(가정)(data)
button_1 = [0.0, 0.0, 0.0]   # 층수버튼 중심점의 x, y, z 저장됨(가정)(data)
button_2 = [0.0, 0.0, 0.0]   # 층수버튼 중심점의 x, y, z 저장됨(가정)(data)
button_3 = [0.0, 0.0, 0.0]   # 층수버튼 중심점의 x, y, z 저장됨(가정)(data)
button_4 = [0.0, 0.0, 0.0]   # 층수버튼 중심점의 x, y, z 저장됨(가정)(data)
led_number = [0, 0, 0, 0]   # 엘리베이터가 몇층에 있는지 확인 (ex. 2층 -> [0, 1, 0, 0])
robot_reached = 0   # 지도의 절대좌표로 도착했을 시 1로 바뀜   ## 혹은 현재 로봇의 위치랑 도착해야하는 절대위치 가져와서 같을경우 비교&toggle switch 만들거나

class TeleopKeyboard(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_keyboard')
        key_value = ''

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
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

    def send_goal_task_space(self, path_time):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2] 
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    # 현재 위치를 담습니다.
    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z

    # 얘넨 필요한거같은데 뭔 역할을 하는질 모르겠네
    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 7):
                goal_kinematics_pose[index] = present_kinematics_pose[index]
            # for index in range(0, 5):
            #     goal_joint_angle[index] = present_joint_angle[index]

# 얘네는 그냥 설정
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
    # print_present_values()
    return key

def print_present_values():
    print('Kinematics Pose(Pose X, Y, Z ): {:.3f}, {:.3f}, {:.3f}\n'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2]))
    # print('timer_counts : [  ', timer, '  ]')

def main():
    global timer
    global up_location
    global client_address
    global down_location
    global button_1
    global button_2
    global button_3
    global button_4
    global led_number
    global robot_reached
    global toggle
    global abcs_0
    global abcs_1
    global abcs_2
    print('yesbom babo 2')

    abcs_0 = 0
    abcs_1 = 0
    abcs_2 = 0
    toggle = 0

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
            print('"yesbom babo 3')
            print(toggle)

            # toggle=~은 "로봇이 엘베 앞에 도착하기 전"   (그리고 toggle바뀜) (if robot_reached != 0:사용)
            # toggle=~는 "엘베에 타기"   (그리고 toggle바뀜)
                # (앞에 사람 타고 타거나, 사람많으면 다음에 타거나 등등 예외처리 필요) (엘베에는 로봇자리의 안내가 되어있음) (1층일 경우 안거쳐도 되게?)

            # toggle=0은 엘베 앞에서 "상승버튼 누르기"   (and then toggle바뀜)
            if toggle == 0:
                print("debug 1")
                with open('/home/boma/colcon_ws/data1.txt', 'r') as f:
                    print(f)                    
                    up_location = f.read()
                    up_location = float(up_location)
                    print(up_location[0], up_location[1], up_location[2])
                    if up_location != 0:
                        print("The locaiton of button is ", up_location)
                        if timer == 0 :
                            timer = 60
                        elif timer != 0:
                            timer -= 1
                            if timer >= 50:
                                abcs_0 =  up_location[0] - present_kinematics_pose[0]
                                abcs_1 =  up_location[1] - present_kinematics_pose[1]
                                abcs_2 =  up_location[2] - present_kinematics_pose[2]
                                goal_kinematics_pose[0] = prev_goal_kinematics_pose[0] + abcs_0
                                goal_kinematics_pose[1] = prev_goal_kinematics_pose[1] + abcs_1
                                goal_kinematics_pose[2] = prev_goal_kinematics_pose[2] + abcs_2
                                teleop_keyboard.send_goal_task_space(path_time)
                                # path_time = 2.0
                            elif timer < 10:
                                timer = 0
                                with open('/home/boma/colcon_ws/data1.txt', 'w') as f:
                                    # f.write('0.0, 0.0, 0.0')
                                    f.write('0')
                                toggle = 1

            # # toggle=3은 "층수버튼 누르기"   (->누를시 toggle=4)
            # elif toggle == 3:
            #     with open('/home/boma/colcon_ws/data2.txt', 'r') as f:
            #         client_address = f.read()
            #         client_address = int(client_address)
            #         if client_address != 0:
            #             if client_address[0] == 1:
            #                 toggle = 4
            #             elif client_address[0] == 2:
            #                 # box이름들이 button_1, button_2, button_3, button_4이라고 가정 ---
            #                 with open('/home/boma/colcon_ws/data.txt', 'r') as f:
            #                     button_2 = f.read()
            #                     button_2 = int(button_2)
            #                     if button_2 != 0:
            #                         print("We're pressing the button of the ", client_address[0], " floor")
            #                         if timer == 0 :
            #                             timer = 60
            #                         elif timer != 0:
            #                             timer -= 1
            #                             if timer >= 45:
            #                                 kine_move(button_2[0], button_2[1], button_2[2])
            #                             elif timer > 1:
            #                                 timer = 0
            #                                 with open('/home/boma/colcon_ws/data.txt', 'w') as f:
            #                                     f.write('0.0, 0.0, 0.0')
            #                                 toggle = 4
            #             elif client_address[0] == 3:
            #                 with open('/home/boma/colcon_ws/data.txt', 'r') as f:
            #                     button_3 = f.read()
            #                     button_3 = int(button_3)
            #                     if button_3 != 0:
            #                         print("We're pressing the button of the ", client_address[0], " floor")
            #                         if timer == 0 :
            #                             timer = 60
            #                         elif timer != 0:
            #                             timer -= 1
            #                             if timer >= 45:
            #                                 kine_move(button_3[0], button_3[1], button_3[2])
            #                             elif timer > 1:
            #                                 timer = 0
            #                                 with open('/home/boma/colcon_ws/data.txt', 'w') as f:
            #                                     f.write('0.0, 0.0, 0.0')
            #                                 toggle = 4
            #             elif client_address[0] == 4:
            #                 with open('/home/boma/colcon_ws/data.txt', 'r') as f:
            #                     button_4 = f.read()
            #                     button_4 = int(button_4)
            #                     if button_4 != 0:
            #                         print("We're pressing the button of the ", client_address[0], " floor")
            #                         if timer == 0 :
            #                             timer = 60
            #                         elif timer != 0:
            #                             timer -= 1
            #                             if timer >= 45:
            #                                 kine_move(button_4[0], button_4[1], button_4[2])
            #                             elif timer > 1:
            #                                 timer = 0
            #                                 with open('/home/boma/colcon_ws/data.txt', 'w') as f:
            #                                     f.write('0.0, 0.0, 0.0')
            #                                 toggle = 4
                                
            # toggle=4는 "1호or2호 방문하기" (->방문시 toggle=5)
            # toggle=5는 "택배 내려놓기" (->성공시 toggle=6)
            # toggle=6는 "해당 층의 엘베앞으로 가기" (->도착시 toggle=7)
            # toggle=7는 "엘베타기" (->방문시 toggle=8)
            # toggle=8는 "층수누르기" (->누를시 toggle=9)
            # toggle=9는 "1층에 내리기" (->방문시 toggle=10)


    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__':
    main()