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


usage = """
Control Your OpenManipulator!

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
        self.goal_task_space = self.create_client(SetKinematicsPose, 'goal_task_space_path')
        self.tool_control = self.create_client(SetJointPosition, 'goal_tool_control')
        self.goal_task_space_req = SetKinematicsPose.Request()
        self.tool_control_req = SetJointPosition.Request()

    def send_goal_task_space(self):
        self.goal_task_space_req.end_effector_name = 'gripper'
        self.goal_task_space_req.kinematics_pose.pose.position.x = goal_kinematics_pose[0]
        self.goal_task_space_req.kinematics_pose.pose.position.y = goal_kinematics_pose[1]
        self.goal_task_space_req.kinematics_pose.pose.position.z = goal_kinematics_pose[2]
        self.goal_task_space_req.path_time = path_time

        try:
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z

    def open_manipulator_state_callback(self, msg):
        if msg.open_manipulator_moving_state == 'STOPPED':
            for index in range(0, 3):
                goal_kinematics_pose[index] = present_kinematics_pose[index]

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
    print('Kinematics Pose(Pose X, Y, Z): {:.3f}, {:.3f}, {:.3f}'.format(
        present_kinematics_pose[0],
        present_kinematics_pose[1],
        present_kinematics_pose[2]))


def main():
    settings = None
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
    

    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop_keyboard.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()