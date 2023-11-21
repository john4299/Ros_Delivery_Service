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
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty


# present_kinematics_pose = [0.0, 0.0, 0.0]   # 현재 위치
# goal_kinematics_pose = [0.0, 0.0, 0.0]   # 목적지 위치
# up_location = [0.0, 0.0, 0.0]  # 버튼중심점의 x, y, z가 저장됨(가정)(data1)

present_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 현재 위치
goal_kinematics_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 목적지 위치
up_location = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 

present_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]
prev_goal_joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0]

path_time = 2.0  # second


class TeleopSubscriber(Node):

    qos = QoSProfile(depth=10)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    def __init__(self):
        super().__init__('teleop_sub')

        self.node = self
        self.getsee = self.create_subscription(String,'getsee',self.GETSEE,10)
        self.getsee
        self.downarm = self.create_subscription(String,'downarm',self.DOWNARM,10)
        self.downarm
        self.uparm = self.create_subscription(String,'uparm',self.UPARM,10)
        self.uparm
        self.movearm = self.create_subscription(String,'movearm',self.MOVEARM,10)
        self.movearm
        self.turnarm = self.create_subscription(String,'turnarm',self.TURNARM,10)
        self.turnarm
        self.pushup = self.create_subscription(String,'pushup',self.PUSHUP,10)
        self.pushup
        self.seeelvae = self.create_subscription(String,'seeelvae',self.SEEELVAE,10)
        self.seeelvae
        self.seeho = self.create_subscription(String,'seeho',self.SEEHO,10)
        self.seeho
        self.seebox = self.create_subscription(String,'seebox',self.SEEBOX,10)
        self.seebox
        self.pushz = self.create_subscription(String,'pushz',self.PUSHZ,10)
        self.pushz
        self.pullz = self.create_subscription(String,'pullz',self.PULLZ,10)
        self.pullz
        self.kimchi = self.create_subscription(String,'kimchi',self.KIMCHI,10)
        self.kimchi
        self.selfup = self.create_subscription(String,'selfup',self.SELFUP,10)
        self.selfup
        self.seebu = self.create_subscription(String,'seebu',self.SEEBU,10)
        self.seebu
        self.pushbu = self.create_subscription(String,'pushbu',self.PUSHBU,10)
        self.pushbu
        self.seele = self.create_subscription(String,'seele',self.SEELE,10)
        self.seele
        self.seeflo = self.create_subscription(String,'seeflo',self.SEEFLO,10)
        self.seeflo
        self.closeska = self.create_subscription(String,'closeska',self.CLOSESKA,10)
        self.closeska
        self.rest = self.create_subscription(String,'rest',self.REST,10)
        self.rest
        self.test1 = self.create_subscription(String,'test1',self.TEST1,10)
        self.test1
        self.test2 = self.create_subscription(String,'test2',self.TEST2,10)
        self.test2
        self.test3 = self.create_subscription(String,'test3',self.TEST3,10)
        self.test3

        self.joint_state_subscription = self.create_subscription(
            JointState,
            'joint_state',
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


        goal_kinematics_pose[0] = 0.198
        goal_kinematics_pose[1] = 0.001
        goal_kinematics_pose[2] = -0.030
        goal_kinematics_pose[3] = 0.695
        goal_kinematics_pose[4] = -0.003
        goal_kinematics_pose[5] = 0.721
        goal_kinematics_pose[6] = 0.003
        self.node.send_goal_task_space()


        
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
            self.goal_task_space.call_async(self.goal_task_space_req)
        except Exception as e:
            self.get_logger().info('Sending Goal Kinematic Pose failed %r' % (e,))

    def send_tool_control_request(self):
        self.tool_control_req.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        self.tool_control_req.joint_position.position = [goal_joint_angle[0], goal_joint_angle[1], goal_joint_angle[2], goal_joint_angle[3], goal_joint_angle[4]]
        self.tool_control_req.path_time = path_time

        try:
            self.tool_control_result = self.tool_control.call_async(self.tool_control_req)

        except Exception as e:
            self.get_logger().info('Tool control failed %r' % (e,))

    def GETSEE(self, msg):
            global up_location
            #고개 들고, 뻗고, 대기위치 온 뒤 들기
            goal_kinematics_pose[0] = 0.218
            goal_kinematics_pose[1] = 0.009
            goal_kinematics_pose[2] = 0.153
            goal_kinematics_pose[3] = 0.888
            goal_kinematics_pose[4] = -0.010
            goal_kinematics_pose[5] = 0.458
            goal_kinematics_pose[6] = 0.018
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = 0.330
            goal_kinematics_pose[1] = 0.020
            goal_kinematics_pose[2] = 0.129
            goal_kinematics_pose[3] = 0.948
            goal_kinematics_pose[4] = -0.010
            goal_kinematics_pose[5] = 0.316
            goal_kinematics_pose[6] = 0.028
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = 0.345
            goal_kinematics_pose[1] = 0.011
            goal_kinematics_pose[2] = 0.084
            goal_kinematics_pose[3] = 0.937
            goal_kinematics_pose[4] = -0.006
            goal_kinematics_pose[5] = 0.348
            goal_kinematics_pose[6] = 0.015
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = 0.232
            goal_kinematics_pose[1] = 0.015
            goal_kinematics_pose[2] = 0.062
            goal_kinematics_pose[3] = 0.724
            goal_kinematics_pose[4] = -0.023
            goal_kinematics_pose[5] = 0.689
            goal_kinematics_pose[6] = 0.024
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = 0.198
            goal_kinematics_pose[1] = 0.001
            goal_kinematics_pose[2] = -0.030
            goal_kinematics_pose[3] = 0.695
            goal_kinematics_pose[4] = -0.003
            goal_kinematics_pose[5] = 0.721
            goal_kinematics_pose[6] = 0.003
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "checknum"
            print(msg)
            msg_publisher = self.create_publisher(String, "checknum", 10)
            msg_publisher.publish(msg)

    
    def DOWNARM(self, msg):
            global up_location
            #초기 위치
            goal_kinematics_pose[0] = 0.198
            goal_kinematics_pose[1] = 0.001
            goal_kinematics_pose[2] = -0.030
            goal_kinematics_pose[3] = 0.695
            goal_kinematics_pose[4] = -0.003
            goal_kinematics_pose[5] = 0.721
            goal_kinematics_pose[6] = 0.003
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "goelvae"
            print(msg)
            msg_publisher = self.create_publisher(String, "goelvae", 10)
            msg_publisher.publish(msg)


    def UPARM(self, msg):
            global up_location
            global path_time
            #고개 든 뒤 엘베 탑승 버튼 바라보기
            goal_kinematics_pose[0] = 0.093
            goal_kinematics_pose[1] = -0.002
            goal_kinematics_pose[2] = 0.102
            goal_kinematics_pose[3] = 0.700
            goal_kinematics_pose[4] = 0.011
            goal_kinematics_pose[5] = 0.714
            goal_kinematics_pose[6] = -0.011
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = -0.111
            goal_kinematics_pose[1] = 0.128
            goal_kinematics_pose[2] = 0.144
            goal_kinematics_pose[3] = 0.390
            goal_kinematics_pose[4] = -0.006
            goal_kinematics_pose[5] = 0.002
            goal_kinematics_pose[6] = 0.921
            self.node.send_goal_task_space()
            self.node.open_manipulator_state_callback('STOPPED')
            time.sleep(4)
            

            msg = String()
            msg.data = "apart"
            print(msg)
            msg_publisher = self.create_publisher(String, "apart", 10)
            msg_publisher.publish(msg)


    def MOVEARM(self,msg):
            global up_location
            self.node.open_manipulator_state_callback('STOPPED')
            #탑승 버튼 누른 후 엘베 문 바라보기
            self.msg_data = msg.data
            distance = self.msg_data

            up_location = str(distance)
            up_location = up_location.split(" ")
            up_location = up_location[1]
            up_location = up_location.split(", ")   

            up_location[0] = float(up_location[0])
            up_location[1] = float(up_location[1])
            up_location[2] = float(up_location[2])

            goal_kinematics_pose[0] = present_kinematics_pose[0] + up_location[2]
            goal_kinematics_pose[1] = present_kinematics_pose[1] + up_location[0]
            goal_kinematics_pose[2] = present_kinematics_pose[2] + up_location[1]
            goal_kinematics_pose[3] = 0.370
            goal_kinematics_pose[4] = 0.009
            goal_kinematics_pose[5] = -0.003
            goal_kinematics_pose[6] = 0.930
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = -0.043
            goal_kinematics_pose[1] = 0.259
            goal_kinematics_pose[2] = 0.133
            goal_kinematics_pose[3] = 0.629
            goal_kinematics_pose[4] = 0.017
            goal_kinematics_pose[5] = -0.014
            goal_kinematics_pose[6] = 0.777
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "openska"
            print(msg)
            msg_publisher = self.create_publisher(String, "openska", 10)
            msg_publisher.publish(msg)


    def TURNARM(self, msg):
            global up_location
            #고개 돌려 좌측 층 버튼 보기
            goal_kinematics_pose[0] = -0.165
            goal_kinematics_pose[1] = 0.026
            goal_kinematics_pose[2] = 0.143
            goal_kinematics_pose[3] = 0.074
            goal_kinematics_pose[4] = 0.003
            goal_kinematics_pose[5] = -0.000
            goal_kinematics_pose[6] = 0.997
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "wherebu"
            print(msg)
            msg_publisher = self.create_publisher(String, "wherebu", 10)
            msg_publisher.publish(msg)


    def PUSHUP(self,msg):
            global up_location
            self.node.open_manipulator_state_callback('STOPPED')
            #전달 받은 버튼 거리값 이동 후 엘베 층 수 보기
            self.msg_data = msg.data
            distance = self.msg_data

            up_location = str(distance)
            up_location = up_location.split(" ")
            up_location = up_location[1]
            up_location = up_location.split(", ")

            up_location[0] = float(up_location[0])
            up_location[1] = float(up_location[1])
            up_location[2] = float(up_location[2])

            goal_kinematics_pose[0] = present_kinematics_pose[0] + up_location[2]
            goal_kinematics_pose[1] = present_kinematics_pose[1] + up_location[0]
            goal_kinematics_pose[2] = present_kinematics_pose[2] + up_location[1]
            self.node.send_goal_task_space()
            time.sleep(4)
            goal_kinematics_pose[0] = -0.260
            goal_kinematics_pose[1] = 0.040
            goal_kinematics_pose[2] = 0.176
            goal_kinematics_pose[3] = 0.074
            goal_kinematics_pose[4] = 0.028
            goal_kinematics_pose[5] = -0.002
            goal_kinematics_pose[6] = 0.997
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = 0.061
            goal_kinematics_pose[1] = 0.149
            goal_kinematics_pose[2] = 0.212
            goal_kinematics_pose[3] = 0.793
            goal_kinematics_pose[4] = 0.121
            goal_kinematics_pose[5] = -0.168
            goal_kinematics_pose[6] = 0.573
            self.node.send_goal_task_space()
            time.sleep(10)

            msg = String()
            msg.data = "whatflo"
            print(msg)
            msg_publisher = self.create_publisher(String, "whatflo", 10)
            msg_publisher.publish(msg)

            
    def SEEELVAE(self, msg):
            global up_location
            #엘베 문 바라보기
            goal_kinematics_pose[0] = 0.065
            goal_kinematics_pose[1] = 0.159
            goal_kinematics_pose[2] = 0.169
            goal_kinematics_pose[3] = 0.811
            goal_kinematics_pose[4] = 0.026
            goal_kinematics_pose[5] = -0.035
            goal_kinematics_pose[6] = 0.584
            self.node.send_goal_task_space()
            time.sleep(10)           

            msg = String()
            msg.data = "seedoor"
            print(msg)
            msg_publisher = self.create_publisher(String, "seedoor", 10)
            msg_publisher.publish(msg)


    def SEEHO(self, msg):
            global up_location
            #세대 호 수 보기
            goal_kinematics_pose[0] = 0.061
            goal_kinematics_pose[1] = 0.149
            goal_kinematics_pose[2] = 0.212
            goal_kinematics_pose[3] = 0.793
            goal_kinematics_pose[4] = 0.121
            goal_kinematics_pose[5] = -0.168
            goal_kinematics_pose[6] = 0.573
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "checkho"
            print(msg)
            msg_publisher = self.create_publisher(String, "checkho", 10)
            msg_publisher.publish(msg)


    def SEEBOX(self, msg):
            global up_location
            #바구니 보기
            goal_kinematics_pose[0] = 0.093
            goal_kinematics_pose[1] = -0.002
            goal_kinematics_pose[2] = 0.102
            goal_kinematics_pose[3] = 0.700
            goal_kinematics_pose[4] = 0.011
            goal_kinematics_pose[5] = 0.714
            goal_kinematics_pose[6] = -0.011
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "wherebox"
            print(msg)
            msg_publisher = self.create_publisher(String, "wherebox", 10)
            msg_publisher.publish(msg)


    def PUSHZ(self, msg):
            global up_location
            self.node.open_manipulator_state_callback('STOPPED')
            #box x,y 좌표로만 이동
            self.msg_data = msg.data
            distance = self.msg_data

            up_location = str(distance)
            up_location = up_location.split(" ")
            up_location = up_location[1]
            up_location = up_location.split(", ")

            up_location[0] = float(up_location[0])
            up_location[1] = float(up_location[1])
            up_location[2] = float(up_location[2])

            goal_kinematics_pose[0] = present_kinematics_pose[0] + up_location[2]
            goal_kinematics_pose[1] = present_kinematics_pose[1] + up_location[0]
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "agreeqr"
            print(msg)
            msg_publisher = self.create_publisher(String, "agreeqr", 10)
            msg_publisher.publish(msg)
    

    def PULLZ(self, msg):
            global up_location
            self.node.open_manipulator_state_callback('STOPPED')
            #grap 열기 -> z좌표 이동 -> grap 닫기 -> 고개 들기 -> 배출 위치로 이동 -> grap 열기 -> 바구니 보기
            goal_joint_angle[4] = 0.01
            self.node.send_goal_joint_space(path_time)
            time.sleep(1)

            self.msg_data = msg.data
            distance = self.msg_data

            up_location = str(distance)
            up_location = up_location.split(" ")
            up_location = up_location[1]
            up_location = up_location.split(", ")

            up_location[0] = float(up_location[0])
            up_location[1] = float(up_location[1])
            up_location[2] = float(up_location[2])

            goal_kinematics_pose[2] = present_kinematics_pose[2] + up_location[1]
            self.node.send_goal_task_space()
            time.sleep(2)

            goal_joint_angle[4] = -0.01
            self.node.send_goal_joint_space()
            time.sleep(1)

            goal_kinematics_pose[0] = 0.016
            goal_kinematics_pose[1] = -0.234
            goal_kinematics_pose[2] = 0.105
            goal_kinematics_pose[3] = 0.670
            goal_kinematics_pose[4] = 0.240
            goal_kinematics_pose[5] = 0.244
            goal_kinematics_pose[6] = -0.660
            self.node.send_goal_task_space()
            time.sleep(1)

            goal_kinematics_pose[0] = 0.029
            goal_kinematics_pose[1] = -0.206
            goal_kinematics_pose[2] = -0.059
            goal_kinematics_pose[3] = 0.526
            goal_kinematics_pose[4] = 0.474
            goal_kinematics_pose[5] = 0.515
            goal_kinematics_pose[6] = -0.484
            self.node.send_goal_task_space()
            time.sleep(2)

            goal_joint_angle[4] = 0.01
            self.node.send_goal_joint_space(path_time)
            time.sleep(1)

            goal_kinematics_pose[0] = 0.016
            goal_kinematics_pose[1] = -0.234
            goal_kinematics_pose[2] = 0.105
            goal_kinematics_pose[3] = 0.670
            goal_kinematics_pose[4] = 0.240
            goal_kinematics_pose[5] = 0.244
            goal_kinematics_pose[6] = -0.660
            self.node.send_goal_task_space()
            time.sleep(2)

            goal_kinematics_pose[0] = 0.093
            goal_kinematics_pose[1] = -0.002
            goal_kinematics_pose[2] = 0.102
            goal_kinematics_pose[3] = 0.700
            goal_kinematics_pose[4] = 0.011
            goal_kinematics_pose[5] = 0.714
            goal_kinematics_pose[6] = -0.011
            self.node.send_goal_task_space()
            time.sleep(2)


            msg = String()
            msg.data = "remain"
            print(msg)
            msg_publisher = self.create_publisher(String, "remain", 10)
            msg_publisher.publish(msg)
    

    def KIMCHI(self, msg):
            global up_location
            #인증샷 위치
            goal_kinematics_pose[0] = 0.009
            goal_kinematics_pose[1] = -0.245
            goal_kinematics_pose[2] = -0.078
            goal_kinematics_pose[3] = 0.702
            goal_kinematics_pose[4] = 0.014
            goal_kinematics_pose[5] = 0.014
            goal_kinematics_pose[6] = -0.712
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "selfie"
            print(msg)
            msg_publisher = self.create_publisher(String, "selfie", 10)
            msg_publisher.publish(msg)

    def SELFUP(self, msg):
            global up_location
            #고개 들기
            goal_kinematics_pose[0] = 0.093
            goal_kinematics_pose[1] = -0.002
            goal_kinematics_pose[2] = 0.102
            goal_kinematics_pose[3] = 0.700
            goal_kinematics_pose[4] = 0.011
            goal_kinematics_pose[5] = 0.714
            goal_kinematics_pose[6] = -0.011
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "gobu"
            print(msg)
            msg_publisher = self.create_publisher(String, "gobu", 10)
            msg_publisher.publish(msg)
    

    def SEEBU(self, msg):
            global up_location

            goal_kinematics_pose[0] = -0.165
            goal_kinematics_pose[1] = 0.026
            goal_kinematics_pose[2] = 0.143
            goal_kinematics_pose[3] = 0.074
            goal_kinematics_pose[4] = 0.003
            goal_kinematics_pose[5] = -0.000
            goal_kinematics_pose[6] = 0.997
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "seedis"
            print(msg)
            msg_publisher = self.create_publisher(String, "seedis", 10)
            msg_publisher.publish(msg)
    

    def PUSHBU(self, msg):
            global up_location
            self.node.open_manipulator_state_callback('STOPPED')

            #버튼 누르고 엘베 문 보기
            self.msg_data = msg.data
            distance = self.msg_data

            up_location = str(distance)
            up_location = up_location.split(" ")
            up_location = up_location[1]
            up_location = up_location.split(", ")

            up_location[0] = float(up_location[0])
            up_location[1] = float(up_location[1])
            up_location[2] = float(up_location[2])

            goal_kinematics_pose[0] = present_kinematics_pose[0] + up_location[2]
            goal_kinematics_pose[1] = present_kinematics_pose[1] + up_location[0]
            goal_kinematics_pose[2] = present_kinematics_pose[2] + up_location[1]
            self.node.send_goal_task_space()
            time.sleep(2)
            goal_kinematics_pose[0] = 0.050
            goal_kinematics_pose[1] = 0.206
            goal_kinematics_pose[2] = 0.154
            goal_kinematics_pose[3] = 0.693
            goal_kinematics_pose[4] = 0.003
            goal_kinematics_pose[5] = -0.003
            goal_kinematics_pose[6] = 0.721
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "doorska"
            print(msg)
            msg_publisher = self.create_publisher(String, "doorska", 10)
            msg_publisher.publish(msg)
    

    def SEELE(self, msg):
            global up_location

            #좌측 보기
            goal_kinematics_pose[0] = -0.165
            goal_kinematics_pose[1] = 0.026
            goal_kinematics_pose[2] = 0.143
            goal_kinematics_pose[3] = 0.074
            goal_kinematics_pose[4] = 0.003
            goal_kinematics_pose[5] = -0.000
            goal_kinematics_pose[6] = 0.997
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "firbu"
            print(msg)
            msg_publisher = self.create_publisher(String, "firbu", 10)
            msg_publisher.publish(msg)
    

    def SEEFLO(self, msg):
            global up_location
            self.node.open_manipulator_state_callback('STOPPED')
            #누르고 층수 보기
            self.msg_data = msg.data
            distance = self.msg_data

            up_location = str(distance)
            up_location = up_location.split(" ")
            up_location = up_location[1]
            up_location = up_location.split(", ")

            up_location[0] = float(up_location[0])
            up_location[1] = float(up_location[1])
            up_location[2] = float(up_location[2])

            goal_kinematics_pose[0] = present_kinematics_pose[0] + up_location[2]
            goal_kinematics_pose[1] = present_kinematics_pose[1] + up_location[0]
            goal_kinematics_pose[2] = present_kinematics_pose[2] + up_location[1]
            self.node.send_goal_task_space()
            time.sleep(1)
            goal_kinematics_pose[0] = 0.061
            goal_kinematics_pose[1] = 0.149
            goal_kinematics_pose[2] = 0.212
            goal_kinematics_pose[3] = 0.793
            goal_kinematics_pose[4] = 0.121
            goal_kinematics_pose[5] = -0.168
            goal_kinematics_pose[6] = 0.573
            self.node.send_goal_task_space()
            time.sleep(2)

            msg = String()
            msg.data = "seefir"
            print(msg)
            msg_publisher = self.create_publisher(String, "seefir", 10)
            msg_publisher.publish(msg)
    

    def CLOSESKA(self, msg):
            global up_location

            #엘베 문 보기
            goal_kinematics_pose[0] = 0.065
            goal_kinematics_pose[1] = 0.159
            goal_kinematics_pose[2] = 0.169
            goal_kinematics_pose[3] = 0.811
            goal_kinematics_pose[4] = 0.026
            goal_kinematics_pose[5] = -0.035
            goal_kinematics_pose[6] = 0.584
            self.node.send_goal_task_space()
            time.sleep(2) 

            msg = String()
            msg.data = "getout"
            print(msg)
            msg_publisher = self.create_publisher(String, "getout", 10)
            msg_publisher.publish(msg)
    

    def REST(self, msg):
            global up_location

            #팔 바구니 쪽으로 내리기
            goal_kinematics_pose[0] = 0.198
            goal_kinematics_pose[1] = 0.001
            goal_kinematics_pose[2] = -0.030
            goal_kinematics_pose[3] = 0.695
            goal_kinematics_pose[4] = -0.003
            goal_kinematics_pose[5] = 0.721
            goal_kinematics_pose[6] = 0.003
            self.node.send_goal_task_space()
            time.sleep(2)

            print('finish!!!!!')


    def joint_state_callback(self, msg):
        present_joint_angle[0] = msg.position[0]
        present_joint_angle[1] = msg.position[1]
        present_joint_angle[2] = msg.position[2]
        present_joint_angle[3] = msg.position[3]
        present_joint_angle[4] = msg.position[4]


    # 목적지 넣습니다.
    def kinematics_pose_callback(self, msg):
        present_kinematics_pose[0] = msg.pose.position.x
        present_kinematics_pose[1] = msg.pose.position.y
        present_kinematics_pose[2] = msg.pose.position.z
        present_kinematics_pose[3] = msg.pose.orientation.w
        present_kinematics_pose[4] = msg.pose.orientation.x
        present_kinematics_pose[5] = msg.pose.orientation.y
        present_kinematics_pose[6] = msg.pose.orientation.z

    # 멈춰있을때, 현재 위치를 담습니다.
    def open_manipulator_state_callback(self, msg):
        if msg == 'STOPPED':
            for index in range(0, 7):
                present_kinematics_pose[index] = goal_kinematics_pose[index]
                #goal_kinematics_pose[index] = present_kinematics_pose[index]


def main(args=None):
    settings = None

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    try:
        rclpy.init(args=args)
    except Exception as e:
        print(e)
        return

    try:
        node = TeleopSubscriber()
        rclpy.spin(node)  # Spin the node
    except Exception as e:
        print(e)

    finally:
        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
