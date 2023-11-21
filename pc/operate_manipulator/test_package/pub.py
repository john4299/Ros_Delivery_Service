import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import os
state = 0
toggle = 0
timer = 0

class Publisher(Node):

    def __init__(self):
        super().__init__('pub')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        # os.sys('python3 /home/boma/colcon_ws/src/best/detect_RS3.py --weights final.pt --conf-thres 0.5')

    # a = '바보'

    # if a == '바보'
    #     self.publisher_ = self.create_publisher(String, 'topic', 10)
    #     timer_period = 0.5  # seconds
    #     self.timer = self.create_timer(timer_period, self.callback)
    #     a = '멍청이'


    def callback(self):
        global state
        global toggle

        msg = String()  # String 메시지 생성
        
        if state == 0:
            toggle = 0

        # std_msgs.msg.String(data='wait')라고 나옴
        if toggle == 0:
            with open('/home/boma/state.txt', 'r') as f:                   
                state = f.read()
                state = float(state)
            msg.data = "wait"  # 메시지 데이터 설정
            self.publisher_.publish(msg)  # 메시지 게시
            print(msg)
            if state != 0:
                toggle = 1

        else:
            if state == 1:
                msg.data = "stay1"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0

            elif state == 2:
                msg.data = "stay2"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0

            elif state == 3:
                msg.data = "catch"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0

            elif state == 4:
                msg.data = "push"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0

            elif state == 5:
                msg.data = "home"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0

            elif state == 6:
                msg.data = "grap"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0

            elif state == 7:
                msg.data = "qr"  # 메시지 데이터 설정
                self.publisher_.publish(msg)  # 메시지 게시
                print(msg)
                toggle = 0
        
        
def main(args=None):
    rclpy.init(args=args)

    node = Publisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()