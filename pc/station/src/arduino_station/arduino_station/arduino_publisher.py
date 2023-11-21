import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time

class ArduinoPublisher(Node):

    def __init__(self):
        super().__init__('arduino_publisher')
        self.publisher_ = self.create_publisher(String, '/arduino_data', 10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.data_callback)

        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter("baud_rate", 11520) #파라미터를 불러온다
        
        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value #파라미터의 값을 저장한다
        
        self.ser = serial.Serial(arduino_port, baud_rate) #아두이노와 시리얼 연결
        time.sleep(3) #아두이노 연결 대기
        self.get_logger().info('스테이션 연결!!')

    def data_callback(self):
        msg = String()
        msg.data = self.ser.readline().decode()
        msg.data = msg.data.replace("\r", "").replace("\n", "")
        self.publisher_.publish(msg)
        self.get_logger().info('{0}'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)

    arduino_publisher = ArduinoPublisher()
    
    rclpy.spin(arduino_publisher)
    
    arduino_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    






