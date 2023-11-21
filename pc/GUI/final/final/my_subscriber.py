import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class Subscriber(Node):
    
    def __init__(self):
        super().__init__('sub')
        self.subscription = self.create_subscription(
            String,
            'test',     # project topic name : selfie
            self.callback,
            10)
        self.subscription
       
    
    def callback(self, msg):
        self.msg_data = msg.data 
        if(self.msg_data=="topic data"):
            
            self.get_logger().info('Message received on my_topic!')
            
            # python file execute
            try:
                subprocess.run(['python3', '/home/computer520/ros2_study/src/final/final/alarm2.py'])
            except Exception as e:
                self.get_logger().error(f'Error executing the script: {e}')
            
            

def main(args=None):
    rp.init(args=args)
    
    node = Subscriber()
    rp.spin(node)
    
    node.destroy_node() 
    rp.shutdown()

if __name__ == '__main__':
    main()