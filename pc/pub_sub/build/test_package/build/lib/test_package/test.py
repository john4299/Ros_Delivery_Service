import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import String



class test(Node):
    
    def __init__(self):
        super().__init__('fuck')
        self.publish=self.create_publisher(
            String,
            'u',
            10
        )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        
    def callback(self):
        msg = String()  # String 메시지 생성
        msg.data = "예봄이"  # 메시지 데이터 설정
        self.publish.publish(msg)  # 메시지 게시
        print(msg)
        
    
def main():
    rp.init()
    
    node = test()
    
    rp.spin(node)
    
    node.destroy_node()
    rp.shutdown()
    
    
    
if __name__ == '__main__':
    main()
