import os
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
from std_msgs.msg import String


yolo = 'python3' + ' /home/boma/Robot-repo-1/yolov7/detect_final.py' + ' --weights' + ' /home/boma/Robot-repo-1/yolov7/final.pt' + ' --conf-thres 0.6'
qr = 'python3' + ' /home/boma/Desktop/qr.py'
door = 'python3' + ' /home/boma/Desktop/depth_el.py'
label = 'python3' + ' /home/boma/Robot-repo-1/yolov7/detect_final.py' + ' --weights' + ' /home/boma/Robot-repo-1/yolov7/lastbest.pt' + ' --conf-thres 0.6'
# pyqt = 'python3' + '/home/boma/Desktop/alarm2.py'

number = 0
munber = 0
tak = 1
class REALSENSE(Node):
    def __init__(self):
        super().__init__('realsesne')

def main(args=None):
    while tak == 1:
        with open('/home/boma/Desktop/test.txt', 'r') as f:
            number = f.read()
            print(number)
            if number == '1':
                os.system(yolo)
                with open('/home/boma/Desktop/test.txt', 'w') as f:
                    f.write(0)
            elif number == '2':
                os.system(qr)
                with open('/home/boma/Desktop/test.txt', 'w') as f:
                    f.write(0)
            elif number == '5':
                os.system(door)
                with open('/home/boma/Desktop/test.txt', 'w') as f:
                    f.write(0)
            elif number == '8':
                os.system(label)
                with open('/home/boma/Desktop/test.txt', 'w') as f:
                    f.write(0)
        # with open('/home/boma/Desktop/pyqt_test.txt', 'r') as f:
        #     munber = f.read()
        #     munber = int(munber)
        #     if munber == 1:
        #         os.system(pyqt)
        #         with open('/home/boma/Desktop/pyqt_test.txt', 'w') as f:
        #             f.write('0')


if __name__ == '__main__':
    main()