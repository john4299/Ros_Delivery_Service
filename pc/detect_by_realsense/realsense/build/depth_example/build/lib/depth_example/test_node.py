import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import argparse
import time
from pathlib import Path
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import numpy as np
from ultralytics import YOLO
import os
import math
import os.path as osp
from tqdm import tqdm

import argparse #명령행 인자를 파싱하기 위한 ARGPARSE 모듈을 임포트 합니다.
import os #운영 체제 관련 작업을 수행하기 위한 OS 모듈을 임포트 합니다.
import os.path as osp #os.path 모듈을 osp별칭으로 임포트 합니다.
import torch #pytorch 라이브러리를 임포트 합니다.

import sys
sys.path.append('/home')
from noma.venv.REAL.YOLOv6.yolov6.utils.events import LOGGER #YOLOv6라이브러리 에서LOGGER를 가져옵니다.
from noma.venv.REAL.YOLOv6.yolov6.utils.events import LOGGER, load_yaml
from noma.venv.REAL.YOLOv6.yolov6.layers.common import DetectBackend
from noma.venv.REAL.YOLOv6.yolov6.data.data_augment import letterbox
from noma.venv.REAL.YOLOv6.yolov6.data.datasets import LoadData
from noma.venv.REAL.YOLOv6.yolov6.utils.nms import non_max_suppression



class DetectNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.colcor_subscriber = self.create_subscription(
            Image,
            '/camera/color/image_raw',#이미지 토픽
            self.color_image_callback,
            10
        )
        self.detect_publisher = self.create_publisher(
            Int32MultiArray,
            '/person_detect',
            10
        )
        self.cv_bridge = CvBridge()

        #self.model = YOLO('/home/noma/venv/REAL/YOLOv6/weights/yolo6.pt') #욜로 모델 불러오기
        self.model = YOLO('/home/noma/Downloads/yolov6n.pt')

    def color_image_callback(self, amg): #add_help인자를 기본값으로 받는 get_args_parser 함수를 정의합니다.
        # Build a YOLOv6n model from scratch
        # Display model information (optional)
        self.model.info()

        # Train the model on the COCO8 example dataset for 100 epochs
        results = self.model.train(data='coco8.yaml', epochs=100, imgsz=640)

        # Run inference with the YOLOv6n model on the 'bus.jpg' image
        results = model('path/to/bus.jpg')

def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()