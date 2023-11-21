import os
import sys
from rclpy.node import Node
from sensor_msgs.msg import Image
import rclpy
from std_msgs.msg import String
import time
import mysql.connector
import pandas as pd

distance = 0
mail = 0
target = 3
timer = 0
ska = 0

# Define file paths as variables
base_path = '/home/boma/Desktop/'
test_file_path = os.path.join(base_path, 'test.txt')
pixel_data_file_path = os.path.join(base_path, 'pixel_data.txt')
qr_data_file_path = os.path.join(base_path, 'qrbarcode_data.txt')


class MAIL(Node):
    def __init__(self):
        super().__init__('mail')
        
        # Callho
        self.subscription = self.create_subscription(String, 'callho', self.Callho, 10)
        self.subscription

        # Checknum
        self.subscription = self.create_subscription(String, 'checknum', self.Checknum, 10)
        self.subscription

        # apart
        self.subscription = self.create_subscription(String, 'apart', self.Apart, 10)
        self.subscription

        # openska
        self.subscription = self.create_subscription(String, 'openska', self.Openska, 10)
        self.subscription

        # wherebu
        self.subscription = self.create_subscription(String, 'wherebu', self.Wherebu, 10)
        self.subscription

        # whatflo
        self.subscription = self.create_subscription(String, 'whatflo', self.Whatflo, 10)
        self.subscription

        # Seedoor
        self.subscription = self.create_subscription(String, 'seedoor', self.Seedoor, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'checkho', self.Checkho, 10)
        self.subscription

        # 후반경기 시작

        # boxdis
        self.subscription = self.create_subscription(String, 'boxdis', self.Boxdis, 10)
        self.subscription

        # agreeqr
        self.subscription = self.create_subscription(String, 'agreeqr', self.Agreeqr, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'remain', self.Remain, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'selfie', self.Selfie, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'seedis', self.Seedis, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'doorska', self.Doorska, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'firbu', self.Firbu, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'seefir', self.Seefir, 10)
        self.subscription

        # checkho
        self.subscription = self.create_subscription(String, 'getout', self.Getout, 10)
        self.subscription

    # GOAL 저장하기
    def Callho(self, msg):
        global goal_name
        global goal_floor
        global goal_line
        mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )
        cur = mydb.cursor(buffered=True)
        cur.execute("select name,floor,line from address")
        result = cur.fetchall()
        # goal_name = str(result[2]).replace('(', '').replace(')', '').replace("'", "").split(', ')[0]
        goal_floor = str(result[2]).replace('(', '').replace(')', '').replace("'", "").split(', ')[1]
        goal_line = str(result[2]).replace('(', '').replace(')', '').replace("'", "").split(', ')[2]
        # df = pd.DataFrame(result)
        # for result_iterator in result:
        #     print(result_iterator)
        mydb.close()


    # M>R>M : 택배개수 일치 확인 < 택배 있는지만 확인
    def Checknum(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('1')  # yolo 켜기
            time.sleep(4)
        with open(test_file_path, 'w') as f:
            f.write('4')  # yolo로 좌표 넣기
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()
        if distance != 0:
            msg = String()
            msg.data = 'downarm'
            publisher = self.create_publisher(String, 'downarm', 10)
            publisher.publish(msg)  # 메시지 게시
            print(msg)
            with open(pixel_data_file_path, 'w') as f:
                f.write('')

    # M>R>M : 버튼 거리값 가져오기
    def Apart(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('4')  # yolo로 좌표 넣기
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()

        msg = String()
        msg.data = distance
        publisher = self.create_publisher(String, 'movearm', 10)
        publisher.publish(msg)  # 메시지 게시
        print(msg)
        with open(pixel_data_file_path, 'w') as f:
            f.write('')

    # M>R>T : 엘베문 열림 감지
    def Openska(self, msg):
        global ska
        with open(test_file_path, 'w') as f:
            f.write(3)  # yolo 끄기
        print('yolo close')
        time.sleep(3.5)
        with open(test_file_path, 'w') as f:
            f.write(5)  # door 키기
        print('door open')
        time.sleep(3.5)
        print('check')
        with open(test_file_path, 'r') as s:
            ska = s.read()
            if ska == 6:
                msg = String()
                msg.data = 'takein'
                publisher = self.create_publisher(String, 'takein', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)
                with open(test_file_path, 'w') as f:
                    f.write(7)  # door 끄기

    # M>R>M : 버튼 거리 뽑기
    def Wherebu(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('1')  # yolo 켜기
            time.sleep(4)
        with open(test_file_path, 'w') as f:
            f.write('4')  # yolo로 좌표 넣기
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()
        msg = String()
        msg.data = distance
        publisher = self.create_publisher(String, 'pushup', 10)
        publisher.publish(msg)  # 메시지 게시
        print(msg)
        with open(pixel_data_file_path, 'w') as f:
            f.write('')

    # M>R>M : 목표 층수 확인
    def Whatflo(self, msg):
        with open(test_file_path, 'w') as f:
            f.write('4')  # write
        with open(pixel_data_file_path, 'r') as d:
            detect = d.read()
        detect_1 = int(detect.split(":")[0])
        detect_1 = detect_1[:0]
        while detect_1 != goal_floor:
            with open(test_file_path, 'w') as f:
                f.write('4')
            with open(pixel_data_file_path, 'r') as d:
                detect = d.read()
            detect_1 = int(detect.split(":")[0])
            detect_1 = detect_1[:0]
            msg = String()
            # 판단 후에 가야할 때 publish하기
            if goal_floor == detect_1:
                msg.data = 'seeelvae'
                publisher = self.create_publisher(String, 'seeelvae', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)

    # M>R>T : 엘베 문 열림 감지
    def Seedoor(self, msg):
        global ska
        with open(test_file_path, 'w') as f:
            f.write('3')  # yolo 끄기
        with open(test_file_path, 'w') as f:
            f.write('5')  # door 키기
        with open(test_file_path, 'r') as s:
            ska = s.read()
            if ska == 6:
                msg = String()
                msg.data = 'takein'
                publisher = self.create_publisher(String, 'takein', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)
                with open(test_file_path, 'w') as f:
                    f.write('7')  # door 끄기

    # M>R>M : 몇호인지 일치 확인
    def Checkho(self, msg):
        with open(test_file_path, 'w') as f:
            f.write('1')
            time.sleep(4)
        with open(test_file_path, 'w') as f:
            f.write('4')
        with open(pixel_data_file_path, 'r') as d:
            detect = d.read()
        detect = int(detect.split(":")[0])
        detect = detect[1:2]
        if goal_line == detect:
            msg = String()
            msg.data = 'detected'
            publisher = self.create_publisher(String, 'seebox', 10)
            publisher.publish(msg)  # 메시지 게시
            print(msg)
            with open(pixel_data_file_path, 'w') as f:
                f.write('')

    # M>R>M: 바구니 거리주기
    def Boxdis(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('4')  # pixel에 적게 만듬
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()  # pixel 읽기
        msg = String()
        msg.data = distance
        publisher = self.create_publisher(String, 'pushz', 10)
        publisher.publish(msg)  # 메시지 게시
        print(msg)
        with open(pixel_data_file_path, 'w') as f:
            f.write('')
    
    # M>R>M: QR찍고 일치 확인, 일치시 거리주기
    def Agreeqr(self, msg):
        global qrinfo
        global distance
        # with open(test_file_path, 'w') as f:
        #     f.write('3')  # yolo 끄기
        # with open(test_file_path, 'w') as f:
        #     f.write('2')  # qr 열기
        # with open(qr_data_file_path, 'r') as q:
        #     qrinfo = q.read()
        #     time.sleep(1)

        # qrinfo 정제하기

        # 택배와 qr이 일치할 경우
        if goal_line == qrinfo:
            with open(test_file_path, 'w') as f:
                f.write('1')  # yolo 키기
                time.sleep(4)
            with open(test_file_path, 'w') as f:
                f.write('4')  # yolo로 좌표 넣기
            with open(pixel_data_file_path, 'r') as d:
                distance = d.read()
            msg = String()
            msg.data = distance
            publisher = self.create_publisher(String, 'pullz', 10)
            publisher.publish(msg)  # 메시지 게시
            print(msg)
            with open(pixel_data_file_path, 'w') as f:
                f.write('')

        ## 택배와 qr이 일치하지 않을 경우     (for cut scenario)
        # if goal_line != qrinfo:
        #     msg = String()
        #     msg.data = '다른 box를 확인하세요'
        #     publisher = self.create_publisher(String, 'pullz', 10)
        #     publisher.publish(msg)  # 메시지 게시
        #     print(msg)
        #     with open(pixel_data_file_path, 'w') as f:
        #         f.write('')
        #         with open(test_file_path, 'w') as f:
        #             f.write('3')  # qr 끄기


    # M>R>T: 남은거 확인, DETECT 없을 시 TOPIC
    def Remain(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('4')  # yolo write
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()
        if distance == '':
            msg = String()
            msg.data = 'littlego'
            publisher = self.create_publisher(String, 'littlego', 10)
            publisher.publish(msg)  # 메시지 게시
            print(msg)
            with open(pixel_data_file_path, 'w') as f:
                f.write('')

    # M>R>M: 인증샷
    def Selfie(self, msg):
        msg = String()
        msg.data = '인증샷'
        publisher = self.create_publisher(String, 'selfup', 10)
        publisher.publish(msg)  # 메시지 게시
        print(msg)

    # M>R>M: 하강버튼 거리보내기
    def Seedis(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('4')  # yolo write
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()  # pixel 읽기
        msg = String()
        msg.data = distance
        publisher = self.create_publisher(String, 'pushbu', 10)
        publisher.publish(msg)  # 메시지 게시
        print(msg)
        with open(pixel_data_file_path, 'w') as f:
            f.write('')

    # M>R>T: 문열림 확인
    def Doorska(self, msg):
        global ska
        with open(test_file_path, 'w') as f:
            f.write('3')  # yolo 끄기
        with open(test_file_path, 'w') as f:
            f.write('5')  # door 키기
        with open(test_file_path, 'r') as s:
            ska = s.read()
            if ska == 6:
                msg = String()
                msg.data = 'getin'
                publisher = self.create_publisher(String, 'getin', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)
                with open(test_file_path, 'w') as f:
                    f.write('7')  # door 끄기

    # M>R>M: 1층 버튼 거리주기
    def Firbu(self, msg):
        global distance
        with open(test_file_path, 'w') as f:
            f.write('4')  # yolo write
        with open(pixel_data_file_path, 'r') as d:
            distance = d.read()  # pixel 읽기
        msg = String()
        msg.data = distance
        publisher = self.create_publisher(String, 'seeflo', 10)
        publisher.publish(msg)  # 메시지 게시
        print(msg)
        with open(pixel_data_file_path, 'w') as f:
            f.write('')


    # M>R>M: 현재엘베 상태-1층 인식
    def Seefir(self, msg):
        goal_floor = 1
        with open(test_file_path, 'w') as f:
            f.write('4')  # write
        with open(pixel_data_file_path, 'r') as d:
            detect = d.read()
        detect_1 = int(detect.split(":")[0])
        detect_1 = detect_1[:0]
        while detect_1 != goal_floor:
            with open(test_file_path, 'w') as f:
                f.write('4')
            with open(pixel_data_file_path, 'r') as d:
                detect = d.read()
            detect_1 = int(detect.split(":")[0])
            detect_1 = detect_1[:0]
            msg = String()
            # 판단 후에 가야할 때 publish하기
            if goal_floor == detect_1:
                msg.data = 'closeska'
                publisher = self.create_publisher(String, 'closeska', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)


    # M>R>T: 열림 인지
    def Getout(self, msg):
        global ska
        with open(test_file_path, 'w') as f:
            f.write('3')  # yolo 끄기
        with open(test_file_path, 'w') as f:
            f.write('5')  # door 키기
        with open(test_file_path, 'r') as s:
            ska = s.read()
            if ska == 6:
                msg = String()
                msg.data = 'gohome'
                publisher = self.create_publisher(String, 'gohome', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)
                with open(test_file_path, 'w') as f:
                    f.write('7')  # door 끄기

        with open(test_file_path, 'w') as f:
            f.write('3')  # yolo 끄기
        with open(test_file_path, 'w') as f:
            f.write('5')  # door 키기
        with open(test_file_path, 'r') as s:
            ska = s.read()
            if ska == 6:
                msg = String()
                msg.data = 'rest'
                publisher = self.create_publisher(String, 'rest', 10)
                publisher.publish(msg)  # 메시지 게시
                print(msg)
                with open(test_file_path, 'w') as f:
                    f.write('7')  # door 끄기



def main(args=None):
    rclpy.init(args=args)
    node = MAIL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()