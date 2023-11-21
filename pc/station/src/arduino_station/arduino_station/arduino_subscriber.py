import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial
import time
import os

import qrcode
import cv2
import pyzbar.pyzbar as pyzbar
from playsound import playsound
import mysql.connector
import re
import numpy as np
import pandas as pd

robot_state = 0
floor_information = 0

mydb = mysql.connector.connect(
    host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
    port = 3306,
    user = "admin",
    password = "12345678",
    database = "delivery"
)

def extract_floor_from_address(address):
    match = re.search(r'(\d{3,4})호', address)
    if match:
        ho = match.group(1)
        return int(ho[:2]) if len(ho) == 4 else int(ho[0]) if len(ho) == 3 else None
    else:
        return None
    
def extract_line_from_address(address):
    match = re.search(r'(\d{3,4})호', address)
    if match:
        ho = match.group(1)
        return int(ho[3:]) if len(ho) == 4 else int(ho[2:]) if len(ho) == 3 else None
    else:
        return None
    
def invoice_save():
    with open('./test.txt', 'w') as f:
        f.write('0')

    data_list = []
    used_codes = []
    split_code = []
    window_closed = False  # 창을 닫을지 여부를 나타내는 변수

    try:
        f = open("./qrbarcode_data.txt", 'r', encoding='utf8')
        data_list = f.readlines()
    except FileNotFoundError:
        pass
    else:
        f.close()

    cap = cv2.VideoCapture(2, cv2.CAP_ANY)

    for i in data_list:
        used_codes.append(i.rsplit('\n'))

    while True:
        success, frame = cap.read()
        with open('./test.txt', 'r') as f:
            number = f.read()
            number = int(number)
            
        if success:
            cv2.imshow('cam', frame)
            
            for code in pyzbar.decode(frame):
                cv2.imwrite('my_qr_code.png', frame)
                my_code = code.data.decode('utf-8')
                split_data = my_code.split(',')
                # print(split_data)
                if my_code not in used_codes:
                    print('인식 성공 : ' , my_code)
                    # print(my_code.split(','))
                    split_code = my_code.split(',')
                  #  playsound("qrbarcode_beep.mp3")
                    used_codes.append(my_code)
                    # print(used_codes)

                    f2 = open('./qrbarcode_data.txt', 'a', encoding='utf8')
                    f2.write(my_code+'\n')
                    f2.close()
                    
                    window_closed = True  # 창을 닫을 준비
                    break
                else:
                    print('이미 인식된 코드입니다.')
               #     playsound("qrbarcode_beep.mp3")

        key = cv2.waitKey(1)
        if key == 27:  # 'Esc' 키를 누르면 루프를 종료
            break
            
        if number == 3:
            with open('./test.txt', 'w') as f:
                f.write('0')
            break

        if window_closed:  # 창을 닫을 준비가 되면
            cv2.destroyAllWindows()  # 창을 닫음
            break


    if split_code:
        address = split_code[0]
        name = split_code[1]
        phone_number = split_code[2]
        floor = extract_floor_from_address(address)
        line = extract_line_from_address(address)

        mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )

        cur = mydb.cursor()

        cur.execute(f"INSERT INTO address (address, floor, line, name, phonenumber) VALUES ('{address}', '{floor}', '{line}', '{name}', '{phone_number}')")

        cur = mydb.cursor(buffered=True)
        cur.execute("SELECT * from address")

        result = cur.fetchall()
        # for result_iterator in result:
            # print(result_iterator)

        mydb.commit()
        mydb.close()

def read_floor():

    mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )

    global floor_information
    
    cur = mydb.cursor(buffered=True)
    cur.execute("SELECT * from address")

    result = cur.fetchall()
    df = pd.DataFrame(result)
    floor_information = df.iloc[-1,1]
    print(floor_information)

    mydb.commit()
    mydb.close()


class ArduinoSubscriber(Node):

    def __init__(self):

        mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )

        cur = mydb.cursor()

        try: 
            cur.execute("DROP TABLE address")
            cur.execute("CREATE TABLE address (address varchar(64), floor varchar(16), line varchar(16), name varchar(16), phonenumber varchar(16))")
        except:
            cur.execute("CREATE TABLE address (address varchar(64), floor varchar(16), line varchar(16), name varchar(16), phonenumber varchar(16))")

        mydb.commit()
        mydb.close()

        super().__init__('arduino_subscriber')

        self.subscriber_ = self.create_subscription(String, '/arduino_data', self.arduino_callback, 10)
        
        self.subscriber_ = self.create_subscription(String, '/robot_state', self.robot_callback, 10)


        self.declare_parameter('arduino_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)

        arduino_port = self.get_parameter('arduino_port').value
        baud_rate = self.get_parameter('baud_rate').value

        self.ser = serial.Serial(arduino_port, baud_rate)
        time.sleep(3)
        self.get_logger().info('스테이션 아누이노 연결!')


    def arduino_callback(self, msg):
        global robot_state
        global floor_information

        cmd = msg.data
        self.get_logger().info("Station needs {0}".format(cmd))

        if cmd == "Camera":
            self.get_logger().info("Call Camera")
            invoice_save()
            time.sleep(1)
            read_floor()
            time.sleep(1)
            floor_info = floor_information.encode()
            self.ser.write(floor_info)
                
        elif cmd == "Robot":
            self.get_logger().info("Call Robot")
            if robot_state == 0:
                callbot_msg = String()
                callbot_msg.data = 'genshin'
                self.publisher_ = self.create_publisher(String, 'callbot', 10)
                self.publisher_.publish(callbot_msg)

                callho_msg = String()
                callho_msg.data = 'impact'
                self.publisher_ = self.create_publisher(String, 'callho', 10)
                self.publisher_.publish(callho_msg)

                robot_state = 1

        else:
            self.get_logger().info("Wrong Command")

    def robot_callback(self, msg):
        global robot_state

        cmd =  msg.data
        if cmd == "ROBOT_LEAVE":
            if robot_state == 1:
                self.get_logger().info("Goodbye Robot")
                robot_state = 0


def main(args=None):
    rclpy.init(args=args)

    arduino_subscriber = ArduinoSubscriber()
    rclpy.spin(arduino_subscriber)
    
    arduino_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


        