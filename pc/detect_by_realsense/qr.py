import qrcode
import cv2
import pyzbar.pyzbar as pyzbar
import mysql.connector
import re
import pyrealsense2 as rs
import numpy as np

mydb = mysql.connector.connect(
    host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
    port = 3306,
    user = "admin",
    password = "12345678",
    database = "delivery"
)

cur = mydb.cursor()
cur.execute("DROP TABLE address")

cur.execute("CREATE TABLE address (address varchar(16), floor varchar(16), name varchar(16), phonenumber varchar(16))")

def extract_floor_from_address(address):
    match = re.search(r'(\d{3,4})호', address)
    if match:
        ho = match.group(1)
        return int(ho[:2]) if len(ho) == 4 else int(ho[0]) if len(ho) == 3 else None
    else:
        return None
    
def invoice_save():
    with open('/home/boma/Desktop/test.txt', 'w') as f:
        f.write('0')

    data_list = []
    used_codes = []
    split_code = []
  #  window_closed = False  # 창을 닫을지 여부를 나타내는 변수

    try:
        f = open("qrbarcode_data.txt", 'r', encoding='utf8')
        data_list = f.readlines()
    except FileNotFoundError:
        pass
    else:
        f.close()

    cap = cv2.VideoCapture(4)

    for i in data_list:
        used_codes.append(i.rsplit('\n'))

    while True:
        success, frame = cap.read()
        with open('/home/boma/Desktop/test.txt', 'r') as f:
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

                    f2 = open('qrbarcode_data.txt', 'a', encoding='utf8')
                    f2.write(my_code+'\n')
                    f2.close()
                    
                 #   window_closed = True  # 창을 닫을 준비
                    break
                else:
                    print('이미 인식된 코드입니다.')
               #     playsound("qrbarcode_beep.mp3")

        key = cv2.waitKey(1)
        if key == 27:  # 'Esc' 키를 누르면 루프를 종료
            break
            
        if number == 3:
            with open('/home/boma/Desktop/test.txt', 'w') as f:
                f.write('0')
            break

    #    if window_closed:  # 창을 닫을 준비가 되면
    #        cv2.destroyAllWindows()  # 창을 닫음
    #        break

    if split_code:
        address = split_code[0]
        name = split_code[1]
        phone_number = split_code[2]
        floor = extract_floor_from_address(address)

        mydb = mysql.connector.connect(
            host = "final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port = 3306,
            user = "admin",
            password = "12345678",
            database = "delivery"
        )

        cur = mydb.cursor()

        cur.execute(f"INSERT INTO address (address, floor, name, phonenumber) VALUES ('{address}', '{floor}', '{name}', '{phone_number}')")

        cur = mydb.cursor(buffered=True)
        cur.execute("SELECT * from address")

        result = cur.fetchall()
        # for result_iterator in result:
            # print(result_iterator)

        mydb.commit()
        mydb.close()

invoice_save()
