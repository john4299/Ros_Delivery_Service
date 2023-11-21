import sys
import mysql.connector
import webbrowser
import cv2
import os

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtSql import QSqlDatabase, QSqlQuery
from datetime import datetime
from qt_material import apply_stylesheet, QtStyleTools

sys.path.append('')


from_class = uic.loadUiType("/home/boma/realsense/src/depth_example/depth_example/alarm2.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self, parent=None):
        super().__init__()
        self.setupUi(self)


        # MySQL 데이터베이스 연결 설정
        self.db = mysql.connector.connect(
            host="final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port=3306,
            user="admin",
            password="12345678",
            database="delivery"
        )
        self.cursor = self.db.cursor()

    
        
        # 사진 찍어서 저장하기 (image overwrite)
        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        time = 0    

        if not capture.isOpened():
            print("WebCam is not running")
            exit()
            
        while True:
            ret, frame = capture.read()
            time = time + 1
            cv2.imshow("VideoFrame", frame)
            
            cv2.imwrite('/home/computer520/dev_ws/Qt/images/box2.png', frame) 

            break
            
        capture.release()
        cv2.destroyAllWindows()



        self.cursor.execute("SELECT * FROM address")
        data_address = self.cursor.fetchall() 

        # address에서 뒤에서 4글자 따오기
        # picture 테이블에 데이터 추가하기
        for address_row in data_address:
            address_value = address_row[0] 
            last_four_characters = address_value[-4:]

        # table_picture
        self.cursor.execute("SELECT * FROM picture")
        data = self.cursor.fetchall() 
        
        # SQL에 table로 저장되도록 전송하기
        # table 초기화
        self.cursor.execute("DELETE FROM picture")

        home = last_four_characters
        link = '/home/computer520/dev_ws/Qt/images/box2.png'
        info = (home, link)

        cursor = self.db.cursor(buffered=True)
        sql = 'INSERT INTO picture (home, link) VALUES (%s, %s)'
        self.cursor.execute("SET time_zone='Asia/Seoul'")
    
        cursor.execute(sql, tuple(info))
        self.db.commit()

        # 저장된 사진 링크 pyqt에 띄우도록 table 연결하기
        self.cursor.execute("SELECT * FROM picture")
        data = self.cursor.fetchall() 
        self.tablepicture.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch) 
        self.tablepicture.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tablepicture.verticalHeader().setVisible(False)

        # tableWidget에 SQL DATABASE table의 data 추가하기
        self.tablepicture.setRowCount(len(data))
        self.tablepicture.setColumnCount(len(data[0]))

        for row_index, row_data in enumerate(data):
             for col_index, col_data in enumerate(row_data):
                 item = QTableWidgetItem(str(col_data))
                 self.tablepicture.setItem(row_index, col_index, item)

        
        # btn clicked 
        self.btnCheck.clicked.connect(self.showpic)
        self.btnCheck.clicked.connect(self.deliverystate)


        # table-address tablewidget에 나타내기
        self.cursor.execute("SELECT * FROM address")
        data2 = self.cursor.fetchall() 
        
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  
        self.tableWidget.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(False)
        
        # tableWidget에 SQL DATABASE table의 data 추가하기
        self.tableWidget.setRowCount(len(data2))
        self.tableWidget.setColumnCount(len(data2[0]))

        for row_index, row_data in enumerate(data2):
             for col_index, col_data in enumerate(row_data):
                 item = QTableWidgetItem(str(col_data))
                 self.tableWidget.setItem(row_index, col_index, item)
        



    def showpic(self):
        self.pixmap = QPixmap()
        self.pixmap.load('/home/computer520/dev_ws/Qt/images/box2.png')
        self.pixMap.setPixmap(self.pixmap)

            
    def deliverystate(self):
        self.labelstate.setText("Your package arrived")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    myWindows = WindowClass()
    
    myWindows.show()

    sys.exit(app.exec_())
