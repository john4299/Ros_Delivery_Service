import sys
import mysql.connector
import webbrowser
import cv2
import subprocess

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import uic

from PyQt5.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
from PyQt5.QtSql import QSqlDatabase, QSqlQuery
from datetime import datetime
from qt_material import apply_stylesheet, QtStyleTools, list_themes

import rclpy as rp
from rclpy.node import Node
from std_msgs.msg import Empty
import numpy as np
import pyrealsense2 as rs

           
# UI file path
from_class = uic.loadUiType("/home/computer520/ros2_study/src/final/final/alarm2.ui")[0]

class WindowClass(QMainWindow, from_class):
    
    def __init__(self, parent=None):
        super().__init__()
        self.setupUi(self)
        self.set_style()

        # camera capture
        capture = cv2.VideoCapture(4)
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
            
            # save_enter
            if cv2.waitKey(10) == 13:
                cv2.imwrite('/home/computer520/dev_ws/Qt/images/box2.png', frame) 
                break

            elif cv2.waitKey(10) == 27:
                break

            if time == 100:
                time = 0
            
        capture.release()
        cv2.destroyAllWindows() 
    
    
        # MySQL database connect
        self.db = mysql.connector.connect(
            host="final-database.chobs2hlpex2.ap-northeast-2.rds.amazonaws.com",
            port=3306,
            user="admin",
            password="12345678",
            database="delivery"
        )
        self.cursor = self.db.cursor()


        self.cursor.execute("SELECT * FROM address")
        data_address = self.cursor.fetchall() 

        
        # add data to table
        for address_row in data_address:
            address_value = address_row[0] 
            last_four_characters = address_value[-4:]

        # table_picture
        self.cursor.execute("SELECT * FROM picture")
        data = self.cursor.fetchall() 
        
        # init table
        self.cursor.execute("DELETE FROM picture")

        # When delivery is completed, the address of the package must be received from the robot and saved on the table.
        # And need to change the parameter 'home'
        home = last_four_characters 
        link = '/home/computer520/dev_ws/Qt/images/box2.png'
        info = (home, link)

        cursor = self.db.cursor(buffered=True)
        sql = 'INSERT INTO picture (home, link) VALUES (%s, %s)'
        self.cursor.execute("SET time_zone='Asia/Seoul'")
    
        cursor.execute(sql, tuple(info))
        self.db.commit()

        # table_picture connect
        self.cursor.execute("SELECT * FROM picture")
        data = self.cursor.fetchall() 
        self.tablepicture.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch) 
        self.tablepicture.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tablepicture.verticalHeader().setVisible(False)

        # tablewidget SQL
        self.tablepicture.setRowCount(len(data))
        self.tablepicture.setColumnCount(len(data[0]))

        for row_index, row_data in enumerate(data):
             for col_index, col_data in enumerate(row_data):
                 item = QTableWidgetItem(str(col_data))
                 self.tablepicture.setItem(row_index, col_index, item)

        
        # btn clicked 
        self.btnCheck.clicked.connect(self.showpic)
        self.btnCheck.clicked.connect(self.deliverystate)


        # table_address tablewidget
        self.cursor.execute("SELECT * FROM address")
        data2 = self.cursor.fetchall() 
        
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)  
        self.tableWidget.verticalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tableWidget.verticalHeader().setVisible(False)
        
        # tableWidget SQL
        self.tableWidget.setRowCount(len(data2))
        self.tableWidget.setColumnCount(len(data2[0]))

        for row_index, row_data in enumerate(data2):
             for col_index, col_data in enumerate(row_data):
                 item = QTableWidgetItem(str(col_data))
                 self.tableWidget.setItem(row_index, col_index, item)

   


    # PyQt style
    def set_style(self):
        extra = {}
        extra['font_family'] = 'Roboto'
        extra['density_scale'] = str(0)
        theme = 'light_orange.xml'
        invert = True
        apply_stylesheet(QApplication.instance(), theme=theme, extra=extra, invert_secondary=invert)
    

    def showpic(self):
        self.pixmap = QPixmap()
        self.pixmap.load('/home/computer520/dev_ws/Qt/images/box2.png') #image path
        self.pixMap.setPixmap(self.pixmap)

            
    def deliverystate(self):
        self.labelstate.setText("Your package arrived")

    
    


if __name__ == "__main__":
    app = QApplication(sys.argv)
    
    myWindows = WindowClass()
    
    myWindows.show()

    sys.exit(app.exec_())


    
    

   

