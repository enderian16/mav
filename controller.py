from collections.abc import Callable, Iterable, Mapping
import time
import typing
from typing import Any
import keyboard
from pymavlink import mavutil
from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import QObject, QThread, pyqtSignal
from UI import Ui_MainWindow
import threading


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        # in python3, super(Class, self).xxx = super().xxx
        super(MainWindow, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_control()

    def setup_control(self):
        # TODO
        self.ui.takeoff.clicked.connect(self.takeoff)
        self.ui.link.clicked.connect(self.link)
        self.ui.returnland.clicked.connect(self.returnland)
        self.ui.keephight.clicked.connect(self.keephight)
        self.ui.rtl.clicked.connect(self.rtl)
        self.ui.y1.valueChanged.connect(self.changeY)
        self.ui.x1.valueChanged.connect(self.changeX)
        self.ui.z1.valueChanged.connect(self.changeZ)
           
    def link(self):
        try:
        # 開始一個監聽UDP埠的連線
            the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
            self.the_connection = the_connection 
        # 等待第一個心跳訊息
        # 這會設定鏈路的遠端系統及元件的ID
            the_connection.wait_heartbeat()
        # 向飛機發送解鎖指令
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        # 等待指令確認訊息
            print("等待指令確認訊息")
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,    # confirmation 
            1,    # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,    # GUIDED mode for Copter
            0, 0, 0, 0, 0  # unused parameters
            )
            print("改變模式為 'GUIDED'")
        # 接收COMMAND_ACK訊息並輸出
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)
        #print(self.the_connection)
        except:
            print("錯誤")

    def takeoff(self):
        try:
            # 向飛機再次發送解鎖指令
            #print(self.the_connection)
            the_connection= self.the_connection
            #print(the_connection)
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,  # GUIDED mode
            0, 0, 0, 0, 0  # unused parameters
            )
            print("解鎖")
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)

            time.sleep(1)
            # 向飛機發送起飛指令，目標高度為50
            print("起飛")
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 20)
            msg = self.the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)

            time.sleep(10)
            print("從系統接收到的心跳訊息 (系統 %u 元件 %u)" %
            (the_connection.target_system, the_connection.target_component))
        except:
            print("錯誤")
    def returnland(self):
        try:
            print("降落")
            the_connection= self.the_connection
            #print(the_connection)
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,  # GUIDED mode
            0, 0, 0, 0, 0  # unused parameters
            )
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 2)
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)   
        except:
            print("錯誤")
    def keephight(self):
        try:
            print("高度保持")
            the_connection= self.the_connection
            #print(the_connection)
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            4,  # GUIDED mode
            0, 0, 0, 0, 0  # unused parameters
            )        
            time.sleep(1)
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
            0,
            0, 0, 0, 0, 0, 0
            )
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)
        except:
            print("錯誤")

    def rtl(self):
        try:
            print("返航")
            the_connection= self.the_connection
            #print(the_connection)
            the_connection.mav.command_long_send(
                the_connection.target_system,
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,  # confirmation
                1,  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                4,  # GUIDED mode
                0, 0, 0, 0, 0  # unused parameters
            ) 
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                      mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
            msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            print(msg)
        except:
            print("錯誤")         
    def changeY(self):
        try:
            if self.ui.x1.value() or self.ui.z1.value() == 0:
                self.slider_thread = SliderReaderThread(self.get_slider_valueY)
                self.slider_thread.connectionup(self.the_connection)            
                self.slider_thread.start()
        except:
            print("錯誤Y")
    def changeX(self):
        try:
            if self.ui.z1.value() or self.ui.y1.value() == 0:
                self.slider_thread = SliderReaderThread(self.get_slider_valueY)
                self.slider_thread.connectionup(self.the_connection)            
                self.slider_thread.start()
        except:
            print("錯誤X")   
    def changeZ(self):
        try:
            if self.ui.x1.value() or self.ui.y1.value() == 0:
                self.slider_thread = SliderReaderThread(self.get_slider_valueY)
                self.slider_thread.connectionup(self.the_connection)            
                self.slider_thread.start()
        except:
            print("錯誤Z")
    def get_slider_valueY(self):
        return int(self.ui.x1.value()),int(self.ui.y1.value()),int(self.ui.z1.value())
        
class SliderReaderThread(threading.Thread):
    def __init__(self, get_value_func):
        threading.Thread.__init__(self)
        self.get_value_func = get_value_func

    def run(self):
        x,y,z = 0,0,0
        while True:
            x,y,z = self.get_value_func()
            time.sleep(1)
            self.move(x,y,z)
            if x==y==z==0:
                break
    def connectionup(self,connection):
        self.connection = connection
        #print(self.connection)
        
    def move(self,x,y,z):
        print(self.connection)
        print(x,y,z)
        try:
            the_connection= self.connectionn
            the_connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms (not used)
                    the_connection.target_system,  # target system
                    the_connection.target_component,  # target component
                    mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
                    0b0001111111111000,  # type_mask (only positions enabled)
                    x, y, z,  # x, y, z positions
                    0, 0, 0,  # x, y, z velocity in m/s (not used)
                    0, 0, 0,  # x, y, z acceleration (not used)
                    0, 0  # yaw, yaw_rate (not used)
                )
        except:
            print("錯誤飛行")



'''
class ThreadTask(QThread):
    print('控制飛行')
    def run(self,connection):
        self.connection = connection
        print(connection)
        print(self.connection)
    def startY(self,value):
        while value == 1:
            self.move(0,1,0)
            time.sleep(1)
            if value == 0:
                break
        while value == -1:
            self.move(0,-1,0)
            time.sleep(-1)
            if value ==0:
                break
            #self.move(0,0,0)       
    def move(self,x,y,z):
        #print(self.connection)
        #print(x,y,z)
        try:
            
            the_connection= self.connectionn
            the_connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms (not used)
                    the_connection.target_system,  # target system
                    the_connection.target_component,  # target component
                    mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
                    0b0001111111111000,  # type_mask (only positions enabled)
                    x, y, z,  # x, y, z positions
                    0, 0, 0,  # x, y, z velocity in m/s (not used)
                    0, 0, 0,  # x, y, z acceleration (not used)
                    0, 0  # yaw, yaw_rate (not used)
                )
            print(x,y,z)
            
                #if x and y and z ==0:
                    #break
        except:
            print("錯誤")
'''