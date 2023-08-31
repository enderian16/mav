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
            the_connection = mavutil.mavlink_connection('udpin:127.0.0.1:14550')
            timeout_seconds = 5
            self.the_connection = the_connection
        # 等待第一個心跳訊息
        # 這會設定鏈路的遠端系統及元件的ID    
            the_connection.wait_heartbeat(timeout=timeout_seconds)
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
            self.qthread=data_threading()
            self.qthread.connectiondata(self.the_connection,self.ui)     
            self.qthread.start()
            self.qthread.data_hight.connect(self.get_data_hight) 
        except:
            print("接收錯誤")

    def get_data_hight(self,hight):
        self.ui.hight.setText("目前高度%s公尺"%hight)

    def takeoff(self):
        try:
            the_connection= self.the_connection
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
            #msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
            #print(msg)
            time.sleep(1)
            print("起飛")
            the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 20)
        except:
            print("起飛錯誤")
    def returnland(self):
        try:
            the_connection= self.the_connection
            print("降落")
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
            print("降落錯誤")
    def keephight(self):
        try:
            the_connection= self.the_connection
            print("高度保持")
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
            print("高度保持錯誤")

    def rtl(self):
        try:
            the_connection= self.the_connection
            print("返航")
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
            print("返航錯誤")         
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
    def get_hight(self,hight):
        self.ui.hight.setText(str(hight))

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
            the_connection= self.connection
            the_connection.mav.set_position_target_local_ned_send(
                    0,  # time_boot_ms (not used)
                    the_connection.target_system,  # target system
                    the_connection.target_component,  # target component
                    mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,  # frame
                    0b0001111111111000,  # type_mask (only positions enabled)
                    x, y, -z,  # x, y, z positions
                    0, 0, 0,  # x, y, z velocity in m/s (not used)
                    0, 0, 0,  # x, y, z acceleration (not used)
                    0, 0  # yaw, yaw_rate (not used)
                )
        except:
            print("錯誤飛行")

class data_threading(QObject,threading.Thread):
    data_hight = pyqtSignal(str)
    def __init__(self):
        threading.Thread.__init__(self)
        super(data_threading, self).__init__()

    def connectiondata(self,connection,ui):
        self.connection = connection
        self.ui = ui

    def run(self):
        while True:
            try:    
                the_connection= self.connection
                msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                #print("目前的高度： %s" % msg.relative_alt)
                self.data_hight.emit(str(msg.relative_alt/1000))
                time.sleep(0.1)
            except:
                print("擷取錯誤")
