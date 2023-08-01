# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'test.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(815, 594)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.takeoff = QtWidgets.QPushButton(self.centralwidget)
        self.takeoff.setGeometry(QtCore.QRect(30, 400, 75, 23))
        self.takeoff.setObjectName("takeoff")
        self.returnland = QtWidgets.QPushButton(self.centralwidget)
        self.returnland.setGeometry(QtCore.QRect(30, 440, 75, 23))
        self.returnland.setObjectName("returnland")
        self.rtl = QtWidgets.QPushButton(self.centralwidget)
        self.rtl.setGeometry(QtCore.QRect(30, 480, 75, 23))
        self.rtl.setObjectName("rtl")
        self.keephight = QtWidgets.QPushButton(self.centralwidget)
        self.keephight.setGeometry(QtCore.QRect(30, 520, 75, 23))
        self.keephight.setObjectName("keephight")
        self.hight = QtWidgets.QTextEdit(self.centralwidget)
        self.hight.setGeometry(QtCore.QRect(693, 500, 91, 41))
        self.hight.setObjectName("hight")
        self.y1 = QtWidgets.QSlider(self.centralwidget)
        self.y1.setGeometry(QtCore.QRect(240, 480, 121, 21))
        self.y1.setMinimum(-1)
        self.y1.setMaximum(1)
        self.y1.setOrientation(QtCore.Qt.Horizontal)
        self.y1.setObjectName("y1")
        self.z1 = QtWidgets.QSlider(self.centralwidget)
        self.z1.setGeometry(QtCore.QRect(730, 369, 22, 101))
        self.z1.setMinimum(-1)
        self.z1.setMaximum(1)
        self.z1.setOrientation(QtCore.Qt.Vertical)
        self.z1.setObjectName("z1")
        self.y2 = QtWidgets.QLabel(self.centralwidget)
        self.y2.setGeometry(QtCore.QRect(280, 510, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.y2.setFont(font)
        self.y2.setObjectName("y2")
        self.x2 = QtWidgets.QLabel(self.centralwidget)
        self.x2.setGeometry(QtCore.QRect(480, 510, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.x2.setFont(font)
        self.x2.setObjectName("x2")
        self.z2 = QtWidgets.QLabel(self.centralwidget)
        self.z2.setGeometry(QtCore.QRect(720, 330, 41, 31))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.z2.setFont(font)
        self.z2.setObjectName("z2")
        self.link = QtWidgets.QPushButton(self.centralwidget)
        self.link.setGeometry(QtCore.QRect(20, 10, 81, 41))
        font = QtGui.QFont()
        font.setPointSize(16)
        self.link.setFont(font)
        self.link.setObjectName("link")
        self.x1 = QtWidgets.QSlider(self.centralwidget)
        self.x1.setGeometry(QtCore.QRect(490, 409, 22, 91))
        self.x1.setMinimum(-1)
        self.x1.setMaximum(1)
        self.x1.setOrientation(QtCore.Qt.Vertical)
        self.x1.setObjectName("x1")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 815, 21))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.takeoff.setText(_translate("MainWindow", "起飛"))
        self.returnland.setText(_translate("MainWindow", "降落"))
        self.rtl.setText(_translate("MainWindow", "返航"))
        self.keephight.setText(_translate("MainWindow", "高度保持"))
        self.y2.setText(_translate("MainWindow", "左右"))
        self.x2.setText(_translate("MainWindow", "前後"))
        self.z2.setText(_translate("MainWindow", "高度"))
        self.link.setText(_translate("MainWindow", "連線"))