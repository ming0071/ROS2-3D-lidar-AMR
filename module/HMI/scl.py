#!/usr/bin/env python

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QCoreApplication, Qt
import subprocess
import os


class Ui_mainWindow(object):
    def __init__(self):
        self.external_processes = []  # 使用列表來跟踪所有外部進程

    def setupUi(self, mainWindow):
        mainWindow.setObjectName("mainWindow")
        mainWindow.resize(1280, 800)
        self.centralwidget = QtWidgets.QWidget(mainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(10, 200, 260, 450))
        font = QtGui.QFont()
        font.setPointSize(50)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(290, 200, 260, 450))
        font = QtGui.QFont()
        font.setPointSize(50)
        self.pushButton_2.setFont(font)
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(570, 200, 260, 450))
        font = QtGui.QFont()
        font.setPointSize(50)
        self.pushButton_3.setFont(font)
        self.pushButton_3.setObjectName("pushButton_3")
        self.lineEdit = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit.setGeometry(QtCore.QRect(440, 20, 400, 90))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(239, 239, 239))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        self.lineEdit.setPalette(palette)
        font = QtGui.QFont()
        font.setPointSize(40)
        font.setBold(True)
        font.setWeight(75)
        self.lineEdit.setFont(font)
        self.lineEdit.setMaxLength(32773)
        self.lineEdit.setCursorPosition(4)
        self.lineEdit.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setReadOnly(True)
        self.pushButton_4 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_4.setGeometry(QtCore.QRect(880, 120, 295, 285))
        font = QtGui.QFont()
        font.setPointSize(50)
        self.pushButton_4.setFont(font)
        self.pushButton_4.setObjectName("pushButton_4")
        self.pushButton_4.setStyleSheet("background-color: red")
        self.pushButton_5 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_5.setGeometry(QtCore.QRect(910, 420, 240, 130))
        font = QtGui.QFont()
        font.setPointSize(40)
        self.pushButton_5.setFont(font)
        self.pushButton_5.setObjectName("pushButton_5")
        self.pushButton_5.setStyleSheet("background-color: green")
        self.pushButton_6 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_6.setGeometry(QtCore.QRect(910, 570, 240, 130))
        font = QtGui.QFont()
        font.setPointSize(40)
        self.pushButton_6.setFont(font)
        self.pushButton_6.setObjectName("pushButton_6")
        self.pushButton_6.setStyleSheet("background-color: green")
        self.lineEdit_2 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_2.setGeometry(QtCore.QRect(1180, 450, 65, 70))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(190, 190, 190))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(239, 239, 239))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.PlaceholderText, brush)
        self.lineEdit_2.setPalette(palette)
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.lineEdit_2.setFont(font)
        self.lineEdit_2.setMaxLength(32773)
        self.lineEdit_2.setCursorPosition(1)
        self.lineEdit_2.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_2.setObjectName("lineEdit_2")
        self.lineEdit_2.setReadOnly(True)
        self.lineEdit_3 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_3.setGeometry(QtCore.QRect(1180, 610, 65, 70))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(190, 190, 190))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(239, 239, 239))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.PlaceholderText, brush)
        self.lineEdit_3.setPalette(palette)
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.lineEdit_3.setFont(font)
        self.lineEdit_3.setMaxLength(32773)
        self.lineEdit_3.setCursorPosition(1)
        self.lineEdit_3.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_3.setObjectName("lineEdit_3")
        self.lineEdit_3.setReadOnly(True)
        self.lineEdit_4 = QtWidgets.QLineEdit(self.centralwidget)
        self.lineEdit_4.setGeometry(QtCore.QRect(270, 660, 300, 80))
        palette = QtGui.QPalette()
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Active, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(243, 243, 243))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(204, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Inactive, QtGui.QPalette.PlaceholderText, brush)
        brush = QtGui.QBrush(QtGui.QColor(190, 190, 190))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Text, brush)
        brush = QtGui.QBrush(QtGui.QColor(239, 239, 239))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.Base, brush)
        brush = QtGui.QBrush(QtGui.QColor(0, 0, 0, 128))
        brush.setStyle(QtCore.Qt.SolidPattern)
        palette.setBrush(QtGui.QPalette.Disabled, QtGui.QPalette.PlaceholderText, brush)
        self.lineEdit_4.setPalette(palette)
        font = QtGui.QFont()
        font.setPointSize(35)
        font.setBold(True)
        font.setWeight(75)
        self.lineEdit_4.setFont(font)
        self.lineEdit_4.setMaxLength(32773)
        self.lineEdit_4.setCursorPosition(7)
        self.lineEdit_4.setAlignment(QtCore.Qt.AlignCenter)
        self.lineEdit_4.setObjectName("lineEdit_4")
        self.lineEdit_4.setReadOnly(True)
        mainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(mainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 28))
        self.menubar.setObjectName("menubar")
        mainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(mainWindow)
        self.statusbar.setObjectName("statusbar")
        mainWindow.setStatusBar(self.statusbar)

        self.external_process = None  # 用於存儲外部進程的Popen對象

        self.msg_box = QtWidgets.QMessageBox()  # 定義彈出視窗
        self.msg_box.setStandardButtons(QtWidgets.QMessageBox.Ok)
        #        self.msg_box.accepted.connect(self.on_msg_box_accepted)  # 連接 QMessageBox 的 accepted 信號到自定義的槽函數

        self.retranslateUi(mainWindow)
        QtCore.QMetaObject.connectSlotsByName(mainWindow)

    def retranslateUi(self, mainWindow):
        _translate = QtCore.QCoreApplication.translate
        mainWindow.setWindowTitle(_translate("mainWindow", "UI界面"))
        self.pushButton.setText(_translate("mainWindow", "A站點\n (檢驗區)"))
        self.pushButton_2.setText(_translate("mainWindow", "B站點\n (充電區)"))
        self.pushButton_3.setText(_translate("mainWindow", "C站點\n (下貨區)"))
        self.lineEdit.setText(_translate("mainWindow", "信錦工廠"))
        self.pushButton_4.setText(_translate("mainWindow", "緊急停止"))
        self.pushButton_5.setText(_translate("mainWindow", "啟動機電"))
        self.pushButton_6.setText(_translate("mainWindow", "啟動定位"))
        self.lineEdit_2.setText(_translate("mainWindow", "1"))
        self.lineEdit_3.setText(_translate("mainWindow", "2"))
        self.lineEdit_4.setText(_translate("mainWindow", "3  選擇任務"))

    def startLaunch(self):
        try:
            launch_path = "/home/ubt/AMR/src/scl_agv/launch/sync_mission1.launch"
            self.external_process = subprocess.Popen(["roslaunch", launch_path])
            self.external_processes.append(
                self.external_process
            )  # 將新的外部進程加入列表
            print("前往A站點，正在執行，PID：", self.external_process.pid)

            # 等待進程完成並捕獲輸出
            self.external_process.wait()
            print("前往A站點任務完成")

            # 彈出確認視窗
            self.msg_box.setWindowTitle("到達指定站點")
            self.msg_box.setText("已經完成上貨！請選擇下一個站點")
            self.msg_box.exec_()

            # 顯示確認訊息框
        #            self.msg_box.setText("已經完成上貨！")
        #           self.msg_box.exec_()

        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    def startLaunch2(self):
        try:
            launch_path = "/home/ubt/AMR/src/scl_agv/launch/sync_mission2.launch"
            self.external_process = subprocess.Popen(["roslaunch", launch_path])
            self.external_processes.append(
                self.external_process
            )  # 將新的外部進程加入列表
            print("前往B站點，正在執行，PID：", self.external_process.pid)

            # 等待進程完成並捕獲輸出
            self.external_process.wait()
            print("前往B站點任務完成")

            # 彈出確認視窗
            self.msg_box.setWindowTitle("到達指定站點")
            self.msg_box.setText("已經完成上貨！請選擇下一個站點")
            self.msg_box.exec_()

        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    def startLaunch3(self):
        try:
            launch_path = "/home/ubt/AMR/src/scl_agv/launch/sync_mission3.launch"
            self.external_process = subprocess.Popen(["roslaunch", launch_path])
            self.external_processes.append(
                self.external_process
            )  # 將新的外部進程加入列表
            print("前往C站點，正在執行，PID：", self.external_process.pid)

            # 等待進程完成並捕獲輸出
            self.external_process.wait()
            print("前往C站點任務完成")

            # 彈出確認視窗
            self.msg_box.setWindowTitle("到達指定站點")
            self.msg_box.setText("已經完成上貨！請選擇下一個站點")
            self.msg_box.exec_()

        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    def startLaunch5(self):
        try:
            launch_path = "/home/ubt/AMR/src/scl_agv/launch/hdl_3d_localization.launch"
            self.external_process = subprocess.Popen(["roslaunch", launch_path])
            self.external_processes.append(
                self.external_process
            )  # 將新的外部進程加入列表
            print("啟動機電的按鈕，正在執行，PID：", self.external_process.pid)

        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    def startLaunch6(self):
        try:
            launch_path = (
                "/home/ubt/hdl_ws/src/hdl_localization/launch/hdl_localization.launch"
            )
            self.external_process = subprocess.Popen(["roslaunch", launch_path])
            self.external_processes.append(
                self.external_process
            )  # 將新的外部進程加入列表
            print("啟動定位的按鈕，正在執行，PID：", self.external_process.pid)

        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    #    def on_msg_box_accepted(self):
    #       try:
    #          launch_path = "/home/ubt/AMR/src/scl_agv/launch/sync_goback.launch"
    #         self.external_process = subprocess.Popen(["roslaunch", launch_path])
    #        print("執行回原點的任務，正在執行，PID：", self.external_process.pid)
    #   except Exception as e:
    #      print(f"發生錯誤：{str(e)}")

    def terminateProcess(self):
        try:
            for process in self.external_processes:
                process.terminate()  # 逐個終止列表中的外部進程
                print("終止程式")

                process.wait()
                print("程式已緊急停止")

            # 清空外部進程列表
            self.external_processes = []

        except Exception as e:
            print(f"發生錯誤：{str(e)}")


if __name__ == "__main__":
    import sys

    QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = QtWidgets.QMainWindow()
    ui = Ui_mainWindow()
    ui.setupUi(mainWindow)

    ui.pushButton.clicked.connect(ui.startLaunch)
    ui.pushButton_2.clicked.connect(ui.startLaunch2)
    ui.pushButton_3.clicked.connect(ui.startLaunch3)
    ui.pushButton_4.clicked.connect(ui.terminateProcess)
    ui.pushButton_5.clicked.connect(ui.startLaunch5)
    ui.pushButton_6.clicked.connect(ui.startLaunch6)

    mainWindow.show()
    sys.exit(app.exec_())
