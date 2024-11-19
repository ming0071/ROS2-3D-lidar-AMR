#!/usr/bin/env python

import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMessageBox, QMainWindow, QApplication
from PyQt5.QtCore import QCoreApplication, Qt


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.external_processes = []
        self.init_ui()

    def init_ui(self):
        self.setObjectName("mainWindow")
        self.resize(1280, 800)
        self.setWindowTitle("UI界面")

        self.central_widget = QtWidgets.QWidget(self)
        self.setCentralWidget(self.central_widget)

        self.create_buttons()
        self.create_line_edits()
        self.create_menu_and_status_bar()

        self.connect_buttons()

    def create_buttons(self):
        buttons = [
            ("pushButton", "A站點\n(檢驗區)", 10, 200, 260, 450, None, "large"),
            ("pushButton_2", "B站點\n(充電區)", 290, 200, 260, 450, None, "large"),
            ("pushButton_3", "C站點\n(下貨區)", 570, 200, 260, 450, None, "large"),
            (
                "pushButton_4",
                "緊急停止",
                880,
                120,
                295,
                285,
                "background-color: red",
                "medium",
            ),
            (
                "pushButton_5",
                "啟動機電",
                910,
                420,
                240,
                130,
                "background-color: green",
                "medium",
            ),
            (
                "pushButton_6",
                "啟動定位",
                910,
                570,
                240,
                130,
                "background-color: green",
                "medium",
            ),
            (
                "pushButton_close",
                "關閉視窗",
                1120,
                10,
                150,
                50,
                "background-color: gray",
                "small",
            ),
        ]

        for name, text, x, y, w, h, style, size in buttons:
            self.create_button(name, text, x, y, w, h, style, size)

    def create_button(self, name, text, x, y, width, height, style=None, size="medium"):
        button = QtWidgets.QPushButton(self.central_widget)
        button.setGeometry(QtCore.QRect(x, y, width, height))
        button.setObjectName(name)
        button.setText(text)
        font = QtGui.QFont()
        font.setPointSize(self.get_font_size(size))
        button.setFont(font)
        if style:
            button.setStyleSheet(style)
        setattr(self, name, button)

    def create_line_edits(self):
        line_edits = [
            ("lineEdit", "信錦工廠", 440, 20, 400, 90, "medium", True),
            ("lineEdit_2", "1", 1180, 450, 65, 70, "small", True),
            ("lineEdit_3", "2", 1180, 610, 65, 70, "small", True),
            ("lineEdit_4", "3  選擇任務", 270, 660, 300, 80, "medium", True),
        ]

        for name, text, x, y, w, h, size, read_only in line_edits:
            self.create_line_edit(name, text, x, y, w, h, size, read_only)

    def create_line_edit(self, name, text, x, y, width, height, size, read_only=False):
        line_edit = QtWidgets.QLineEdit(self.central_widget)
        line_edit.setGeometry(QtCore.QRect(x, y, width, height))
        line_edit.setObjectName(name)
        line_edit.setText(text)
        font = QtGui.QFont()
        font.setPointSize(self.get_font_size(size))
        font.setBold(True)
        line_edit.setFont(font)
        line_edit.setAlignment(QtCore.Qt.AlignCenter)
        line_edit.setReadOnly(read_only)
        setattr(self, name, line_edit)

    def get_font_size(self, size):
        return {"large": 50, "medium": 40, "small": 25}.get(size, 40)

    def create_menu_and_status_bar(self):
        self.menubar = self.menuBar()
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 28))
        self.statusbar = self.statusBar()

    def connect_buttons(self):
        self.pushButton.clicked.connect(self.launch_mission_a)
        self.pushButton_2.clicked.connect(self.launch_mission_b)
        self.pushButton_3.clicked.connect(self.launch_mission_c)
        self.pushButton_4.clicked.connect(self.terminate_process)
        self.pushButton_5.clicked.connect(self.launch_3d_localization)
        self.pushButton_6.clicked.connect(self.launch_localization)
        self.pushButton_close.clicked.connect(self.close_application)

    def launch_mission_a(self):
        self.mission_launch(
            "/home/ubt/AMR/src/scl_agv/launch/sync_mission1.launch", "A站點"
        )

    def launch_mission_b(self):
        self.mission_launch(
            "/home/ubt/AMR/src/scl_agv/launch/sync_mission2.launch", "B站點"
        )

    def launch_mission_c(self):
        self.mission_launch(
            "/home/ubt/AMR/src/scl_agv/launch/sync_mission3.launch", "C站點"
        )

    def launch_3d_localization(self):
        workspace_setup = "/home/ubt/AMR/devel/setup.bash"
        launch_file = "/home/ubt/AMR/src/scl_agv/launch/hdl_3d_localization.launch"
        self.start_launch(workspace_setup, launch_file, "啟動機電")

    def launch_localization(self):
        workspace_setup = "/home/ubt/hdl_ws/devel/setup.bash"
        launch_file = (
            "/home/ubt/hdl_ws/src/hdl_localization/launch/hdl_localization.launch"
        )
        self.start_launch(workspace_setup, launch_file, "啟動定位")

    def start_launch(self, workspace_setup, launch_path, station_name):
        try:
            command = f"bash -c 'source {workspace_setup} && roslaunch {launch_path}; exec bash'"
            process = subprocess.Popen(command, shell=True)
            self.external_processes.append(process)
            print(f"前往{station_name}，正在執行，PID：{process.pid}")

        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    def mission_launch(self, launch_path, station_name):
        try:
            process = subprocess.Popen(["roslaunch", launch_path])
            self.external_processes.append(process)
            print(f"前往{station_name}，正在執行，PID：{process.pid}")

            process.wait()
            print(f"前往{station_name}任務完成")

            self.show_message_box("到達指定站點", "已經完成上貨！請選擇下一個站點")
        except Exception as e:
            print(f"發生錯誤：{str(e)}")

    def show_message_box(self, title, message):
        QMessageBox.information(self, title, message)

    def terminate_process(self):
        for process in self.external_processes:
            try:
                process.terminate()
                process.wait(timeout=3)
                if process.poll() is None:
                    process.kill()
                    process.wait()
                print(f"程式已終止：PID {process.pid}")
            except Exception as e:
                print(f"終止程式時發生錯誤：{str(e)}")

        self.external_processes.clear()
        if self.statusbar is not None:
            try:
                subprocess.run(["pkill", "-f", "roslaunch"], stderr=subprocess.DEVNULL)
                print("所有 ROS 啟動進程已終止")
            except subprocess.CalledProcessError as e:
                print(f"終止 ROS 進程時發生錯誤：{str(e)}")

    def close_application(self):
        self.terminate_process()
        QCoreApplication.instance().quit()


if __name__ == "__main__":
    QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
