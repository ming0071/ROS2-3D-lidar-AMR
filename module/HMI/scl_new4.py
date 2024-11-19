#!/usr/bin/env python

import sys
import subprocess
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import (
    QMessageBox,
    QMainWindow,
    QApplication,
    QGridLayout,
    QHBoxLayout,
    QPushButton,
    QWidget,
    QSplitter,
)
from PyQt5.QtCore import QCoreApplication, Qt

from rviz import bindings as rviz


def is_roscore_running():
    try:
        # 檢查 roscore 是否在運行
        result = subprocess.run(
            ["pgrep", "-f", "roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
        )
        return result.returncode == 0
    except FileNotFoundError:
        # 如果 pgrep 命令不存在，表示這個系統可能不支持 pgrep，則假設 roscore 沒有運行
        return False


def start_roscore():
    if not is_roscore_running():
        # 如果 roscore 沒有運行，則啟動它
        subprocess.Popen(["gnome-terminal", "--", "roscore"])
        print("Started roscore.")
    else:
        print("roscore is already running.")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.external_processes = []
        self.init_ui()

    def init_ui(self):
        self.setObjectName("mainWindow")
        self.resize(1280, 800)
        self.setWindowFlags(self.windowFlags() | Qt.WindowStaysOnTopHint)
        self.setWindowTitle("UI界面 with RViz")

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)

        main_layout = QHBoxLayout(self.central_widget)

        # Create a splitter for RViz and controls
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)

        # Create RViz frame
        self.create_rviz_frame()
        splitter.addWidget(self.frame)

        # Create right side widget for buttons and controls
        right_widget = QWidget()
        right_layout = QGridLayout(right_widget)  # Use QGridLayout for clearer layout
        splitter.addWidget(right_widget)

        # Set the stretch factor to 7 for RViz and 3 for the controls (buttons)
        splitter.setStretchFactor(0, 8)  # Index 0 是 RViz 區域
        splitter.setStretchFactor(1, 2)  # Index 1 是按鈕區域

        # Add lineEdit for 信錦工廠 (on top of RViz)
        self.create_line_edit(right_layout, "lineEdit", "信錦工廠", "large", True)
        right_layout.addWidget(self.lineEdit, 0, 0, 1, 2)  # Span two columns

        # Row 1: Close button (one row)
        button_height = (
            self.get_font_size("large") + 20
        )  # Example height, adjust as needed
        self.create_button(
            right_layout,
            "pushButton_close",
            "關閉視窗",
            "medium",
            "background-color: gray",
            button_height,
        )
        right_layout.addWidget(
            self.pushButton_close, 1, 0, 1, 2
        )  # Spanning two columns

        # Row 2: process stop button (one row)
        self.create_button(
            right_layout,
            "pushButton_4",
            "關閉程序",
            "medium",
            "background-color: red",
            button_height,
        )
        right_layout.addWidget(self.pushButton_4, 2, 0, 1, 2)  # Spanning two columns

        # Row 3: lineEdit_2 (1) and pushButton_5 (啟動機電) on the same row
        self.create_line_edit(
            right_layout, "lineEdit_2", "1", "small", True, button_height
        )
        self.create_button(
            right_layout,
            "pushButton_5",
            "啟動機電",
            "medium",
            "background-color: green",
            button_height,
        )
        right_layout.addWidget(self.lineEdit_2, 3, 0)
        right_layout.addWidget(self.pushButton_5, 3, 1)

        # Row 4: lineEdit_3 (2) and pushButton_6 (啟動定位) on the same row
        self.create_line_edit(
            right_layout, "lineEdit_3", "2", "small", True, button_height
        )
        self.create_button(
            right_layout,
            "pushButton_6",
            "啟動定位",
            "medium",
            "background-color: green",
            button_height,
        )
        right_layout.addWidget(self.lineEdit_3, 4, 0)
        right_layout.addWidget(self.pushButton_6, 4, 1)

        # Row 5: lineEdit_4 (3) on the left, buttons A, B, C in a column on the right
        self.create_line_edit(
            right_layout, "lineEdit_4", "3", "small", True, button_height * 4.4
        )
        self.create_button(
            right_layout, "pushButton", "A站點(檢驗區)", "medium", None, button_height
        )
        self.create_button(
            right_layout, "pushButton_2", "B站點(充電區)", "medium", None, button_height
        )
        self.create_button(
            right_layout, "pushButton_3", "C站點(下貨區)", "medium", None, button_height
        )
        right_layout.addWidget(self.lineEdit_4, 5, 0, 3, 1)
        right_layout.addWidget(self.pushButton, 5, 1, 1, 1)
        right_layout.addWidget(self.pushButton_2, 6, 1, 1, 1)
        right_layout.addWidget(self.pushButton_3, 7, 1, 1, 1)

        # Adjust column stretches to control proportion
        right_layout.setColumnStretch(0, 1)  # Stretch ratio for lineEdit
        right_layout.setColumnStretch(
            1, 5
        )  # Stretch ratio for pushButton, making lineEdit 20% of pushButton

        right_layout.setVerticalSpacing(3)

        # Set the splitter sizes to achieve a 7:3 ratio
        total_width = self.width() - self.central_widget.width()
        rviz_width = total_width * 0.7
        right_width = total_width * 0.3
        splitter.setSizes([rviz_width, right_width])

        self.create_menu_and_status_bar()
        self.connect_buttons()

    def create_rviz_frame(self):
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, "/home/ubt/AMR/src/scl_agv/rviz/slam2.rviz")
        self.frame.load(config)

        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

    def create_button(self, layout, name, text, size, style=None, fixed_height=None):
        button = QPushButton(text)
        button.setObjectName(name)
        font = QtGui.QFont()
        font.setPointSize(self.get_font_size(size))
        button.setFont(font)
        if style:
            button.setStyleSheet(style)
        if fixed_height:
            button.setFixedHeight(fixed_height)
        layout.addWidget(button)
        setattr(self, name, button)

    def create_line_edit(
        self, layout, name, text, size, read_only=False, fixed_height=None
    ):
        line_edit = QtWidgets.QLineEdit(text)
        line_edit.setObjectName(name)
        font = QtGui.QFont()
        font.setPointSize(self.get_font_size(size))
        font.setBold(True)
        line_edit.setFont(font)
        line_edit.setAlignment(QtCore.Qt.AlignCenter)
        line_edit.setReadOnly(read_only)
        if fixed_height:
            line_edit.setFixedHeight(fixed_height)
        layout.addWidget(line_edit)
        setattr(self, name, line_edit)

    def get_font_size(self, size):
        return {"large": 40, "medium": 30, "small": 20}.get(size, 30)

    def create_menu_and_status_bar(self):
        self.menubar = self.menuBar()
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1280, 28))
        self.statusbar = self.statusBar()

    def connect_buttons(self):
        self.pushButton_close.clicked.connect(self.close_application)
        self.pushButton_4.clicked.connect(self.terminate_process)
        self.pushButton_5.clicked.connect(self.launch_3d_localization)
        self.pushButton_6.clicked.connect(self.launch_localization)
        self.pushButton.clicked.connect(self.launch_mission_a)
        self.pushButton_2.clicked.connect(self.launch_mission_b)
        self.pushButton_3.clicked.connect(self.launch_mission_c)

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
        # 終止進程
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

        # 終止所有 ROS 啟動的進程
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
    start_roscore()
    QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling)
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
