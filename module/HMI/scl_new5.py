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
    QLineEdit,
    QWidget,
    QSplitter,
)
from PyQt5.QtCore import QCoreApplication, Qt
from rviz import bindings as rviz

FONT_SIZES = {"large": 40, "medium": 30, "small": 20}
BUTTON_STYLES = {
    "close": "background-color: gray",
    "stop": "background-color: red",
    "start": "background-color: green"
}
RVIZ_CONFIG_PATH = "/home/ubt/AMR/src/scl_agv/rviz/slam2.rviz"
WORKSPACE_SETUP = {
    "機電系統": "/home/ubt/AMR/devel/setup.bash",
    "3D定位": "/home/ubt/hdl_ws/devel/setup.bash",
}
LAUNCH_COMMANDS = {
    "A站點": "/home/ubt/AMR/src/scl_agv/launch/sync_mission1.launch",
    "B站點": "/home/ubt/AMR/src/scl_agv/launch/sync_mission2.launch",
    "C站點": "/home/ubt/AMR/src/scl_agv/launch/sync_mission3.launch",
    "機電系統": "/home/ubt/AMR/src/scl_agv/launch/hdl_3d_localization.launch",
    "3D定位": "/home/ubt/hdl_ws/src/hdl_localization/launch/hdl_localization.launch"
}

# ROSManager 類負責處理 ROS 的啟動、終止等邏輯
class ROSManager:
    def __init__(self):
        self.external_processes = []

    def is_roscore_running(self):
        try:
            result = subprocess.run(
                ["pgrep", "-f", "roscore"], stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            return result.returncode == 0
        except FileNotFoundError:
            return False

    def start_roscore(self):
        if not self.is_roscore_running():
            subprocess.Popen(["gnome-terminal", "--", "roscore"])
            print("Started roscore.")
        else:
            print("roscore is already running.")
            
    def launch_ros(self, workspace_setup, launch_path):
        try:
            command = f"bash -c 'source {workspace_setup} && roslaunch {launch_path}; exec bash'"
            process = subprocess.Popen(command, shell=True)
            self.external_processes.append(process)
            print(f"ROS launch started, PID: {process.pid}")
        except Exception as e:
            print(f"Failed to launch ROS: {str(e)}")
            
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

    def terminate_processes(self):
        for process in self.external_processes:
            try:
                # 優雅地終止進程
                process.terminate()
                process.wait(timeout=3)
                if process.poll() is None:
                    # 如果超時，強制終止進程
                    process.kill()
                    process.wait()
                print(f"程式已終止：PID {process.pid}")
            except subprocess.TimeoutExpired:
                print(f"進程 {process.pid} 終止超時，嘗試強制終止。")
                try:
                    process.kill()
                    process.wait()
                    print(f"強制終止進程：PID {process.pid}")
                except Exception as e:
                    print(f"強制終止進程時發生錯誤：{str(e)}")
            except Exception as e:
                print(f"終止進程時發生錯誤：{str(e)}")
            finally:
                # 清理該進程
                self.external_processes.clear()

        # 確保所有 ROS 相關進程被終止
        try:
            subprocess.run(["pkill", "-f", "roslaunch"], stderr=subprocess.DEVNULL)
            print("所有 ROS 啟動進程已終止")
        except subprocess.CalledProcessError as e:
            print(f"終止 ROS 進程時發生錯誤：{str(e)}")

    


# MainWindow 負責 UI 和 ROSManager 的交互
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ros_manager = ROSManager()  # 使用 ROSManager
        self.ros_manager.start_roscore()
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
        self.create_widget("line_edit",right_layout, "lineEdit", "信錦工廠", "large", True)
        right_layout.addWidget(self.lineEdit, 0, 0, 1, 2)  # Span two columns

        # Row 1: Close button (one row)
        button_height = FONT_SIZES["large"] + 20
        self.create_widget("button",right_layout, "pushButton_close", "關閉視窗", "medium", BUTTON_STYLES["close"], button_height)
        right_layout.addWidget(self.pushButton_close, 1, 0, 1, 2)  # Spanning two columns

        # Row 2: process stop button (one row)
        self.create_widget("button",right_layout, "pushButton_4", "關閉程序", "medium", BUTTON_STYLES["stop"], button_height)
        right_layout.addWidget(self.pushButton_4, 2, 0, 1, 2)  # Spanning two columns

        # Row 3: lineEdit_2 (1) and pushButton_5 (啟動機電) on the same row
        self.create_widget("line_edit",right_layout, "lineEdit_2", "1", "small", True, button_height)
        self.create_widget("button",right_layout, "pushButton_5", "啟動機電", "medium", BUTTON_STYLES["start"], button_height)
        right_layout.addWidget(self.lineEdit_2, 3, 0)
        right_layout.addWidget(self.pushButton_5, 3, 1)

        # Row 4: lineEdit_3 (2) and pushButton_6 (啟動定位) on the same row
        self.create_widget("line_edit",right_layout, "lineEdit_3", "2", "small", True, button_height)
        self.create_widget("button",right_layout, "pushButton_6", "啟動定位", "medium", BUTTON_STYLES["start"], button_height)
        right_layout.addWidget(self.lineEdit_3, 4, 0)
        right_layout.addWidget(self.pushButton_6, 4, 1)

        # Row 5: lineEdit_4 (3) on the left, buttons A, B, C in a column on the right
        self.create_widget("line_edit",right_layout, "lineEdit_4", "3", "small", True, button_height * 4.4)
        self.create_widget("button",right_layout, "pushButton", "A站點(檢驗區)", "medium", None, button_height)
        self.create_widget("button",right_layout, "pushButton_2", "B站點(充電區)", "medium", None, button_height)
        self.create_widget("button",right_layout, "pushButton_3", "C站點(下貨區)", "medium", None, button_height)
        right_layout.addWidget(self.lineEdit_4, 5, 0, 3, 1)
        right_layout.addWidget(self.pushButton, 5, 1, 1, 1)
        right_layout.addWidget(self.pushButton_2, 6, 1, 1, 1)
        right_layout.addWidget(self.pushButton_3, 7, 1, 1, 1)

        # Adjust column stretches to control proportion
        right_layout.setColumnStretch(0, 1)  # Stretch ratio for lineEdit
        right_layout.setColumnStretch(1, 5)  # Stretch ratio for pushButton

        right_layout.setVerticalSpacing(3)

        self.create_menu_and_status_bar()
        self.connect_buttons()

    def create_rviz_frame(self):
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath("")
        self.frame.initialize()

        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile(config, RVIZ_CONFIG_PATH)
        self.frame.load(config)

        self.frame.setMenuBar(None)
        self.frame.setStatusBar(None)
        self.frame.setHideButtonVisibility(False)

        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt(0)

    def create_widget(self, widget_type, layout, name, text, size, style=None, fixed_height=None, read_only=False):
        if widget_type == "button":
            widget = QPushButton(text)
            widget.setFont(QtGui.QFont("", FONT_SIZES.get(size, 30)))
            if style:
                widget.setStyleSheet(style)
            if fixed_height:
                widget.setFixedHeight(fixed_height)
        elif widget_type == "line_edit":
            widget = QLineEdit(text)
            widget.setFont(QtGui.QFont("", FONT_SIZES.get(size, 30)))
            widget.setAlignment(QtCore.Qt.AlignCenter)
            widget.setReadOnly(read_only)
            if fixed_height:
                widget.setFixedHeight(fixed_height)
        layout.addWidget(widget)
        setattr(self, name, widget)


    def create_menu_and_status_bar(self):
        self.menubar = self.menuBar()
        self.menubar.setObjectName("menubar")
        self.setMenuBar(self.menubar)

        self.statusbar = self.statusBar()
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

    def connect_buttons(self):
        self.pushButton_close.clicked.connect(self.close_application)
        self.pushButton_4.clicked.connect(self.ros_manager.terminate_processes)
        self.pushButton_5.clicked.connect(self.launch_3d_localization)
        self.pushButton_6.clicked.connect(self.launch_localization)
        self.pushButton.clicked.connect(lambda: self.launch_mission(LAUNCH_COMMANDS["A站點"], "A站點"))
        self.pushButton_2.clicked.connect(lambda: self.launch_mission(LAUNCH_COMMANDS["B站點"], "B站點"))
        self.pushButton_3.clicked.connect(lambda: self.launch_mission(LAUNCH_COMMANDS["C站點"], "C站點"))
        
    def launch_mission(self, launch_file, station_name):
        self.ros_manager.mission_launch(launch_file, station_name)

    def launch_3d_localization(self):
        workspace_setup = WORKSPACE_SETUP["機電系統"]
        launch_file = LAUNCH_COMMANDS["機電系統"]
        self.ros_manager.launch_ros(workspace_setup, launch_file)

    def launch_localization(self):
        workspace_setup = WORKSPACE_SETUP["3D定位"]
        launch_file = LAUNCH_COMMANDS["3D定位"]
        self.ros_manager.launch_ros(workspace_setup, launch_file)

    def close_application(self):
        self.ros_manager.terminate_processes()
        QCoreApplication.instance().quit()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())