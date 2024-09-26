# -*- coding: utf-8 -*-

from __future__ import print_function #python2, pythion3 호환을 위해 사용

import math
import os, sys
import rospy
import time

print(sys.version) #python 버전 출력
import warnings

warnings.filterwarnings('ignore') #경고 메시지 무시

from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import *
from pyqtgraph import PlotWidget, plot
import pyqtgraph as pg

import matplotlib.pyplot as plt

import numpy as np
import math
import utm

form_class = uic.loadUiType('waypoint_playback_ui.ui')[0]
#파일 로드, uic.loadUiType해당 함수가 튜플을 반환하는데 첫번째가 UI 클래스, 두번째가 Qt 베이스 클래스임. 그래서 0번 사용

print(form_class)

GPS_Fix = np.zeros(2)
X = np.zeros(800)
Y = np.zeros(800)
Yaw = np.zeros(800)
waypoint_start_id = 0
waypoint_finish_id = 10
no_line = -1
heading_angle_gps = 0.0
heading_angle_gps_old = 0.0
heading_angle_topic = 0.0

gps1_utm = np.zeros(2)
gps_utm_old = np.zeros(2)
gps2_utm = np.zeros(2)
dual_gps_flag = 0
auto_save_flag = 0
auto_replay_flag = 0
auto_waypints_save_gap_distance = 2
auto_waypints_save_gap_angle = 10

#ros 경로 초기화
def pubish_reset_path():
    pub = rospy.Publisher('/reset_path', Bool, queue_size=1)
    pub.publish(1)


def pubish_waypoint_start_topic():
    pub_run = rospy.Publisher('/wp/waypoint_run_flag', Int8, queue_size=1)
    pub_run.publish(1);


def pubish_waypoint_stop_topic():
    pub_run = rospy.Publisher('/wp/waypoint_run_flag', Int8, queue_size=1)
    pub_run.publish(0);


def pubish_selected_gps_waypoint_topic(data):
    pub_run = rospy.Publisher('/gps/fix_replay', NavSatFix, queue_size=1)

    msg = NavSatFix()
    # msg.header = self.get_header()
    msg.header.frame_id = 'gps'
    msg.latitude = data[0]
    msg.longitude = data[1]
    msg.altitude = 0
    msg.status.status = 2
    # pylint: disable=line-too-long
    msg.status.service = 0

    pub_run.publish(msg);


# class Thread_Auto_Waypoints_Save(QThread):

class Thread_Waypoints_Publish(QThread):
    global auto_replay_flag
    global waypoint_start_id
    global waypoint_finish_id

    def __init__(self, parent):
        super(Thread_Waypoints_Publish, self).__init__(parent)
        self.parent = parent  # self.parent를 사용하여 WindowClass 위젯을 제어할 수 있다.
        print("Thread_Waypoints_Publish")
        print(waypoint_start_id, waypoint_finish_id)

    def run(self):

        data_no = self.parent.waypoint_tableWidget.rowCount();
        print("row count :", data_no)

        for i in range(waypoint_start_id, waypoint_finish_id):
            data1 = self.parent.waypoint_tableWidget.item(i, 0)
            data2 = self.parent.waypoint_tableWidget.item(i, 1)

            utm_east = float(data1.text())
            utm_north = float(data2.text())
            print(i, utm_east, utm_north)
            coordinate = utm.to_latlon(utm_east, utm_north, 52, 'Northern')
            print(coordinate)
            pubish_selected_gps_waypoint_topic(coordinate)
            time.sleep(0.1)
            if auto_replay_flag == 0:
                break

    def stop(self):
        if auto_replay_flag == 0:
            print("Thread_Waypoints_Publish quit")
            self.quit()


class Thread_Auto_Waypoints_Save(QThread):
    global auto_save_flag
    global no_line
    global gps1_utm
    global gps2_utm
    global gps1_utm_old
    global heading_angle_gps
    global heading_angle_gps_old
    global auto_waypints_save_gap_angle
    global auto_waypints_save_gap_distance

    def __init__(self, parent):
        super(Thread_Auto_Waypoints_Save, self).__init__(parent)
        self.parent = parent  # self.parent를 사용하여 WindowClass 위젯을 제어할 수 있다.
        print("Thread_Auto_Waypoints_Save")

    def run(self):

        while auto_save_flag == 1:
            # print("Thread working")
            gps_utm_east = (gps1_utm[0] + gps2_utm[0]) / 2
            gps_utm_north = (gps1_utm[1] + gps2_utm[1]) / 2

            no_line = self.parent.waypoint_tableWidget.rowCount();
            row = no_line

            if (no_line == 0):
                self.parent.waypoint_tableWidget.insertRow(row)  # row 추가
                self.parent.waypoint_tableWidget.setItem(row, 0, QTableWidgetItem(str(gps1_utm[0])))
                self.parent.waypoint_tableWidget.item(row, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                self.parent.waypoint_tableWidget.setItem(row, 1, QTableWidgetItem(str(gps1_utm[1])))
                self.parent.waypoint_tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                self.parent.waypoint_tableWidget.setItem(row, 2, QTableWidgetItem(str(heading_angle_gps)))
                self.parent.waypoint_tableWidget.item(row, 2).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                gps_utm_old[0] = gps_utm_east
                gps_utm_old[1] = gps_utm_north
                heading_angle_gps_old = heading_angle_gps

                self.parent.waypoint_tableWidget.resizeColumnToContents(0)
                self.parent.waypoint_tableWidget.resizeColumnToContents(1)
                self.parent.waypoint_tableWidget.resizeColumnToContents(2)
                self.parent.label_waypoint_no.setText("No. of Waypoints : " + str(no_line + 1))

            distance = math.sqrt(
                (gps_utm_east - gps_utm_old[0]) * (gps_utm_east - gps_utm_old[0]) + (gps_utm_north - gps_utm_old[1]) * (
                            gps_utm_north - gps_utm_old[1]))
            delta_yaw_angle = math.fabs(heading_angle_gps - heading_angle_gps_old)

            # print("yaw_angle : ", heading_angle_gps)

            if (distance >= auto_waypints_save_gap_distance) or (delta_yaw_angle > auto_waypints_save_gap_angle):
                self.parent.waypoint_tableWidget.resizeColumnToContents(0)
                self.parent.waypoint_tableWidget.resizeColumnToContents(1)
                self.parent.waypoint_tableWidget.resizeColumnToContents(2)

                self.parent.waypoint_tableWidget.insertRow(row)  # row 추가
                self.parent.waypoint_tableWidget.setItem(row, 0, QTableWidgetItem(str(gps1_utm[0])))
                self.parent.waypoint_tableWidget.item(row, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                self.parent.waypoint_tableWidget.setItem(row, 1, QTableWidgetItem(str(gps1_utm[1])))
                self.parent.waypoint_tableWidget.item(row, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                self.parent.waypoint_tableWidget.setItem(row, 2, QTableWidgetItem(str(heading_angle_gps)))
                self.parent.waypoint_tableWidget.item(row, 2).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                gps_utm_old[0] = gps_utm_east
                gps_utm_old[1] = gps_utm_north
                heading_angle_gps_old = heading_angle_gps
                print("distance : ", distance)
                print("row : ", row, "no_line", no_line)
                self.parent.lineEdit_Finish_WP_ID.setText(str(no_line + 1))
                self.parent.label_waypoint_no.setText("No. of Waypoints : " + str(no_line + 1))
            # self.parent.update_gps_waypoints()
        # time.sleep(4)

    def stop(self):
        if auto_save_flag == 0:
            print("Thread_Auto_Waypoints_Save quit")
            self.quit()


class WindowClass(QDialog, form_class):

    def __init__(self):

        super(QDialog, self).__init__()

        rospy.init_node('ROS_gps_gui', anonymous=True)

        # GPS Fix_status
        rospy.Subscriber('/gps/fix_status1', Bool, self.subGPS1FixStatusCallback)
        rospy.Subscriber('/gps/fix_status2', Bool, self.subGPS2FixStatusCallback)

        # GPS Fix_data
        rospy.Subscriber('/fix', NavSatFix, self.subGPS1FixDataCallback)
        rospy.Subscriber('/gps1/fix', NavSatFix, self.subGPS1FixDataCallback)
        rospy.Subscriber('/gps2/fix', NavSatFix, self.subGPS2FixDataCallback)

        # rospy.Subscriber('/ublox_gps/fix',  NavSatFix, self.subGPS1FixDataCallback)
        # rospy.Subscriber('/ublox_gps2/fix', NavSatFix, self.subGPS2FixDataCallback)

        # GPS Yaw Angle
        # rospy.Subscriber('/gps/heading_angle', Float32, self.subGPSHeadingAngleCallback)
        rospy.Subscriber('/heading_angle', Float64, self.subGPSHeadingAngleCallback)

        self.setupUi(self)

        self.setWindowTitle("GPS GUI new")

        self.waypoint_tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)  # row 단위로 선택 가능
        self.pushButton_Read_WPs.clicked.connect(self.read_gps_waypoints_data)
        self.waypoint_tableWidget.cellClicked.connect(self.select_tableWidget_event)

        self.pushButton_Plot_WPs.clicked.connect(self.Plot_GPS_Waypoints)

        self.pushButton_RESET_PATH.clicked.connect(self.button_reset_clicked)

        self.lineEdit_Start_WP_ID.textChanged.connect(self.Start_WP_changed)
        self.lineEdit_Finish_WP_ID.textChanged.connect(self.Finish_WP_changed)

        self.pushButton_waypoint_publish.clicked.connect(self.single_waypont_publish)
        self.pushButton_waypoints_replay.clicked.connect(self.wayponts_publish)

        self.pushButton_Plot_WPs.setDisabled(True)

    def wapoint_gap_distance_changed(self):

        temp_data = self.lineEdit_waypoint_gap_distance.text()
        auto_waypints_save_gap_distance = float(temp_data)
        print("auto_waypints_save_gap_distance ", auto_waypints_save_gap_distance)

        return

    def wapoint_gap_angle_changed(self):

        temp_data = self.lineEdit_waypoint_gap_angle.text()
        auto_waypints_save_gap_angle = float(temp_data)
        print("auto_waypints_save_gap_angle ", auto_waypints_save_gap_angle)

        return

    def wayponts_publish(self):
        global auto_replay_flag;
        print("all waypoints publish")

        if auto_replay_flag == 0:
            auto_replay_flag = 1
            print("Auto Save On")
            print("auto_save_flag = 1")
            self.pushButton_waypoints_replay.setText("Auto replay Stop")
            self.thread = Thread_Waypoints_Publish(self)
            self.thread.start()
        else:

            print("Auto Save Stop")
            auto_replay_flag = 0
            print("auto_save_flag = 0")
            self.pushButton_waypoints_replay.setText("waypoints_replay")

        return

    def auto_waypoint_thread_run(self):
        global auto_save_flag;

        if auto_save_flag == 0:
            auto_save_flag = 1
            print("Auto Save On")
            print("auto_save_flag = 1")
            self.pushButton_auto_save_waypoints.setText("Auto Save Stop")
            self.thread = Thread_Auto_Waypoints_Save(self)
            self.thread.start()
        else:

            print("Auto Save Stop")
            auto_save_flag = 0
            print("auto_save_flag = 0")
            self.pushButton_auto_save_waypoints.setText("Auto Save WayPoints")

        return

    def single_waypont_publish(self):
        print("selected waypoint publish")

        row = self.waypoint_tableWidget.currentRow()
        if (row < 0):
            return

        data1 = self.waypoint_tableWidget.item(row, 0)
        data2 = self.waypoint_tableWidget.item(row, 1)

        utm_east = float(data1.text())
        utm_north = float(data2.text())

        print(utm_east, utm_north)

        coordinate = utm.to_latlon(utm_east, utm_north, 52, 'Northern')
        print(coordinate)
        pubish_selected_gps_waypoint_topic(coordinate)
        # pubish_selected_gps_waypoint_topic(data):

        return

    def plot_gps_waypoints_graphWidget(self):

        global X, Y
        no_data = self.waypoint_tableWidget.rowCount();
        print("No of WayPoints Data ", no_data);
        if no_data >= 1:
            X_data = np.zeros(no_data)
            Y_data = np.zeros(no_data)
            for i in range(no_data):
                X_data[i] = (X[i] - X[0])
                Y_data[i] = (Y[i] - Y[0])

            self.graphWidget.setTitle("GPS WayPoints")
            pen = pg.mkPen(color=(255, 0, 0))
            self.graphWidget.clear()
            self.graphWidget.setBackground('w')
            self.graphWidget.plot(X_data, Y_data, pen=pen);

        return

    def read_gps_waypoints_data(self):
        print("read GPS WayPoints")
        global no_line

        self.pushButton_Plot_WPs.setEnabled(True)

        fname = QFileDialog.getOpenFileName(self, 'Open file', './')

        if fname[0]:
            print(fname[0])
            f = open(fname[0], 'r')
            self.label_file_name.setText(fname[0])
            read_lines = f.readlines()
            no_line = len(read_lines)
            read_lines = list(map(lambda s: s.strip(), read_lines))
            self.waypoint_tableWidget.setRowCount(no_line)
            print(read_lines)

            for i in range(no_line):
                print(read_lines[i])

                Data = read_lines[i].split()
                X[i] = float(Data[0])
                Y[i] = float(Data[1])
                Yaw[i] = float(Data[2])

                # coordinate = utm.from_latlon(X[i],Y[i])

                self.waypoint_tableWidget.setItem(i, 0, QTableWidgetItem(Data[0]))
                self.waypoint_tableWidget.item(i, 0).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)
                self.waypoint_tableWidget.setItem(i, 1, QTableWidgetItem(Data[1]))
                self.waypoint_tableWidget.item(i, 1).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)

                self.waypoint_tableWidget.setItem(i, 2, QTableWidgetItem(Data[2]))
                self.waypoint_tableWidget.item(i, 2).setTextAlignment(Qt.AlignCenter | Qt.AlignVCenter)

            self.waypoint_tableWidget.setHorizontalHeaderLabels(["utm_e", "utm_n", "heading_angle"])
            self.waypoint_tableWidget.resizeColumnToContents(0)
            self.waypoint_tableWidget.resizeColumnToContents(1)
            self.waypoint_tableWidget.resizeColumnToContents(2)
            self.plot_gps_waypoints_graphWidget()

            self.lineEdit_Finish_WP_ID.setText(str(no_line))
            f.close()

            self.label_waypoint_no.setText("No. of Waypoints : " + str(no_line))
            return

    def select_tableWidget_event(self):
        print("selected")
        global no_line
        self.waypoint_tableWidget.setSelectionBehavior(QAbstractItemView.SelectRows)  # row 단위로 선택 가능
        row = self.waypoint_tableWidget.currentRow()
        no_line = row
        print(row)
        return

    def update_gps_waypoints(self):
        print("update gps waypoints")

        data_no = self.waypoint_tableWidget.rowCount();
        print("data no ", data_no)

        for i in range(data_no):
            data1 = self.waypoint_tableWidget.item(i, 0)

            if data1 is not None:
                X[i] = float(data1.text())

            data2 = self.waypoint_tableWidget.item(i, 1)
            if data2 is not None:
                Y[i] = float(data2.text())

            print(i, X[i], Y[i])

        self.plot_gps_waypoints_graphWidget()

        return

    def Plot_GPS_Waypoints(self):
        print("plot waypoints")
        data_no = self.waypoint_tableWidget.rowCount();
        print("data no : ", data_no)
        global X, Y

        for i in range(data_no):
            print(i, "  X : Y", X[i], Y[i])

        if data_no >= 1:
            X_data = np.zeros(data_no)
            Y_data = np.zeros(data_no)
            for i in range(data_no):
                X_data[i] = X[i] - X[0]
                Y_data[i] = Y[i] - Y[0]

            plt.scatter(X_data, Y_data)
            plt.title("GPS Waypoints")
            plt.show()

        return

    def button_reset_clicked(self):
        print("Reset Path")
        pubish_reset_path()
        time.sleep(0.2)
        pubish_reset_path()
        return

    def Start_WP_changed(self):
        global waypoint_start_id
        input_id = self.lineEdit_Start_WP_ID.text()
        waypoint_start_id = int(input_id)
        print("wayoint start id ", input_id)
        return

    def Finish_WP_changed(self):
        global waypoint_finish_id
        input_id = self.lineEdit_Finish_WP_ID.text()
        waypoint_finish_id = int(input_id)
        print("wayoint finish id ", waypoint_finish_id)
        return

    def button_Start_WP_clicked(self):
        input_id = self.lineEdit_Start_WP_ID.text()
        waypoint_start_id = int(input_id)
        print("wayoint start id ", input_id)
        pubish_waypoint_start_id(waypoint_start_id)

    def button_Finish_WP_clicked(self):
        input_id = self.lineEdit_Finish_WP_ID.text()
        waypoint_finish_id = int(input_id)
        print("wayoint finish id ", waypoint_finish_id)
        pubish_waypoint_finish_id(waypoint_finish_id)

    def subGPS1FixStatusCallback(self, fix_data):
        if fix_data.data == 1:
            self.label_gps1.setText("GPS 1 :  Fix  ")
            GPS_Fix[0] = 1;
        # print("GSP1 Fix")
        else:
            self.label_gps1.setText("GPS 1 :  Float")
            GPS_Fix[0] = 0;
            print("GSP1 Float")
        return

    def subGPS2FixStatusCallback(self, fix_data):
        if fix_data.data == 1:
            self.label_gps2.setText("GPS 2 :  Fix  ")
            GPS_Fix[1] = 1;
        # print("GSP2 Fix")
        else:
            self.label_gps2.setText("GPS 2 :  Float")
            GPS_Fix[1] = 0;
            print("GSP2 Float")

    def subGPSHeadingAngleCallback(self, y_angle):

        heading_angle_topic = y_angle.data
        # self.label_yaw_angle_1.setText("Yaw Angle : {:5.2f} ".format(heading_angle_topic) )

        print(y_angle.data)
        return

    def subGPS1FixDataCallback(self, gps_fix):
        self.label_gps_data1.setText("Lon : {:9.5f}  Lati : {:9.4f}".format(gps_fix.latitude, gps_fix.longitude))
        coordinate = utm.from_latlon(gps_fix.latitude, gps_fix.longitude)
        gps1_utm[0] = coordinate[0]
        gps1_utm[1] = coordinate[1]
        self.calculate_yaw_angle()
        # print(gps1_utm[0], gps1_utm[1])
        return

    # 이 부분은 검정 해야 함
    def calculate_yaw_angle(self):
        global heading_angle_gps
        global gps1_utm, gps2_utm
        print("Calculate GPS Angle")
        if (GPS_Fix[0] == 1) & (GPS_Fix[1] == 1):
            dis_x = gps1_utm[0] - gps2_utm[0]
            dis_y = gps1_utm[1] - gps2_utm[1]

            angle = -math.atan2(dis_x, dis_y) * 180.0 / math.pi
            # self.label_yaw_angle.setText("Yaw Angle : {:5.2f} ".format(angle))
            self.label_yaw_angle_2.setText("Yaw Angle(sim) : {:5.2f} ".format(angle))
            heading_angle_gps = angle
            print("GPS Heading Angle", angle)
        else:
            dis_x = gps1_utm[0] - gps2_utm[0]
            dis_y = gps1_utm[1] - gps2_utm[1]

            angle = -math.atan2(dis_x, dis_y) * 180.0 / math.pi
            # self.label_yaw_angle.setText("Yaw Angle : {:5.2f} ".format(angle))
            self.label_yaw_angle_2.setText("Yaw Angle(sim) : {:5.2f} ".format(angle))
            heading_angle_gps = angle
            print("GPS Heading Angle", angle)

        return

    def subGPS2FixDataCallback(self, gps_fix):
        self.label_gps_data2.setText("Lon : {:9.5f}  Lati : {:9.4f}".format(gps_fix.latitude, gps_fix.longitude))
        coordinate = utm.from_latlon(gps_fix.latitude, gps_fix.longitude)
        gps2_utm[0] = coordinate[0]
        gps2_utm[1] = coordinate[1]
        return


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindow = WindowClass()
    myWindow.show()
    app.exec_()
