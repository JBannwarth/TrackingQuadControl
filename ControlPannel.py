# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ControlPannel.ui'
#
# Created: Wed May 13 14:42:15 2015
#      by: PyQt4 UI code generator 4.11.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_ControlPannel(object):
    def setupUi(self, ControlPannel):
        ControlPannel.setObjectName(_fromUtf8("ControlPannel"))
        ControlPannel.resize(437, 488)
        ControlPannel.setMinimumSize(QtCore.QSize(410, 390))
        self.centralwidget = QtGui.QWidget(ControlPannel)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setEnabled(True)
        self.tabWidget.setGeometry(QtCore.QRect(10, 10, 411, 421))
        self.tabWidget.setFocusPolicy(QtCore.Qt.NoFocus)
        self.tabWidget.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tab = QtGui.QWidget()
        self.tab.setObjectName(_fromUtf8("tab"))
        self.set_land_position_button = QtGui.QPushButton(self.tab)
        self.set_land_position_button.setGeometry(QtCore.QRect(220, 290, 161, 27))
        self.set_land_position_button.setObjectName(_fromUtf8("set_land_position_button"))
        self.groupBox_4 = QtGui.QGroupBox(self.tab)
        self.groupBox_4.setGeometry(QtCore.QRect(220, 160, 171, 121))
        self.groupBox_4.setObjectName(_fromUtf8("groupBox_4"))
        self.layoutWidget = QtGui.QWidget(self.groupBox_4)
        self.layoutWidget.setGeometry(QtCore.QRect(0, 20, 161, 97))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout_2.setMargin(0)
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.verticalLayout_6 = QtGui.QVBoxLayout()
        self.verticalLayout_6.setObjectName(_fromUtf8("verticalLayout_6"))
        self.label_9 = QtGui.QLabel(self.layoutWidget)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.verticalLayout_6.addWidget(self.label_9)
        self.label_10 = QtGui.QLabel(self.layoutWidget)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.verticalLayout_6.addWidget(self.label_10)
        self.label_11 = QtGui.QLabel(self.layoutWidget)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.verticalLayout_6.addWidget(self.label_11)
        self.horizontalLayout_2.addLayout(self.verticalLayout_6)
        self.verticalLayout_5 = QtGui.QVBoxLayout()
        self.verticalLayout_5.setObjectName(_fromUtf8("verticalLayout_5"))
        self.imu_roll_box = QtGui.QLineEdit(self.layoutWidget)
        self.imu_roll_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.imu_roll_box.setReadOnly(True)
        self.imu_roll_box.setObjectName(_fromUtf8("imu_roll_box"))
        self.verticalLayout_5.addWidget(self.imu_roll_box)
        self.imu_pitch_box = QtGui.QLineEdit(self.layoutWidget)
        self.imu_pitch_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.imu_pitch_box.setReadOnly(True)
        self.imu_pitch_box.setObjectName(_fromUtf8("imu_pitch_box"))
        self.verticalLayout_5.addWidget(self.imu_pitch_box)
        self.imu_yaw_box = QtGui.QLineEdit(self.layoutWidget)
        self.imu_yaw_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.imu_yaw_box.setReadOnly(True)
        self.imu_yaw_box.setObjectName(_fromUtf8("imu_yaw_box"))
        self.verticalLayout_5.addWidget(self.imu_yaw_box)
        self.horizontalLayout_2.addLayout(self.verticalLayout_5)
        self.control_toggle_button = QtGui.QPushButton(self.tab)
        self.control_toggle_button.setGeometry(QtCore.QRect(220, 320, 161, 27))
        self.control_toggle_button.setObjectName(_fromUtf8("control_toggle_button"))
        self.groupBox_3 = QtGui.QGroupBox(self.tab)
        self.groupBox_3.setGeometry(QtCore.QRect(0, 0, 421, 161))
        self.groupBox_3.setObjectName(_fromUtf8("groupBox_3"))
        self.layoutWidget1 = QtGui.QWidget(self.groupBox_3)
        self.layoutWidget1.setGeometry(QtCore.QRect(1, 21, 390, 97))
        self.layoutWidget1.setObjectName(_fromUtf8("layoutWidget1"))
        self.horizontalLayout = QtGui.QHBoxLayout(self.layoutWidget1)
        self.horizontalLayout.setMargin(0)
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.verticalLayout_3 = QtGui.QVBoxLayout()
        self.verticalLayout_3.setObjectName(_fromUtf8("verticalLayout_3"))
        self.label_2 = QtGui.QLabel(self.layoutWidget1)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.verticalLayout_3.addWidget(self.label_2)
        self.label_3 = QtGui.QLabel(self.layoutWidget1)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.verticalLayout_3.addWidget(self.label_3)
        self.label_4 = QtGui.QLabel(self.layoutWidget1)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.verticalLayout_3.addWidget(self.label_4)
        self.horizontalLayout.addLayout(self.verticalLayout_3)
        self.verticalLayout = QtGui.QVBoxLayout()
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.tracking_x_position_box = QtGui.QLineEdit(self.layoutWidget1)
        self.tracking_x_position_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.tracking_x_position_box.setReadOnly(True)
        self.tracking_x_position_box.setObjectName(_fromUtf8("tracking_x_position_box"))
        self.verticalLayout.addWidget(self.tracking_x_position_box)
        self.tracking_y_position_box = QtGui.QLineEdit(self.layoutWidget1)
        self.tracking_y_position_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.tracking_y_position_box.setReadOnly(True)
        self.tracking_y_position_box.setObjectName(_fromUtf8("tracking_y_position_box"))
        self.verticalLayout.addWidget(self.tracking_y_position_box)
        self.tracking_z_position_box = QtGui.QLineEdit(self.layoutWidget1)
        self.tracking_z_position_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.tracking_z_position_box.setReadOnly(True)
        self.tracking_z_position_box.setObjectName(_fromUtf8("tracking_z_position_box"))
        self.verticalLayout.addWidget(self.tracking_z_position_box)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.verticalLayout_4 = QtGui.QVBoxLayout()
        self.verticalLayout_4.setObjectName(_fromUtf8("verticalLayout_4"))
        self.label_5 = QtGui.QLabel(self.layoutWidget1)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.verticalLayout_4.addWidget(self.label_5)
        self.label_6 = QtGui.QLabel(self.layoutWidget1)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.verticalLayout_4.addWidget(self.label_6)
        self.label_7 = QtGui.QLabel(self.layoutWidget1)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.verticalLayout_4.addWidget(self.label_7)
        self.horizontalLayout.addLayout(self.verticalLayout_4)
        self.verticalLayout_2 = QtGui.QVBoxLayout()
        self.verticalLayout_2.setObjectName(_fromUtf8("verticalLayout_2"))
        self.tracking_roll_box = QtGui.QLineEdit(self.layoutWidget1)
        self.tracking_roll_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.tracking_roll_box.setReadOnly(True)
        self.tracking_roll_box.setObjectName(_fromUtf8("tracking_roll_box"))
        self.verticalLayout_2.addWidget(self.tracking_roll_box)
        self.tracking_pitch_box = QtGui.QLineEdit(self.layoutWidget1)
        self.tracking_pitch_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.tracking_pitch_box.setReadOnly(True)
        self.tracking_pitch_box.setObjectName(_fromUtf8("tracking_pitch_box"))
        self.verticalLayout_2.addWidget(self.tracking_pitch_box)
        self.tracking_yaw_box = QtGui.QLineEdit(self.layoutWidget1)
        self.tracking_yaw_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.tracking_yaw_box.setReadOnly(True)
        self.tracking_yaw_box.setObjectName(_fromUtf8("tracking_yaw_box"))
        self.verticalLayout_2.addWidget(self.tracking_yaw_box)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.set_home_position_button = QtGui.QPushButton(self.groupBox_3)
        self.set_home_position_button.setGeometry(QtCore.QRect(0, 120, 211, 27))
        self.set_home_position_button.setObjectName(_fromUtf8("set_home_position_button"))
        self.set_home_orientation_button = QtGui.QPushButton(self.groupBox_3)
        self.set_home_orientation_button.setGeometry(QtCore.QRect(220, 120, 171, 27))
        self.set_home_orientation_button.setObjectName(_fromUtf8("set_home_orientation_button"))
        self.groupBox_2 = QtGui.QGroupBox(self.tab)
        self.groupBox_2.setGeometry(QtCore.QRect(0, 160, 221, 201))
        self.groupBox_2.setFlat(False)
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.goto_position_button = QtGui.QPushButton(self.groupBox_2)
        self.goto_position_button.setGeometry(QtCore.QRect(0, 160, 211, 27))
        self.goto_position_button.setObjectName(_fromUtf8("goto_position_button"))
        self.layoutWidget2 = QtGui.QWidget(self.groupBox_2)
        self.layoutWidget2.setGeometry(QtCore.QRect(0, 20, 211, 131))
        self.layoutWidget2.setObjectName(_fromUtf8("layoutWidget2"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.layoutWidget2)
        self.horizontalLayout_3.setMargin(0)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.verticalLayout_8 = QtGui.QVBoxLayout()
        self.verticalLayout_8.setObjectName(_fromUtf8("verticalLayout_8"))
        self.label_8 = QtGui.QLabel(self.layoutWidget2)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.verticalLayout_8.addWidget(self.label_8)
        self.label_12 = QtGui.QLabel(self.layoutWidget2)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.verticalLayout_8.addWidget(self.label_12)
        self.label_13 = QtGui.QLabel(self.layoutWidget2)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.verticalLayout_8.addWidget(self.label_13)
        self.label_15 = QtGui.QLabel(self.layoutWidget2)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.verticalLayout_8.addWidget(self.label_15)
        self.horizontalLayout_3.addLayout(self.verticalLayout_8)
        self.verticalLayout_7 = QtGui.QVBoxLayout()
        self.verticalLayout_7.setObjectName(_fromUtf8("verticalLayout_7"))
        self.goto_x_position_box = QtGui.QLineEdit(self.layoutWidget2)
        self.goto_x_position_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.goto_x_position_box.setReadOnly(False)
        self.goto_x_position_box.setObjectName(_fromUtf8("goto_x_position_box"))
        self.verticalLayout_7.addWidget(self.goto_x_position_box)
        self.goto_y_position_box = QtGui.QLineEdit(self.layoutWidget2)
        self.goto_y_position_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.goto_y_position_box.setReadOnly(False)
        self.goto_y_position_box.setObjectName(_fromUtf8("goto_y_position_box"))
        self.verticalLayout_7.addWidget(self.goto_y_position_box)
        self.goto_z_position_box = QtGui.QLineEdit(self.layoutWidget2)
        self.goto_z_position_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.goto_z_position_box.setReadOnly(False)
        self.goto_z_position_box.setObjectName(_fromUtf8("goto_z_position_box"))
        self.verticalLayout_7.addWidget(self.goto_z_position_box)
        self.goto_yaw_box = QtGui.QLineEdit(self.layoutWidget2)
        self.goto_yaw_box.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.goto_yaw_box.setReadOnly(False)
        self.goto_yaw_box.setObjectName(_fromUtf8("goto_yaw_box"))
        self.verticalLayout_7.addWidget(self.goto_yaw_box)
        self.horizontalLayout_3.addLayout(self.verticalLayout_7)
        self.take_off_land_button = QtGui.QPushButton(self.tab)
        self.take_off_land_button.setGeometry(QtCore.QRect(220, 350, 161, 27))
        self.take_off_land_button.setObjectName(_fromUtf8("take_off_land_button"))
        self.tabWidget.addTab(self.tab, _fromUtf8(""))
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName(_fromUtf8("tab_2"))
        self.groupBox = QtGui.QGroupBox(self.tab_2)
        self.groupBox.setGeometry(QtCore.QRect(20, 10, 371, 321))
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.layoutWidget3 = QtGui.QWidget(self.groupBox)
        self.layoutWidget3.setGeometry(QtCore.QRect(10, 30, 304, 130))
        self.layoutWidget3.setObjectName(_fromUtf8("layoutWidget3"))
        self.horizontalLayout_5 = QtGui.QHBoxLayout(self.layoutWidget3)
        self.horizontalLayout_5.setMargin(0)
        self.horizontalLayout_5.setObjectName(_fromUtf8("horizontalLayout_5"))
        self.verticalLayout_18 = QtGui.QVBoxLayout()
        self.verticalLayout_18.setObjectName(_fromUtf8("verticalLayout_18"))
        self.ch_box_1 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_1.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_1.setReadOnly(True)
        self.ch_box_1.setObjectName(_fromUtf8("ch_box_1"))
        self.verticalLayout_18.addWidget(self.ch_box_1)
        self.ch_box_2 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_2.setReadOnly(True)
        self.ch_box_2.setObjectName(_fromUtf8("ch_box_2"))
        self.verticalLayout_18.addWidget(self.ch_box_2)
        self.ch_box_3 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_3.setReadOnly(True)
        self.ch_box_3.setObjectName(_fromUtf8("ch_box_3"))
        self.verticalLayout_18.addWidget(self.ch_box_3)
        self.ch_box_4 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_4.setReadOnly(True)
        self.ch_box_4.setObjectName(_fromUtf8("ch_box_4"))
        self.verticalLayout_18.addWidget(self.ch_box_4)
        self.horizontalLayout_5.addLayout(self.verticalLayout_18)
        self.verticalLayout_13 = QtGui.QVBoxLayout()
        self.verticalLayout_13.setObjectName(_fromUtf8("verticalLayout_13"))
        self.ch_box_5 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_5.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_5.setReadOnly(True)
        self.ch_box_5.setObjectName(_fromUtf8("ch_box_5"))
        self.verticalLayout_13.addWidget(self.ch_box_5)
        self.ch_box_6 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_6.setReadOnly(True)
        self.ch_box_6.setObjectName(_fromUtf8("ch_box_6"))
        self.verticalLayout_13.addWidget(self.ch_box_6)
        self.ch_box_7 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_7.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_7.setReadOnly(True)
        self.ch_box_7.setObjectName(_fromUtf8("ch_box_7"))
        self.verticalLayout_13.addWidget(self.ch_box_7)
        self.ch_box_8 = QtGui.QLineEdit(self.layoutWidget3)
        self.ch_box_8.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.ch_box_8.setReadOnly(True)
        self.ch_box_8.setObjectName(_fromUtf8("ch_box_8"))
        self.verticalLayout_13.addWidget(self.ch_box_8)
        self.horizontalLayout_5.addLayout(self.verticalLayout_13)
        self.tabWidget.addTab(self.tab_2, _fromUtf8(""))
        self.tab_3 = QtGui.QWidget()
        self.tab_3.setObjectName(_fromUtf8("tab_3"))
        self.box_1 = QtGui.QLineEdit(self.tab_3)
        self.box_1.setGeometry(QtCore.QRect(40, 20, 71, 27))
        self.box_1.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_1.setReadOnly(True)
        self.box_1.setObjectName(_fromUtf8("box_1"))
        self.box_2 = QtGui.QLineEdit(self.tab_3)
        self.box_2.setGeometry(QtCore.QRect(40, 50, 71, 27))
        self.box_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_2.setReadOnly(True)
        self.box_2.setObjectName(_fromUtf8("box_2"))
        self.box_7 = QtGui.QLineEdit(self.tab_3)
        self.box_7.setGeometry(QtCore.QRect(40, 200, 71, 27))
        self.box_7.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_7.setReadOnly(True)
        self.box_7.setObjectName(_fromUtf8("box_7"))
        self.box_6 = QtGui.QLineEdit(self.tab_3)
        self.box_6.setGeometry(QtCore.QRect(40, 170, 71, 27))
        self.box_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_6.setReadOnly(True)
        self.box_6.setObjectName(_fromUtf8("box_6"))
        self.box_5 = QtGui.QLineEdit(self.tab_3)
        self.box_5.setGeometry(QtCore.QRect(40, 140, 71, 27))
        self.box_5.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_5.setReadOnly(True)
        self.box_5.setObjectName(_fromUtf8("box_5"))
        self.box_4 = QtGui.QLineEdit(self.tab_3)
        self.box_4.setGeometry(QtCore.QRect(40, 110, 71, 27))
        self.box_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_4.setReadOnly(True)
        self.box_4.setObjectName(_fromUtf8("box_4"))
        self.box_3 = QtGui.QLineEdit(self.tab_3)
        self.box_3.setGeometry(QtCore.QRect(40, 80, 71, 27))
        self.box_3.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_3.setReadOnly(True)
        self.box_3.setObjectName(_fromUtf8("box_3"))
        self.box_8 = QtGui.QLineEdit(self.tab_3)
        self.box_8.setGeometry(QtCore.QRect(120, 20, 71, 27))
        self.box_8.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_8.setReadOnly(True)
        self.box_8.setObjectName(_fromUtf8("box_8"))
        self.box_9 = QtGui.QLineEdit(self.tab_3)
        self.box_9.setGeometry(QtCore.QRect(120, 50, 71, 27))
        self.box_9.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_9.setReadOnly(True)
        self.box_9.setObjectName(_fromUtf8("box_9"))
        self.box_10 = QtGui.QLineEdit(self.tab_3)
        self.box_10.setGeometry(QtCore.QRect(120, 80, 71, 27))
        self.box_10.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_10.setReadOnly(True)
        self.box_10.setObjectName(_fromUtf8("box_10"))
        self.box_11 = QtGui.QLineEdit(self.tab_3)
        self.box_11.setGeometry(QtCore.QRect(120, 110, 71, 27))
        self.box_11.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_11.setReadOnly(True)
        self.box_11.setObjectName(_fromUtf8("box_11"))
        self.box_12 = QtGui.QLineEdit(self.tab_3)
        self.box_12.setGeometry(QtCore.QRect(120, 140, 71, 27))
        self.box_12.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_12.setReadOnly(True)
        self.box_12.setObjectName(_fromUtf8("box_12"))
        self.box_13 = QtGui.QLineEdit(self.tab_3)
        self.box_13.setGeometry(QtCore.QRect(120, 170, 71, 27))
        self.box_13.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_13.setReadOnly(True)
        self.box_13.setObjectName(_fromUtf8("box_13"))
        self.box_14 = QtGui.QLineEdit(self.tab_3)
        self.box_14.setGeometry(QtCore.QRect(120, 200, 71, 27))
        self.box_14.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_14.setReadOnly(True)
        self.box_14.setObjectName(_fromUtf8("box_14"))
        self.box_15 = QtGui.QLineEdit(self.tab_3)
        self.box_15.setGeometry(QtCore.QRect(200, 20, 71, 27))
        self.box_15.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_15.setReadOnly(True)
        self.box_15.setObjectName(_fromUtf8("box_15"))
        self.box_16 = QtGui.QLineEdit(self.tab_3)
        self.box_16.setGeometry(QtCore.QRect(200, 50, 71, 27))
        self.box_16.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_16.setReadOnly(True)
        self.box_16.setObjectName(_fromUtf8("box_16"))
        self.box_17 = QtGui.QLineEdit(self.tab_3)
        self.box_17.setGeometry(QtCore.QRect(200, 80, 71, 27))
        self.box_17.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_17.setReadOnly(True)
        self.box_17.setObjectName(_fromUtf8("box_17"))
        self.box_18 = QtGui.QLineEdit(self.tab_3)
        self.box_18.setGeometry(QtCore.QRect(200, 110, 71, 27))
        self.box_18.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_18.setReadOnly(True)
        self.box_18.setObjectName(_fromUtf8("box_18"))
        self.box_19 = QtGui.QLineEdit(self.tab_3)
        self.box_19.setGeometry(QtCore.QRect(200, 140, 71, 27))
        self.box_19.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_19.setReadOnly(True)
        self.box_19.setObjectName(_fromUtf8("box_19"))
        self.box_20 = QtGui.QLineEdit(self.tab_3)
        self.box_20.setGeometry(QtCore.QRect(200, 170, 71, 27))
        self.box_20.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_20.setReadOnly(True)
        self.box_20.setObjectName(_fromUtf8("box_20"))
        self.box_21 = QtGui.QLineEdit(self.tab_3)
        self.box_21.setGeometry(QtCore.QRect(200, 200, 71, 27))
        self.box_21.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.box_21.setReadOnly(True)
        self.box_21.setObjectName(_fromUtf8("box_21"))
        self.tabWidget.addTab(self.tab_3, _fromUtf8(""))
        ControlPannel.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(ControlPannel)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 437, 25))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        ControlPannel.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(ControlPannel)
        self.statusbar.setObjectName(_fromUtf8("statusbar"))
        ControlPannel.setStatusBar(self.statusbar)

        self.retranslateUi(ControlPannel)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(ControlPannel)

    def retranslateUi(self, ControlPannel):
        ControlPannel.setWindowTitle(_translate("ControlPannel", "Quadcopter Control Pannel", None))
        self.set_land_position_button.setText(_translate("ControlPannel", "Set Land Position", None))
        self.groupBox_4.setTitle(_translate("ControlPannel", "IMU Values", None))
        self.label_9.setText(_translate("ControlPannel", "Roll", None))
        self.label_10.setText(_translate("ControlPannel", "Pitch", None))
        self.label_11.setText(_translate("ControlPannel", "Yaw", None))
        self.imu_roll_box.setText(_translate("ControlPannel", "0", None))
        self.imu_pitch_box.setText(_translate("ControlPannel", "0", None))
        self.imu_yaw_box.setText(_translate("ControlPannel", "0", None))
        self.control_toggle_button.setText(_translate("ControlPannel", "Take Over Control", None))
        self.groupBox_3.setTitle(_translate("ControlPannel", "Tracking Position and Orientation", None))
        self.label_2.setText(_translate("ControlPannel", "X Position", None))
        self.label_3.setText(_translate("ControlPannel", "Y Position", None))
        self.label_4.setText(_translate("ControlPannel", "Z Position", None))
        self.tracking_x_position_box.setText(_translate("ControlPannel", "0", None))
        self.tracking_y_position_box.setText(_translate("ControlPannel", "0", None))
        self.tracking_z_position_box.setText(_translate("ControlPannel", "0", None))
        self.label_5.setText(_translate("ControlPannel", "Roll", None))
        self.label_6.setText(_translate("ControlPannel", "Pitch", None))
        self.label_7.setText(_translate("ControlPannel", "Yaw", None))
        self.tracking_roll_box.setText(_translate("ControlPannel", "0", None))
        self.tracking_pitch_box.setText(_translate("ControlPannel", "0", None))
        self.tracking_yaw_box.setText(_translate("ControlPannel", "0", None))
        self.set_home_position_button.setText(_translate("ControlPannel", "Set Zero Position", None))
        self.set_home_orientation_button.setText(_translate("ControlPannel", "Set Zeros Orientation", None))
        self.groupBox_2.setTitle(_translate("ControlPannel", "User Position and Orientation", None))
        self.goto_position_button.setText(_translate("ControlPannel", "Go To Position", None))
        self.label_8.setText(_translate("ControlPannel", "X Position", None))
        self.label_12.setText(_translate("ControlPannel", "Y Position", None))
        self.label_13.setText(_translate("ControlPannel", "Z Position", None))
        self.label_15.setText(_translate("ControlPannel", "Yaw", None))
        self.goto_x_position_box.setText(_translate("ControlPannel", "0", None))
        self.goto_y_position_box.setText(_translate("ControlPannel", "0", None))
        self.goto_z_position_box.setText(_translate("ControlPannel", "0", None))
        self.goto_yaw_box.setText(_translate("ControlPannel", "0", None))
        self.take_off_land_button.setText(_translate("ControlPannel", "Take Off", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), _translate("ControlPannel", "Home", None))
        self.groupBox.setTitle(_translate("ControlPannel", "RC Channel Outputs", None))
        self.ch_box_1.setText(_translate("ControlPannel", "0", None))
        self.ch_box_2.setText(_translate("ControlPannel", "0", None))
        self.ch_box_3.setText(_translate("ControlPannel", "0", None))
        self.ch_box_4.setText(_translate("ControlPannel", "0", None))
        self.ch_box_5.setText(_translate("ControlPannel", "0", None))
        self.ch_box_6.setText(_translate("ControlPannel", "0", None))
        self.ch_box_7.setText(_translate("ControlPannel", "0", None))
        self.ch_box_8.setText(_translate("ControlPannel", "0", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), _translate("ControlPannel", "RC ", None))
        self.box_1.setText(_translate("ControlPannel", "0", None))
        self.box_2.setText(_translate("ControlPannel", "0", None))
        self.box_7.setText(_translate("ControlPannel", "0", None))
        self.box_6.setText(_translate("ControlPannel", "0", None))
        self.box_5.setText(_translate("ControlPannel", "0", None))
        self.box_4.setText(_translate("ControlPannel", "0", None))
        self.box_3.setText(_translate("ControlPannel", "0", None))
        self.box_8.setText(_translate("ControlPannel", "0", None))
        self.box_9.setText(_translate("ControlPannel", "0", None))
        self.box_10.setText(_translate("ControlPannel", "0", None))
        self.box_11.setText(_translate("ControlPannel", "0", None))
        self.box_12.setText(_translate("ControlPannel", "0", None))
        self.box_13.setText(_translate("ControlPannel", "0", None))
        self.box_14.setText(_translate("ControlPannel", "0", None))
        self.box_15.setText(_translate("ControlPannel", "0", None))
        self.box_16.setText(_translate("ControlPannel", "0", None))
        self.box_17.setText(_translate("ControlPannel", "0", None))
        self.box_18.setText(_translate("ControlPannel", "0", None))
        self.box_19.setText(_translate("ControlPannel", "0", None))
        self.box_20.setText(_translate("ControlPannel", "0", None))
        self.box_21.setText(_translate("ControlPannel", "0", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), _translate("ControlPannel", "Debug Boxes", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    ControlPannel = QtGui.QMainWindow()
    ui = Ui_ControlPannel()
    ui.setupUi(ControlPannel)
    ControlPannel.show()
    sys.exit(app.exec_())

