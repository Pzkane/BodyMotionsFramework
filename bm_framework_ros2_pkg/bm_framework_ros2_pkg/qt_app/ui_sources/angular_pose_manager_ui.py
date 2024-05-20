# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'angular_pose_manager.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_Form(object):
    def setupUi(self, Form):
        if not Form.objectName():
            Form.setObjectName(u"Form")
        Form.resize(1238, 463)
        self.verticalLayout = QVBoxLayout(Form)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.pushButton_create_pose = QPushButton(Form)
        self.pushButton_create_pose.setObjectName(u"pushButton_create_pose")

        self.horizontalLayout_2.addWidget(self.pushButton_create_pose)

        self.label_pose_name = QLabel(Form)
        self.label_pose_name.setObjectName(u"label_pose_name")

        self.horizontalLayout_2.addWidget(self.label_pose_name)

        self.lineEdit_pose_name = QLineEdit(Form)
        self.lineEdit_pose_name.setObjectName(u"lineEdit_pose_name")

        self.horizontalLayout_2.addWidget(self.lineEdit_pose_name)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.checkBox_limb_left = QCheckBox(Form)
        self.checkBox_limb_left.setObjectName(u"checkBox_limb_left")
        self.checkBox_limb_left.setChecked(True)

        self.horizontalLayout_4.addWidget(self.checkBox_limb_left)

        self.checkBox_left = QCheckBox(Form)
        self.checkBox_left.setObjectName(u"checkBox_left")
        self.checkBox_left.setChecked(True)

        self.horizontalLayout_4.addWidget(self.checkBox_left)

        self.checkBox_center = QCheckBox(Form)
        self.checkBox_center.setObjectName(u"checkBox_center")
        self.checkBox_center.setChecked(True)

        self.horizontalLayout_4.addWidget(self.checkBox_center)

        self.checkBox_right = QCheckBox(Form)
        self.checkBox_right.setObjectName(u"checkBox_right")
        self.checkBox_right.setChecked(True)

        self.horizontalLayout_4.addWidget(self.checkBox_right)

        self.checkBox_limb_right = QCheckBox(Form)
        self.checkBox_limb_right.setObjectName(u"checkBox_limb_right")
        self.checkBox_limb_right.setChecked(True)

        self.horizontalLayout_4.addWidget(self.checkBox_limb_right)


        self.verticalLayout_2.addLayout(self.horizontalLayout_4)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.radioButton_limb_left = QRadioButton(Form)
        self.radioButton_limb_left.setObjectName(u"radioButton_limb_left")

        self.horizontalLayout_3.addWidget(self.radioButton_limb_left)

        self.radioButton_left = QRadioButton(Form)
        self.radioButton_left.setObjectName(u"radioButton_left")

        self.horizontalLayout_3.addWidget(self.radioButton_left)

        self.radioButton_center = QRadioButton(Form)
        self.radioButton_center.setObjectName(u"radioButton_center")
        self.radioButton_center.setChecked(True)

        self.horizontalLayout_3.addWidget(self.radioButton_center)

        self.radioButton_right = QRadioButton(Form)
        self.radioButton_right.setObjectName(u"radioButton_right")
        self.radioButton_right.setChecked(False)

        self.horizontalLayout_3.addWidget(self.radioButton_right)

        self.radioButton_limb_right = QRadioButton(Form)
        self.radioButton_limb_right.setObjectName(u"radioButton_limb_right")

        self.horizontalLayout_3.addWidget(self.radioButton_limb_right)


        self.verticalLayout_2.addLayout(self.horizontalLayout_3)


        self.horizontalLayout_2.addLayout(self.verticalLayout_2)

        self.widget_pose_type = QWidget(Form)
        self.widget_pose_type.setObjectName(u"widget_pose_type")
        self.horizontalLayout_5 = QHBoxLayout(self.widget_pose_type)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label = QLabel(self.widget_pose_type)
        self.label.setObjectName(u"label")

        self.horizontalLayout_5.addWidget(self.label)

        self.radioButton_static = QRadioButton(self.widget_pose_type)
        self.radioButton_static.setObjectName(u"radioButton_static")
        self.radioButton_static.setChecked(True)

        self.horizontalLayout_5.addWidget(self.radioButton_static)

        self.radioButton_impulse = QRadioButton(self.widget_pose_type)
        self.radioButton_impulse.setObjectName(u"radioButton_impulse")
        self.radioButton_impulse.setChecked(False)

        self.horizontalLayout_5.addWidget(self.radioButton_impulse)


        self.horizontalLayout_2.addWidget(self.widget_pose_type)

        self.widget_imp_target_sensor = QWidget(Form)
        self.widget_imp_target_sensor.setObjectName(u"widget_imp_target_sensor")
        self.widget_imp_target_sensor.setEnabled(False)
        self.horizontalLayout_6 = QHBoxLayout(self.widget_imp_target_sensor)
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_target_sensor = QLabel(self.widget_imp_target_sensor)
        self.label_target_sensor.setObjectName(u"label_target_sensor")

        self.horizontalLayout_6.addWidget(self.label_target_sensor)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.radioButton_limb_left_imp = QRadioButton(self.widget_imp_target_sensor)
        self.radioButton_limb_left_imp.setObjectName(u"radioButton_limb_left_imp")

        self.horizontalLayout_7.addWidget(self.radioButton_limb_left_imp)

        self.radioButton_left_imp = QRadioButton(self.widget_imp_target_sensor)
        self.radioButton_left_imp.setObjectName(u"radioButton_left_imp")

        self.horizontalLayout_7.addWidget(self.radioButton_left_imp)

        self.radioButton_center_imp = QRadioButton(self.widget_imp_target_sensor)
        self.radioButton_center_imp.setObjectName(u"radioButton_center_imp")
        self.radioButton_center_imp.setChecked(True)

        self.horizontalLayout_7.addWidget(self.radioButton_center_imp)

        self.radioButton_right_imp = QRadioButton(self.widget_imp_target_sensor)
        self.radioButton_right_imp.setObjectName(u"radioButton_right_imp")
        self.radioButton_right_imp.setChecked(False)

        self.horizontalLayout_7.addWidget(self.radioButton_right_imp)

        self.radioButton_limb_right_imp = QRadioButton(self.widget_imp_target_sensor)
        self.radioButton_limb_right_imp.setObjectName(u"radioButton_limb_right_imp")

        self.horizontalLayout_7.addWidget(self.radioButton_limb_right_imp)


        self.horizontalLayout_6.addLayout(self.horizontalLayout_7)

        self.label_impulse_status = QLabel(self.widget_imp_target_sensor)
        self.label_impulse_status.setObjectName(u"label_impulse_status")

        self.horizontalLayout_6.addWidget(self.label_impulse_status)


        self.horizontalLayout_2.addWidget(self.widget_imp_target_sensor)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout_2.addItem(self.horizontalSpacer)

        self.horizontalLayout_2.setStretch(5, 1)
        self.horizontalLayout_2.setStretch(6, 1)

        self.verticalLayout.addLayout(self.horizontalLayout_2)

        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout_sensor_readings_7 = QVBoxLayout()
        self.verticalLayout_sensor_readings_7.setObjectName(u"verticalLayout_sensor_readings_7")
        self.horizontalLayout_sensor_info_7 = QHBoxLayout()
        self.horizontalLayout_sensor_info_7.setObjectName(u"horizontalLayout_sensor_info_7")
        self.label_metadata = QLabel(Form)
        self.label_metadata.setObjectName(u"label_metadata")
        self.label_metadata.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_sensor_info_7.addWidget(self.label_metadata)


        self.verticalLayout_sensor_readings_7.addLayout(self.horizontalLayout_sensor_info_7)

        self.tableWidget_metadata = QTableWidget(Form)
        self.tableWidget_metadata.setObjectName(u"tableWidget_metadata")
        self.tableWidget_metadata.setColumnCount(0)

        self.verticalLayout_sensor_readings_7.addWidget(self.tableWidget_metadata)


        self.horizontalLayout.addLayout(self.verticalLayout_sensor_readings_7)

        self.verticalLayout_sensor_readings_2 = QVBoxLayout()
        self.verticalLayout_sensor_readings_2.setObjectName(u"verticalLayout_sensor_readings_2")
        self.horizontalLayout_sensor_info_2 = QHBoxLayout()
        self.horizontalLayout_sensor_info_2.setObjectName(u"horizontalLayout_sensor_info_2")
        self.label_sensor_limb_left = QLabel(Form)
        self.label_sensor_limb_left.setObjectName(u"label_sensor_limb_left")
        self.label_sensor_limb_left.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_sensor_info_2.addWidget(self.label_sensor_limb_left)


        self.verticalLayout_sensor_readings_2.addLayout(self.horizontalLayout_sensor_info_2)

        self.tableWidget_limb_left = QTableWidget(Form)
        if (self.tableWidget_limb_left.columnCount() < 3):
            self.tableWidget_limb_left.setColumnCount(3)
        self.tableWidget_limb_left.setObjectName(u"tableWidget_limb_left")
        self.tableWidget_limb_left.setColumnCount(3)

        self.verticalLayout_sensor_readings_2.addWidget(self.tableWidget_limb_left)


        self.horizontalLayout.addLayout(self.verticalLayout_sensor_readings_2)

        self.verticalLayout_sensor_readings_4 = QVBoxLayout()
        self.verticalLayout_sensor_readings_4.setObjectName(u"verticalLayout_sensor_readings_4")
        self.horizontalLayout_sensor_info_4 = QHBoxLayout()
        self.horizontalLayout_sensor_info_4.setObjectName(u"horizontalLayout_sensor_info_4")
        self.label_sensor_left = QLabel(Form)
        self.label_sensor_left.setObjectName(u"label_sensor_left")
        self.label_sensor_left.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_sensor_info_4.addWidget(self.label_sensor_left)


        self.verticalLayout_sensor_readings_4.addLayout(self.horizontalLayout_sensor_info_4)

        self.tableWidget_left = QTableWidget(Form)
        if (self.tableWidget_left.columnCount() < 3):
            self.tableWidget_left.setColumnCount(3)
        self.tableWidget_left.setObjectName(u"tableWidget_left")
        self.tableWidget_left.setColumnCount(3)

        self.verticalLayout_sensor_readings_4.addWidget(self.tableWidget_left)


        self.horizontalLayout.addLayout(self.verticalLayout_sensor_readings_4)

        self.verticalLayout_sensor_readings_5 = QVBoxLayout()
        self.verticalLayout_sensor_readings_5.setObjectName(u"verticalLayout_sensor_readings_5")
        self.horizontalLayout_sensor_info_5 = QHBoxLayout()
        self.horizontalLayout_sensor_info_5.setObjectName(u"horizontalLayout_sensor_info_5")
        self.label_sensor_center = QLabel(Form)
        self.label_sensor_center.setObjectName(u"label_sensor_center")
        self.label_sensor_center.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_sensor_info_5.addWidget(self.label_sensor_center)


        self.verticalLayout_sensor_readings_5.addLayout(self.horizontalLayout_sensor_info_5)

        self.tableWidget_center = QTableWidget(Form)
        if (self.tableWidget_center.columnCount() < 3):
            self.tableWidget_center.setColumnCount(3)
        self.tableWidget_center.setObjectName(u"tableWidget_center")
        self.tableWidget_center.setColumnCount(3)

        self.verticalLayout_sensor_readings_5.addWidget(self.tableWidget_center)


        self.horizontalLayout.addLayout(self.verticalLayout_sensor_readings_5)

        self.verticalLayout_sensor_readings_3 = QVBoxLayout()
        self.verticalLayout_sensor_readings_3.setObjectName(u"verticalLayout_sensor_readings_3")
        self.horizontalLayout_sensor_info_3 = QHBoxLayout()
        self.horizontalLayout_sensor_info_3.setObjectName(u"horizontalLayout_sensor_info_3")
        self.label_sensor_right = QLabel(Form)
        self.label_sensor_right.setObjectName(u"label_sensor_right")
        self.label_sensor_right.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_sensor_info_3.addWidget(self.label_sensor_right)


        self.verticalLayout_sensor_readings_3.addLayout(self.horizontalLayout_sensor_info_3)

        self.tableWidget_right = QTableWidget(Form)
        if (self.tableWidget_right.columnCount() < 3):
            self.tableWidget_right.setColumnCount(3)
        self.tableWidget_right.setObjectName(u"tableWidget_right")
        self.tableWidget_right.setColumnCount(3)

        self.verticalLayout_sensor_readings_3.addWidget(self.tableWidget_right)


        self.horizontalLayout.addLayout(self.verticalLayout_sensor_readings_3)

        self.verticalLayout_sensor_readings = QVBoxLayout()
        self.verticalLayout_sensor_readings.setObjectName(u"verticalLayout_sensor_readings")
        self.horizontalLayout_sensor_info = QHBoxLayout()
        self.horizontalLayout_sensor_info.setObjectName(u"horizontalLayout_sensor_info")
        self.label_sensor_limb_right = QLabel(Form)
        self.label_sensor_limb_right.setObjectName(u"label_sensor_limb_right")
        self.label_sensor_limb_right.setAlignment(Qt.AlignCenter)

        self.horizontalLayout_sensor_info.addWidget(self.label_sensor_limb_right)


        self.verticalLayout_sensor_readings.addLayout(self.horizontalLayout_sensor_info)

        self.tableWidget_limb_right = QTableWidget(Form)
        if (self.tableWidget_limb_right.columnCount() < 3):
            self.tableWidget_limb_right.setColumnCount(3)
        self.tableWidget_limb_right.setObjectName(u"tableWidget_limb_right")
        self.tableWidget_limb_right.setColumnCount(3)

        self.verticalLayout_sensor_readings.addWidget(self.tableWidget_limb_right)


        self.horizontalLayout.addLayout(self.verticalLayout_sensor_readings)


        self.verticalLayout.addLayout(self.horizontalLayout)


        self.retranslateUi(Form)

        QMetaObject.connectSlotsByName(Form)
    # setupUi

    def retranslateUi(self, Form):
        Form.setWindowTitle(QCoreApplication.translate("Form", u"Form", None))
        self.pushButton_create_pose.setText(QCoreApplication.translate("Form", u"Create New Pose", None))
        self.label_pose_name.setText(QCoreApplication.translate("Form", u"Pose Name:", None))
        self.checkBox_limb_left.setText(QCoreApplication.translate("Form", u"LL", None))
        self.checkBox_left.setText(QCoreApplication.translate("Form", u"L", None))
        self.checkBox_center.setText(QCoreApplication.translate("Form", u"C", None))
        self.checkBox_right.setText(QCoreApplication.translate("Form", u"R", None))
        self.checkBox_limb_right.setText(QCoreApplication.translate("Form", u"LR", None))
        self.radioButton_limb_left.setText("")
        self.radioButton_left.setText("")
        self.radioButton_center.setText("")
        self.radioButton_right.setText("")
        self.radioButton_limb_right.setText("")
        self.label.setText(QCoreApplication.translate("Form", u"Type:", None))
        self.radioButton_static.setText(QCoreApplication.translate("Form", u"static", None))
        self.radioButton_impulse.setText(QCoreApplication.translate("Form", u"impulse", None))
        self.label_target_sensor.setText(QCoreApplication.translate("Form", u"Target Sensor:", None))
        self.radioButton_limb_left_imp.setText(QCoreApplication.translate("Form", u"LL", None))
        self.radioButton_left_imp.setText(QCoreApplication.translate("Form", u"L", None))
        self.radioButton_center_imp.setText(QCoreApplication.translate("Form", u"C", None))
        self.radioButton_right_imp.setText(QCoreApplication.translate("Form", u"R", None))
        self.radioButton_limb_right_imp.setText(QCoreApplication.translate("Form", u"LR", None))
        self.label_impulse_status.setText("")
        self.label_metadata.setText(QCoreApplication.translate("Form", u"Metadata", None))
        self.label_sensor_limb_left.setText(QCoreApplication.translate("Form", u"Limb Left", None))
        self.label_sensor_left.setText(QCoreApplication.translate("Form", u"Left", None))
        self.label_sensor_center.setText(QCoreApplication.translate("Form", u"Center", None))
        self.label_sensor_right.setText(QCoreApplication.translate("Form", u"Right", None))
        self.label_sensor_limb_right.setText(QCoreApplication.translate("Form", u"Limb Right", None))
    # retranslateUi

