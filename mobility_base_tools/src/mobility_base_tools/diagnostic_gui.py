#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014-2017, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
import rospy
import rospkg
import time

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QGraphicsScene, QLabel, QGraphicsEllipseItem, QGraphicsLineItem, QGraphicsTextItem
from python_qt_binding.QtCore import Qt, Signal, QTimer, QDateTime, QDate, QTime, QPoint
from python_qt_binding.QtGui import QPixmap, QFont, QPen, QColor, QBrush, QImage
    
from cmath import *
from math import *

from mobility_base_core_msgs.msg import Mode, JoystickRaw
from rospy.rostime import Time
from geometry_msgs.msg import Twist

from mobility_base_core_msgs.msg import BumperState
from mobility_base_core_msgs.msg import BatteryState
from sensor_msgs.msg import Imu
import std_msgs.msg
import std_srvs.srv
from mobility_base_core_msgs.srv import GetMaxSpeed, SetMaxSpeed
from numpy import isfinite
from mobility_base_core_msgs.srv import SetMaxSpeedRequest


class BumperIndex:
    def __init__(self):
        pass

    BUMPER_1F = 0
    BUMPER_1L = 1
    BUMPER_1R = 2
    BUMPER_1B = 3
    
    BUMPER_2F = 4
    BUMPER_2L = 5
    BUMPER_2R = 6
    BUMPER_2B = 7
    
    BUMPER_3F = 8
    BUMPER_3L = 9
    BUMPER_3R = 10
    BUMPER_3B = 11
    
    BUMPER_4F = 12
    BUMPER_4L = 13
    BUMPER_4R = 14
    BUMPER_4B = 15    


class ChannelIndex:
    def __init__(self):
        pass

    CHAN_ENABLE = 0
    CHAN_ROTATE = 1
    CHAN_FORWARD = 2
    CHAN_LATERAL = 3
    CHAN_MODE = 4
    CHAN_EXTRA = 5       


class DiagnosticGui(Plugin):
    last_suppress_time = Time()
    last_twist_time = Time()
    gui_update_timer = QTimer()
    
    # Raw joystick data    
    joystick_data = []
    joystick_table_vals = []
    joystick_channel_text = ['CHAN_ENABLE: ', 'CHAN_ROTATE: ', 'CHAN_FORWARD: ', 'CHAN_LATERAL: ', 'CHAN_MODE: ', 'CHAN_EXTRA: ']
    joystick_bind_status = False
    joystick_bind_dot_counter = 0
    
    # Mode
    current_mode = -1
    
    # Topic pub timeouts    
    JOYSTICK_SUPPRESS_PERIOD = 0.2  # 5 Hz
    CMD_VEL_TIMEOUT_PERIOD = 0.2    # 5 Hz
    joystick_suppressed = False
    command_received = False
    current_cmd = Twist()
    battery_percent = 0
    battery_voltage = 0
    
    # Checklist icons
    bad_icon = QPixmap()
    good_icon = QPixmap()
    none_icon = QPixmap()
    
    # Checklist status
    GOOD = 0
    BAD = 1
    NONE = 2
    
    # Checklist variables
    checklist_status = []
    
    BATT_MAN = 0
    ESTOP_MAN = 1
    DISABLE_MAN = 2
    JOYSTICK_MAN = 3
    SUPPRESS_MAN = 4    
    BATT_COMP = 5
    ESTOP_COMP = 6
    DISABLE_COMP = 7
    JOYSTICK_COMP = 8
    SUPPRESS_COMP = 9
    CMD_COMP = 10
    
    # Bumpers
    bumper_front_left = 0
    bumper_front_right = 0
    bumper_rear_left = 0
    bumper_rear_right = 0
    
    # Gyro cal
    gyro_cal_status = False
    gyro_x = 0.0
    gyro_y = 0.0
    gyro_z = 0.0
    cal_enabled = True
    cal_time = rospy.Time(0)
    
    # Max speed config
    max_speed_known = False
    max_speed_dirty = True
    max_linear_actual = 0.0
    max_angular_actual = 0.0
    max_linear_setting = 0.0
    max_angular_setting = 0.0
    
    # Wake time
    current_wake_time = rospy.Time(0)
    rel_wake_days = 0
    rel_wake_hours = 0
    rel_wake_minutes = 0
    rel_wake_secs = 0
    
    # Switching between tabs and full GUI
    is_currently_tab = False
    widget_count = 0
    current_tab_idx = -1
    raw_data_tab_idx = 5
    
    # Check connection to base
    base_connected = False
    last_joystick_time = rospy.Time(0)
    
    # Constants    
    stick_ind_lox = 80
    stick_ind_loy = 136
    stick_ind_rox = 286
    stick_ind_roy = 135
    stick_ind_range_pix = 88.0
    stick_ind_range_max = JoystickRaw.MAX
    stick_ind_range_min = JoystickRaw.MIN
    stick_ind_range_mid = JoystickRaw.CENTER
    stick_ind_range_factor = stick_ind_range_pix / (stick_ind_range_max - stick_ind_range_min)
    stick_ind_radius = 7    
    mode_ind_x1 = 52
    mode_ind_y1 = 37
    mode_ind_x2 = 44
    mode_ind_y2 = 13
    power_ind_x1 = 160
    power_ind_x2 = 206
    power_ind_y = 213    
    bumper_fl_x = 70
    bumper_fl_y = 60
    bumper_fr_x = 293
    bumper_fr_y = 60
    bumper_rl_x = 70
    bumper_rl_y = 282
    bumper_rr_x = 293
    bumper_rr_y = 282
    bumper_dx = 62
    bumper_dy = 54    
    joystick_table_left_edge = 440
    joystick_table_top_edge = 525    
    twist_table_left_edge = 700
    twist_table_top_edge = 580    
    battery_table_left_edge = 700
    battery_table_top_edge = 730

    _deferred_fit_in_view = Signal()
    
    def __init__(self, context):
        super(DiagnosticGui, self).__init__(context)

        # Qt setup
        self.context_ = context
        self.init_gui('full')
        
        # ROS setup
        self.topic_timeout_timer = rospy.Timer(rospy.Duration(0.02), self.topic_timeout_cb)
        self.subscribe_topics()
        self.advertise_topics()
        
    def init_gui(self, gui_type):
        if gui_type == 'full':
            self.spawn_full_gui()
            self.init_tables(self._widget, self.joystick_table_left_edge, self.joystick_table_top_edge)
        else:
            self.spawn_tab_gui()
            self.init_tables(self._widget.gui_tabs.widget(self.raw_data_tab_idx), 20, 20)
            self._widget.gui_tabs.setCurrentIndex(self.current_tab_idx)
            
        self.reset_gui_timer()
        self.init_joystick_graphics()
        self.init_bumper_graphics()
        self.init_checklists()
        self.bind_callbacks()
        self.refresh_max_speed()
        
        # Initialize absolute wake time setter to current time
        datetime_now = QDateTime(QDate.currentDate(), QTime.currentTime())
        self._widget.absolute_wake_time_obj.setDateTime(datetime_now)
        temp_time = self._widget.absolute_wake_time_obj.time()
        temp_time = QTime(temp_time.hour(), temp_time.minute())
        self._widget.absolute_wake_time_obj.setTime(temp_time)
        
        # Set connection label text
        if self.base_connected:
            self._widget.disconnected_lbl.setVisible(False)
        else:
            self._widget.disconnected_lbl.setVisible(True)
            self._widget.disconnected_lbl.setText('<font color=#FF0000>NOT CONNECTED</font>')

    def topic_timeout_cb(self, event):
        # Joystick suppression
        if (event.current_real - self.last_suppress_time).to_sec() < self.JOYSTICK_SUPPRESS_PERIOD and self.suppress_dt.to_sec() <= 1.0/9.0:
            self.joystick_suppressed = True
        else:
            self.joystick_suppressed = False
            
        # Command message
        if (event.current_real - self.last_twist_time).to_sec() < self.CMD_VEL_TIMEOUT_PERIOD and self.twist_dt.to_sec() <= 1.0/6.0:
            self.command_received = True
        else:
            self.command_received = False

    def init_checklists(self):
        self.bad_icon.load(os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'images', 'bad.png'))
        self.good_icon.load(os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'images', 'good.png'))
        self.none_icon.load(os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'images', 'none.png'))        
        self.checklist_status=[0 for i in range(self.CMD_COMP + 1)]
        self.checklist_status[self.BATT_MAN] = self.NONE
        self.checklist_status[self.ESTOP_MAN] = self.NONE
        self.checklist_status[self.DISABLE_MAN] = self.NONE
        self.checklist_status[self.JOYSTICK_MAN] = self.NONE
        self.checklist_status[self.SUPPRESS_MAN] = self.NONE        
        self.checklist_status[self.BATT_COMP] = self.NONE
        self.checklist_status[self.ESTOP_COMP] = self.NONE
        self.checklist_status[self.DISABLE_COMP] = self.NONE
        self.checklist_status[self.JOYSTICK_COMP] = self.NONE
        self.checklist_status[self.SUPPRESS_COMP] = self.NONE
        self.checklist_status[self.CMD_COMP] = self.NONE

    def init_tab_tables(self):
        self.joystick_table_vals = [QLabel(self._widget.gui_tabs.widget(self.raw_data_tab_idx)) for i in range(ChannelIndex.CHAN_EXTRA+1)]
        self.joystick_table_labels = [QLabel(self._widget.gui_tabs.widget(self.raw_data_tab_idx)) for i in range(ChannelIndex.CHAN_EXTRA+1)]
        self.joystick_table_heading = QLabel(self._widget.gui_tabs.widget(self.raw_data_tab_idx))
        
    def init_tables(self, widget, left, top):
        # Joystick data table
        self.joystick_data = [0 for i in range(ChannelIndex.CHAN_EXTRA+1)]
        self.joystick_table_vals = [QLabel(widget) for i in range(ChannelIndex.CHAN_EXTRA+1)]
        self.joystick_table_labels = [QLabel(widget) for i in range(ChannelIndex.CHAN_EXTRA+1)]
        self.joystick_table_heading = QLabel(widget)
        
        self.joystick_table_heading.setText('Raw Joystick Data')
        self.joystick_table_heading.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.joystick_table_heading.move(left, top)
        for i in range(len(self.joystick_table_vals)):
            self.joystick_table_vals[i].move(left + 150, top + 30 * (i+1))
            self.joystick_table_vals[i].setText('0000')
            self.joystick_table_vals[i].setFixedWidth(200)
            
            self.joystick_table_labels[i].move(left, top + 30 * (i+1))
            self.joystick_table_labels[i].setText(self.joystick_channel_text[i])
            
        # Twist data table
        self.twist_table_heading = QLabel(widget)
        self.twist_table_heading.setText('Current Twist Command')
        self.twist_table_heading.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.twist_table_heading.move(left + 260, top)
        self.twist_table_labels = [QLabel(widget) for i in range(0, 3)]
        self.twist_table_vals = [QLabel(widget) for i in range(0, 3)]
        for i in range(len(self.twist_table_vals)):
            self.twist_table_vals[i].move(left + 260 + 150, top + 30 * (i+1))
            self.twist_table_vals[i].setText('Not Published')
            self.twist_table_vals[i].setFixedWidth(200)
            self.twist_table_labels[i].move(left + 260, top + 30 * (i+1))
            
        self.twist_table_labels[0].setText('Forward (m/s):')
        self.twist_table_labels[1].setText('Lateral (m/s):')
        self.twist_table_labels[2].setText('Rotation (rad/s):')
        
        # Battery percentage
        self.battery_heading = QLabel(widget)
        self.battery_heading.setText('Current Battery State')
        self.battery_heading.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.battery_heading.move(left + 260, top + 150)
        self.battery_label = QLabel(widget)
        self.battery_label.move(left + 260, top + 150 +30)
        self.battery_label.setText('000 %')
        self.battery_voltage_label = QLabel(widget)
        self.battery_voltage_label.move(left + 260, top + 150 +60)
        self.battery_voltage_label.setText('00.00 V')
        self.battery_voltage_label.setFixedWidth(200)
        
        # Mode
        self.mode_heading = QLabel(widget)
        self.mode_heading.setFont(QFont('Ubuntu', 11, QFont.Bold))
        self.mode_heading.move(left, top + 225)
        self.mode_heading.setText('Current Mode')
        self.mode_label = QLabel(widget)
        self.mode_label.move(left + 120, top + 225)        
        self.mode_label.setText('XXXXXXXXXXXXXXXXXXXXXX')

    def bind_callbacks(self):
        self._widget.start_bind_btn.clicked.connect(self.start_bind)
        self._widget.stop_bind_btn.clicked.connect(self.stop_bind)
        
        self._widget.gyro_cal_btn.clicked.connect(self.cal_gyro)
        self._widget.clear_cal_btn.clicked.connect(self.clear_cal)
        
        self._widget.max_linear_txt.editingFinished.connect(self.max_linear_changed)
        self._widget.max_angular_txt.editingFinished.connect(self.max_angular_changed)
        self._widget.set_speed_btn.clicked.connect(self.set_max_speed)
        self._widget.clear_speed_btn.clicked.connect(self.clear_max_speed)
        
        self._widget.wake_time_days_txt.editingFinished.connect(self.wake_days_changed)
        self._widget.wake_time_hours_txt.editingFinished.connect(self.wake_hours_changed)
        self._widget.wake_time_minutes_txt.editingFinished.connect(self.wake_minutes_changed)
        self._widget.wake_time_secs_txt.editingFinished.connect(self.wake_secs_changed)
        self._widget.set_relative_wake_time_btn.clicked.connect(self.set_relative_wake_time)
        self._widget.set_absolute_wake_time_btn.clicked.connect(self.set_absolute_wake_time)
        self._widget.clear_wake_time_btn.clicked.connect(self.clear_wake_time)
        
        if self.is_currently_tab:
            self._widget.gui_tabs.currentChanged.connect(self.set_current_tab)
        
    def set_current_tab(self, idx):
        self.current_tab_idx = self._widget.gui_tabs.currentIndex()
        
    def start_bind(self):
        self.pub_start_bind.publish(std_msgs.msg.Empty())
        
    def stop_bind(self):
        self.pub_stop_bind.publish(std_msgs.msg.Empty())
        
    def cal_gyro(self):
        gyro_cal_srv = rospy.ServiceProxy('/mobility_base/imu/calibrate', std_srvs.srv.Empty)
        try:
            gyro_cal_srv()
            self.cal_enabled = False
            self.cal_time = rospy.Time.now()
        except:
            pass
 
    def clear_cal(self):
        clear_cal_srv = rospy.ServiceProxy('/mobility_base/imu/clear_cal', std_srvs.srv.Empty)
        try:
            clear_cal_srv()
        except:
            pass
        
    def max_linear_changed(self):
        try:
            float_val = float(self._widget.max_linear_txt.text())
            self.max_linear_setting = float_val;
        except ValueError:
            if self.max_linear_actual <= 0 or not isfinite(self.max_linear_actual):
                self.max_linear_setting = 0.0
            else:
                self.max_linear_setting = self.max_linear_actual
        self.max_speed_dirty = True
        
    def max_angular_changed(self):
        try:
            float_val = float(self._widget.max_angular_txt.text())
            self.max_angular_setting = float_val;
        except ValueError:
            if self.max_angular_actual <= 0 or not isfinite(self.max_angular_actual):
                self.max_angular_setting = 0.0
            else:
                self.max_angular_setting = self.max_angular_actual
        self.max_speed_dirty = True
        
    def set_max_speed(self):
        set_max_speed_srv = rospy.ServiceProxy('/mobility_base/set_max_speed', SetMaxSpeed)
        req = SetMaxSpeedRequest()
        req.linear = self.max_linear_setting
        req.angular = self.max_angular_setting
        try:
            set_max_speed_srv(req)
            self.max_speed_known = False
        except:
            pass
        
    def clear_max_speed(self):
        set_max_speed_srv = rospy.ServiceProxy('/mobility_base/set_max_speed', SetMaxSpeed)
        req = SetMaxSpeedRequest()
        req.linear = float('nan')
        req.angular = float('nan')
        try:
            set_max_speed_srv(req)
            self.max_speed_known = False
        except:
            pass

    def wake_days_changed(self):
        try:
            self.rel_wake_days = float(self._widget.wake_time_days_txt.text())
        except:
            self._widget.wake_time_days_txt.setText(str(self.rel_wake_days))
                        
    def wake_hours_changed(self):
        try:
            self.rel_wake_hours = float(self._widget.wake_time_hours_txt.text())
        except:
            self._widget.wake_time_hours_txt.setText(str(self.rel_wake_hours))
            
    def wake_minutes_changed(self):
        try:
            self.rel_wake_minutes = float(self._widget.wake_time_minutes_txt.text())
        except:
            self._widget.wake_time_minutes_txt.setText(str(self.rel_wake_minutes)) 

    def wake_secs_changed(self):
        try:
            self.rel_wake_secs = float(self._widget.wake_time_secs_txt.text())
        except:
            self._widget.wake_time_secs_txt.setText(str(self.rel_wake_secs)) 

    def set_relative_wake_time(self):
        new_wake_time = std_msgs.msg.Time()
        rel_wake_time = 86400 * self.rel_wake_days + 3600 * self.rel_wake_hours + 60 * self.rel_wake_minutes + self.rel_wake_secs
        new_wake_time.data = rospy.Time.now() + rospy.Duration(rel_wake_time)
        self.pub_set_wake_time.publish(new_wake_time)
        
    def set_absolute_wake_time(self):
        self.pub_set_wake_time.publish(rospy.Time(self._widget.absolute_wake_time_obj.dateTime().toTime_t()))
        
    def clear_wake_time(self):
        self.pub_set_wake_time.publish(rospy.Time(0))
    
    def refresh_max_speed(self):
        self.max_speed_dirty = True
        self.max_speed_known = False
    
    def update_gui_cb(self):
        
        # Switch between tabs and full GUI
        if self.is_currently_tab:
            if self._widget.height() > 830 and self._widget.width() > 1205:
                self.is_currently_tab = False
                self.context_.remove_widget(self._widget)
                self.init_gui('full')
        else:
            if self._widget.height() < 810 or self._widget.width() < 1185:
                self.is_currently_tab = True
                self.context_.remove_widget(self._widget)
                self.init_gui('tab')
        
        # Check connection to base
        if self.base_connected and (rospy.Time.now() - self.last_joystick_time).to_sec() > 1.0:
            self.base_connected = False
            self._widget.disconnected_lbl.setText('<font color=#FF0000>NOT CONNECTED</font>')
            self._widget.disconnected_lbl.setVisible(True)
        
        if not self.base_connected and (rospy.Time.now() - self.last_joystick_time).to_sec() < 1.0:
            self.base_connected = True
#             self._widget.disconnected_lbl.setText('')
            self._widget.disconnected_lbl.setVisible(False)
        
        # Update checklists
        self.update_checklist();

        # Manage 5 second disable of gyro calibration button
        if not self.cal_enabled:
            if (rospy.Time.now() - self.cal_time).to_sec() > 5.0:
                self._widget.gyro_cal_btn.setEnabled(True)
                self._widget.gyro_cal_btn.setText('Calibrate')
                self.cal_enabled = True
            else:
                self._widget.gyro_cal_btn.setEnabled(False)
                self._widget.gyro_cal_btn.setText(str(5 - 0.1*floor(10*(rospy.Time.now() - self.cal_time).to_sec()))) 
                
        # Update joystick graphics
        if not self.check_joystick_valid():
            self.update_check_status(self._widget.joystick_bind_chk, self.NONE)
            
            if not self.joystick_bind_status:
                self._widget.joystick_bind_lbl.setText('')

            
            for l in self.joystick_power_ind:
                l.setVisible(True)
            self.update_right_stick_indicator(self.stick_ind_range_mid, self.stick_ind_range_mid)
            self.update_left_stick_indicator(self.stick_ind_range_mid, self.stick_ind_range_mid)
        else:
            self.update_check_status(self._widget.joystick_bind_chk, self.GOOD)
            self._widget.joystick_bind_lbl.setText('Joystick bound')

            for l in self.joystick_power_ind:
                l.setVisible(False)
            self.update_right_stick_indicator(self.joystick_data[ChannelIndex.CHAN_LATERAL], self.joystick_data[ChannelIndex.CHAN_FORWARD])
            self.update_left_stick_indicator(self.joystick_data[ChannelIndex.CHAN_ROTATE], self.joystick_data[ChannelIndex.CHAN_ENABLE])
        if self.joystick_data[ChannelIndex.CHAN_MODE] < self.stick_ind_range_mid:
            self.mode_ind.setPen(self.cyan_pen)
            self._widget.modeLabel.setText("0 (Computer)")
        else:
            self.mode_ind.setPen(self.magenta_pen)
            self._widget.modeLabel.setText("1 (Manual)")
        
        # Update joystick data table
        for i in range(len(self.joystick_table_vals)):
            self.joystick_table_vals[i].setText(str(self.joystick_data[i]))
            
        # Update twist data table
        if self.command_received:
            self.twist_table_vals[0].setText(str(0.01 * floor(100 * self.current_cmd.linear.x)))
            self.twist_table_vals[1].setText(str(0.01 * floor(100 * self.current_cmd.linear.y)))
            self.twist_table_vals[2].setText(str(0.01 * floor(100 * self.current_cmd.angular.z)))
        else:
            self.twist_table_vals[0].setText('Not Published')
            self.twist_table_vals[1].setText('Not Published')
            self.twist_table_vals[2].setText('Not Published')
            
        # Update battery percentage
        self.battery_label.setText(str(self.battery_percent) + ' %')
        self.battery_voltage_label.setText(str(0.01 * floor(100 * self.battery_voltage)) + ' V')
        
        # Update mode
        self.mode_label.setText(self.get_mode_string(self.current_mode))
        
        # Update bumper graphics
        self.bumper_visible_switch(BumperIndex.BUMPER_1F, self.bumper_front_left)
        self.bumper_visible_switch(BumperIndex.BUMPER_2F, self.bumper_front_right)
        self.bumper_visible_switch(BumperIndex.BUMPER_3F, self.bumper_rear_left)
        self.bumper_visible_switch(BumperIndex.BUMPER_4F, self.bumper_rear_right)
        self.bumper_state_labels[0].setPlainText(str(self.bumper_front_left))
        self.bumper_state_labels[1].setPlainText(str(self.bumper_front_right))
        self.bumper_state_labels[2].setPlainText(str(self.bumper_rear_left))
        self.bumper_state_labels[3].setPlainText(str(self.bumper_rear_right))
        
        # Update gyro cal graphics
        if self.gyro_cal_status:            
            self.update_check_status(self._widget.gyro_cal_chk, self.GOOD)
            self._widget.gyro_cal_lbl.setText('Gyro calibrated')
        else:
            self.update_check_status(self._widget.gyro_cal_chk, self.BAD)
            self._widget.gyro_cal_lbl.setText('Gyro NOT calibrated')
    
        self._widget.gyro_x_lbl.setText('x: ' + str(1e-5*floor(1e5*self.gyro_x)))
        self._widget.gyro_y_lbl.setText('y: ' + str(1e-5*floor(1e5*self.gyro_y)))
        self._widget.gyro_z_lbl.setText('z: ' + str(1e-5*floor(1e5*self.gyro_z)))
    
        # Update max speed configuration
        if not self.max_speed_known:
            service_name = '/mobility_base/get_max_speed'
            get_max_speed_srv = rospy.ServiceProxy(service_name, GetMaxSpeed)
            
            try:
                resp = get_max_speed_srv()
                self.max_speed_known = True
                self.max_linear_actual = resp.linear
                self.max_angular_actual = resp.angular
                
                if self.max_linear_actual <= 0 or not isfinite(self.max_linear_actual):
                    self._widget.max_linear_lbl.setText('Unlimited')
                else:
                    self._widget.max_linear_lbl.setText(str(1e-2*round(1e2*self.max_linear_actual)))
                
                if self.max_angular_actual <= 0 or not isfinite(self.max_angular_actual):
                    self._widget.max_angular_lbl.setText('Unlimited')
                else:
                    self._widget.max_angular_lbl.setText(str(1e-2*round(1e2*self.max_angular_actual)))

            except:
                pass
#                 print service_name + " doesn't exist"
                
        if self.max_speed_dirty:        
            self._widget.max_linear_txt.setText(str(self.max_linear_setting))
            self._widget.max_angular_txt.setText(str(self.max_angular_setting))
            self.max_speed_dirty = False
       
        # Wake time
        if self.current_wake_time == rospy.Time(0):
            self._widget.wake_time_lbl.setText('Not Set')
        else:
            self._widget.wake_time_lbl.setText(time.strftime('%m/%d/%Y,  %H:%M:%S', time.localtime(self.current_wake_time.to_sec())))
            
    def check_joystick_valid(self):
        return self.joystick_data[ChannelIndex.CHAN_FORWARD] > 0 or self.joystick_data[ChannelIndex.CHAN_LATERAL] > 0 or self.joystick_data[ChannelIndex.CHAN_ROTATE] > 0 or self.joystick_data[ChannelIndex.CHAN_MODE] > 0
            
    def get_mode_string(self, mode_num):
        if mode_num == Mode.MODE_ESTOP:
            return 'MODE_ESTOP'
        elif mode_num == Mode.MODE_DISABLED:
            return 'MODE_DISABLED'
        elif mode_num == Mode.MODE_BATTERY_LIMP_HOME:
            return 'MODE_BATTERY_LIMP_HOME'
        elif mode_num == Mode.MODE_BATTERY_CRITICAL:
            return 'MODE_BATTERY_CRITICAL'
        elif mode_num == Mode.MODE_WIRELESS:
            return 'MODE_WIRELESS'
        elif mode_num == Mode.MODE_TIMEOUT:
            return 'MODE_TIMEOUT'
        elif mode_num == Mode.MODE_VELOCITY:
            return 'MODE_VELOCITY'
        elif mode_num == Mode.MODE_VELOCITY_RAW:
            return 'MODE_VELOCITY_RAW'
        else:
            return ''
        
    def update_checklist(self):
        if self.current_mode >= Mode.MODE_TIMEOUT:
            self.checklist_status[self.BATT_MAN] = self.GOOD
            self.checklist_status[self.ESTOP_MAN] = self.GOOD
            self.checklist_status[self.DISABLE_MAN] = self.GOOD
            self.checklist_status[self.JOYSTICK_MAN] = self.BAD
            self.checklist_status[self.BATT_COMP] = self.GOOD
            self.checklist_status[self.ESTOP_COMP] = self.GOOD
            self.checklist_status[self.DISABLE_COMP] = self.GOOD
            self.checklist_status[self.JOYSTICK_COMP] = self.GOOD
        elif self.current_mode == Mode.MODE_WIRELESS:
            self.checklist_status[self.BATT_MAN] = self.GOOD
            self.checklist_status[self.ESTOP_MAN] = self.GOOD
            self.checklist_status[self.DISABLE_MAN] = self.GOOD
            self.checklist_status[self.JOYSTICK_MAN] = self.GOOD
            self.checklist_status[self.BATT_COMP] = self.GOOD
            self.checklist_status[self.ESTOP_COMP] = self.GOOD
            self.checklist_status[self.DISABLE_COMP] = self.GOOD
            self.checklist_status[self.JOYSTICK_COMP] = self.BAD
        elif self.current_mode == Mode.MODE_DISABLED:
            self.checklist_status[self.BATT_MAN] = self.NONE
            self.checklist_status[self.ESTOP_MAN] = self.GOOD
            self.checklist_status[self.DISABLE_MAN] = self.BAD
            self.checklist_status[self.JOYSTICK_MAN] = self.NONE
            self.checklist_status[self.BATT_COMP] = self.NONE
            self.checklist_status[self.ESTOP_COMP] = self.GOOD
            self.checklist_status[self.DISABLE_COMP] = self.BAD
            self.checklist_status[self.JOYSTICK_COMP] = self.NONE
        elif self.current_mode == Mode.MODE_ESTOP:
            self.checklist_status[self.BATT_MAN] = self.NONE
            self.checklist_status[self.ESTOP_MAN] = self.BAD
            self.checklist_status[self.DISABLE_MAN] = self.NONE
            self.checklist_status[self.JOYSTICK_MAN] = self.NONE
            self.checklist_status[self.BATT_COMP] = self.NONE
            self.checklist_status[self.ESTOP_COMP] = self.BAD
            self.checklist_status[self.DISABLE_COMP] = self.NONE
            self.checklist_status[self.JOYSTICK_COMP] = self.NONE
        elif self.current_mode == Mode.MODE_BATTERY_CRITICAL:
            self.checklist_status[self.BATT_MAN] = self.BAD
            self.checklist_status[self.ESTOP_MAN] = self.GOOD
            self.checklist_status[self.DISABLE_MAN] = self.GOOD
            self.checklist_status[self.JOYSTICK_MAN] = self.NONE
            self.checklist_status[self.BATT_COMP] = self.BAD
            self.checklist_status[self.ESTOP_COMP] = self.GOOD
            self.checklist_status[self.DISABLE_COMP] = self.GOOD
            self.checklist_status[self.JOYSTICK_COMP] = self.NONE
        elif self.current_mode == Mode.MODE_BATTERY_LIMP_HOME:
            self.checklist_status[self.BATT_MAN] = self.GOOD
            self.checklist_status[self.ESTOP_MAN] = self.GOOD
            self.checklist_status[self.DISABLE_MAN] = self.GOOD
            self.checklist_status[self.JOYSTICK_MAN] = self.NONE
            self.checklist_status[self.BATT_COMP] = self.BAD
            self.checklist_status[self.ESTOP_COMP] = self.GOOD
            self.checklist_status[self.DISABLE_COMP] = self.GOOD
            self.checklist_status[self.JOYSTICK_COMP] = self.NONE
        
        # Check if joystick is suppressed by topic
        if self.joystick_suppressed:
            self.checklist_status[self.SUPPRESS_COMP] = self.GOOD
            self.checklist_status[self.DISABLE_COMP] = self.NONE
            self.checklist_status[self.JOYSTICK_COMP] = self.NONE
        else:
            self.checklist_status[self.SUPPRESS_COMP] = self.NONE
        if (rospy.Time.now() - self.last_suppress_time).to_sec() > self.JOYSTICK_SUPPRESS_PERIOD:
            self.checklist_status[self.SUPPRESS_MAN] = self.GOOD
        else:
            self.checklist_status[self.SUPPRESS_MAN] = self.BAD
        
        # Command message received
        if self.command_received:
            self.checklist_status[self.CMD_COMP] = self.GOOD
        else:
            self.checklist_status[self.CMD_COMP] = self.BAD
            
        # Override twist checkbox if mode is in TIMEOUT
        if self.current_mode == Mode.MODE_TIMEOUT:
            self.checklist_status[self.CMD_COMP] = self.BAD
        
        # Update checklist graphics    
        self.update_check_status(self._widget.batt_man_chk, self.checklist_status[self.BATT_MAN])
        self.update_check_status(self._widget.estop_man_chk, self.checklist_status[self.ESTOP_MAN])
        self.update_check_status(self._widget.disable_man_chk, self.checklist_status[self.DISABLE_MAN])
        self.update_check_status(self._widget.joystick_man_chk, self.checklist_status[self.JOYSTICK_MAN])
        self.update_check_status(self._widget.suppress_man_chk, self.checklist_status[self.SUPPRESS_MAN])
        self.update_check_status(self._widget.batt_comp_chk, self.checklist_status[self.BATT_COMP])
        self.update_check_status(self._widget.estop_comp_chk, self.checklist_status[self.ESTOP_COMP])
        self.update_check_status(self._widget.disable_comp_chk, self.checklist_status[self.DISABLE_COMP])
        self.update_check_status(self._widget.joystick_comp_chk, self.checklist_status[self.JOYSTICK_COMP])
        self.update_check_status(self._widget.suppress_comp_chk, self.checklist_status[self.SUPPRESS_COMP])
        self.update_check_status(self._widget.cmd_comp_chk, self.checklist_status[self.CMD_COMP])
        self.update_check_status(self._widget.joystick_bind_chk, self.NONE)

    def update_check_status(self, obj, icon_type):
        if icon_type == self.GOOD:
            obj.setPixmap(self.good_icon)
        elif icon_type == self.BAD:
            obj.setPixmap(self.bad_icon)
        else:
            obj.setPixmap(self.none_icon)
        
    def update_table(self):
        for i in range(0, len(self.joystick_table_vals)):
            self.joystick_table_vals[i].setText(55)
    
    def recv_mode(self, mode_msg):
        self.current_mode = mode_msg.mode
    
    def recv_suppress(self, suppress_msg):
        self.suppress_dt = rospy.Time.now() - self.last_suppress_time
        self.last_suppress_time = rospy.Time.now()
    
    def recv_twist(self, twist_msg):
        self.twist_dt = rospy.Time.now() - self.last_twist_time
        self.last_twist_time = rospy.Time.now()
        self.current_cmd = twist_msg
    
    def recv_joystick(self, joystick_msg):
        self.joystick_data = joystick_msg.channels;
        self.last_joystick_time = rospy.Time.now()
    
    def recv_bumpers(self, bumper_msg):
        self.bumper_front_left = bumper_msg.front_left
        self.bumper_front_right = bumper_msg.front_right
        self.bumper_rear_left = bumper_msg.rear_left
        self.bumper_rear_right = bumper_msg.rear_right
    
    def recv_battery(self, battery_msg):
        self.battery_percent = battery_msg.percent
        self.battery_voltage = battery_msg.voltage
    
    def recv_bind_status(self, bind_msg):
        self.joystick_bind_status = bind_msg.data
        
        if bind_msg.data:
            if self.joystick_bind_dot_counter == 0:
                self._widget.joystick_bind_lbl.setText('Binding.')
            elif self.joystick_bind_dot_counter == 1:
                self._widget.joystick_bind_lbl.setText('Binding..')
            elif self.joystick_bind_dot_counter == 2:
                self._widget.joystick_bind_lbl.setText('Binding...')
                
            self.joystick_bind_dot_counter = (self.joystick_bind_dot_counter + 1) % 3

    def recv_gyro_calibrated(self, cal_msg):
        self.gyro_cal_status = cal_msg.data
            
    def recv_imu(self, imu_msg):
        self.gyro_x = imu_msg.angular_velocity.x
        self.gyro_y = imu_msg.angular_velocity.y
        self.gyro_z = imu_msg.angular_velocity.z
    
    def recv_wake_time(self, time_msg):
        self.current_wake_time = time_msg.data
            
    def subscribe_topics(self):
        sub_joystick = rospy.Subscriber('/mobility_base/joystick_raw', JoystickRaw, self.recv_joystick)
        sub_suppress = rospy.Subscriber('/mobility_base/suppress_wireless', std_msgs.msg.Empty, self.recv_suppress)
        sub_twist = rospy.Subscriber('/mobility_base/cmd_vel', Twist, self.recv_twist)
        sub_mode = rospy.Subscriber('/mobility_base/mode', Mode, self.recv_mode)
        sub_bumpers = rospy.Subscriber('/mobility_base/bumper_states', BumperState, self.recv_bumpers)
        sub_battery = rospy.Subscriber('/mobility_base/battery', BatteryState, self.recv_battery)
        sub_bind = rospy.Subscriber('/mobility_base/bind_status', std_msgs.msg.Bool, self.recv_bind_status)
        sub_gyro_calibrated = rospy.Subscriber('/mobility_base/imu/is_calibrated', std_msgs.msg.Bool, self.recv_gyro_calibrated)
        sub_imu = rospy.Subscriber('/mobility_base/imu/data_raw', Imu, self.recv_imu)
        sub_wake_time = rospy.Subscriber('/mobility_base/wake_time', std_msgs.msg.Time, self.recv_wake_time)
    
    def advertise_topics(self):
        self.pub_start_bind = rospy.Publisher('/mobility_base/bind_start', std_msgs.msg.Empty, queue_size=1)
        self.pub_stop_bind = rospy.Publisher('/mobility_base/bind_stop', std_msgs.msg.Empty, queue_size=1)
        self.pub_set_wake_time = rospy.Publisher('/mobility_base/set_wake_time', std_msgs.msg.Time, queue_size=1)
    
    def init_joystick_graphics(self):
        # Pens
        self.cyan_pen = QPen(QColor(0, 255, 255))
        self.magenta_pen = QPen(QColor(255, 0, 255))
        self.red_pen = QPen(QColor(255, 0, 0))
        self.cyan_pen.setWidth(3)
        self.magenta_pen.setWidth(3)
        self.red_pen.setWidth(3)
        self.stick_ind_l = QGraphicsEllipseItem()
        self.stick_ind_r = QGraphicsEllipseItem()
        self.stick_line_l = QGraphicsLineItem()
        self.stick_line_r = QGraphicsLineItem()
        self.mode_ind = QGraphicsLineItem()
        
        # Left joystick indicator circle
        px_l = self.stick_ind_lox - self.stick_ind_radius
        py_l = self.stick_ind_loy - self.stick_ind_radius
        self.stick_ind_l.setRect(px_l, py_l, 2 * self.stick_ind_radius, 2 * self.stick_ind_radius)
        self.stick_ind_l.setBrush(QBrush(QColor(255, 0, 0)))
        self.stick_ind_l.setPen(QPen(QColor(0, 0, 0)))  
        
        # Right joystick indicator circle
        px_r = self.stick_ind_rox - self.stick_ind_radius
        py_r = self.stick_ind_roy - self.stick_ind_radius
        self.stick_ind_r.setRect(px_r, py_r, 2 * self.stick_ind_radius, 2 * self.stick_ind_radius)
        self.stick_ind_r.setBrush(QBrush(QColor(255, 0, 0)))
        self.stick_ind_r.setPen(QPen(QColor(0, 0, 0)))
        
        # Left joystick indicator line
        line_pen = QPen(QColor(255,0,0))
        line_pen.setWidth(4)
        self.stick_line_l.setLine(self.stick_ind_lox, self.stick_ind_loy, self.stick_ind_lox, self.stick_ind_loy)
        self.stick_line_l.setPen(line_pen)
        
        # Right joystick indicator line
        self.stick_line_r.setLine(self.stick_ind_rox, self.stick_ind_roy, self.stick_ind_rox, self.stick_ind_roy)
        self.stick_line_r.setPen(line_pen) 
        
        # Mode indicator line
        self.mode_ind.setLine(self.mode_ind_x1, self.mode_ind_y1, self.mode_ind_x2, self.mode_ind_y2)
        self.mode_ind.setPen(self.cyan_pen)
        
        # Joystick power indicator
        self.joystick_power_ind = []
        self.joystick_power_ind.append(QGraphicsLineItem(self.power_ind_x1, self.power_ind_y + 20, self.power_ind_x1, self.power_ind_y - 20))
        self.joystick_power_ind.append(QGraphicsLineItem(self.power_ind_x1, self.power_ind_y - 20, self.power_ind_x1 + 50, self.power_ind_y - 20))
        self.joystick_power_ind.append(QGraphicsLineItem(self.power_ind_x1+50, self.power_ind_y - 20, self.power_ind_x1+50, self.power_ind_y + 20))
        self.joystick_power_ind.append(QGraphicsLineItem(self.power_ind_x1+50, self.power_ind_y + 20, self.power_ind_x1, self.power_ind_y + 20))
        
        # Populate scene
        graphics_scene = QGraphicsScene()
        graphics_scene.addItem(self.stick_ind_l)
        graphics_scene.addItem(self.stick_ind_r)
        graphics_scene.addItem(self.stick_line_l)
        graphics_scene.addItem(self.stick_line_r)
        graphics_scene.addItem(self.mode_ind)
        for l in self.joystick_power_ind:
            l.setPen(self.red_pen)
            graphics_scene.addItem(l)
        graphics_scene.setSceneRect(0, 0, self._widget.joystickGraphicsView.width() - 4, self._widget.joystickGraphicsView.height() - 4)
        self._widget.joystickGraphicsView.setScene(graphics_scene)
        self._widget.joystickGraphicsView.setBackgroundBrush(QBrush(QImage(os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'images', 'dx6ilabels.jpg'))))
        self._widget.joystickGraphicsView.show()
        
    def init_bumper_graphics(self):
        
        # Pens
        self.blue_pen = QPen(QColor(0,0,255))
        self.blue_pen.setWidth(10)
        
        self.bumper_lines = []

        # Text state labels
        self.bumper_state_labels = [QGraphicsTextItem() for i in range(0,4)]
        for i in range(len(self.bumper_state_labels)):
            self.bumper_state_labels[i].setFont(QFont('Ubuntu', 14, QFont.Bold))
            self.bumper_state_labels[i].setPlainText('00')
        self.bumper_state_labels[0].setPos(self.bumper_fl_x-10, self.bumper_fl_y + 55)
        self.bumper_state_labels[1].setPos(self.bumper_fr_x-10, self.bumper_fr_y + 55)
        self.bumper_state_labels[2].setPos(self.bumper_rl_x-10, self.bumper_rl_y - 80)
        self.bumper_state_labels[3].setPos(self.bumper_rr_x-10, self.bumper_rr_y - 80)
        
        # Bumper indicator lines
        self.bumper_line(self.bumper_fl_x - 20, self.bumper_fl_y - self.bumper_dy, True)
        self.bumper_line(self.bumper_fl_x - self.bumper_dx, self.bumper_fl_y - 20, False)
        self.bumper_line(self.bumper_fl_x + self.bumper_dx, self.bumper_fl_y - 20, False)
        self.bumper_line(self.bumper_fl_x - 20, self.bumper_fl_y + self.bumper_dy, True)
        self.bumper_line(self.bumper_fr_x - 20, self.bumper_fr_y - self.bumper_dy, True)
        self.bumper_line(self.bumper_fr_x - self.bumper_dx, self.bumper_fr_y - 20, False)
        self.bumper_line(self.bumper_fr_x + self.bumper_dx, self.bumper_fr_y - 20, False)
        self.bumper_line(self.bumper_fr_x - 20, self.bumper_fr_y + self.bumper_dy, True)
        self.bumper_line(self.bumper_rl_x - 20, self.bumper_rl_y - self.bumper_dy, True)
        self.bumper_line(self.bumper_rl_x - self.bumper_dx, self.bumper_rl_y - 20, False)
        self.bumper_line(self.bumper_rl_x + self.bumper_dx, self.bumper_rl_y - 20, False)
        self.bumper_line(self.bumper_rl_x - 20, self.bumper_rl_y + self.bumper_dy, True)
        self.bumper_line(self.bumper_rr_x - 20, self.bumper_rr_y - self.bumper_dy, True)
        self.bumper_line(self.bumper_rr_x - self.bumper_dx, self.bumper_rr_y - 20, False)
        self.bumper_line(self.bumper_rr_x + self.bumper_dx, self.bumper_rr_y - 20, False)
        self.bumper_line(self.bumper_rr_x - 20, self.bumper_rr_y + self.bumper_dy, True)
        
        # Populate scene
        graphics_scene = QGraphicsScene()
        for bumper in self.bumper_lines:
            graphics_scene.addItem(bumper)
        for label in self.bumper_state_labels:
            graphics_scene.addItem(label)
        graphics_scene.setSceneRect(0, 0, self._widget.bumperGraphicsView.width() - 4, self._widget.bumperGraphicsView.height() - 4)
        self._widget.bumperGraphicsView.setScene(graphics_scene)
        self._widget.bumperGraphicsView.setBackgroundBrush(QBrush(QImage(os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'images', 'mb_top.png'))))
        self._widget.bumperGraphicsView.show()
    
    def bumper_line(self, x, y, front):
        new_line = QGraphicsLineItem()
        if front:
            new_line.setLine(x, y, x+40, y)
        else:
            new_line.setLine(x, y, x, y+40)
        new_line.setPen(self.blue_pen)
        new_line.setVisible(False)
        self.bumper_lines.append(new_line)
        
    def bumper_visible_switch(self, idx, bumper_state):
        if bumper_state & BumperState.BUMPER_FRONT:
            self.bumper_lines[idx].setVisible(True)
        else:
            self.bumper_lines[idx].setVisible(False)
        if bumper_state & BumperState.BUMPER_LEFT:
            self.bumper_lines[idx+1].setVisible(True)
        else:
            self.bumper_lines[idx+1].setVisible(False)
        if bumper_state & BumperState.BUMPER_RIGHT:
            self.bumper_lines[idx+2].setVisible(True)
        else:
            self.bumper_lines[idx+2].setVisible(False)  
        if bumper_state & BumperState.BUMPER_REAR:
            self.bumper_lines[idx+3].setVisible(True)
        else:
            self.bumper_lines[idx+3].setVisible(False)

    def reset_gui_timer(self):
        self.gui_update_timer = QTimer(self._widget)
        self.gui_update_timer.setInterval(100)
        self.gui_update_timer.setSingleShot(False)
        self.gui_update_timer.timeout.connect(lambda: self.update_gui_cb())
        self.gui_update_timer.start()
        
    def spawn_full_gui(self):
        super(DiagnosticGui, self).__init__(self.context_)
        # Give QObjects reasonable names
        self.setObjectName('DiagnosticGui')

        # Create QWidget
        self._widget = QWidget()

        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'resource', 'DiagnosticGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DiagnosticGui' + str(self.widget_count))
        self.widget_count += 1
        # Add widget to the user interface
        self.context_.add_widget(self._widget)
        
    def spawn_tab_gui(self):
        super(DiagnosticGui, self).__init__(self.context_)
        # Give QObjects reasonable names
        self.setObjectName('DiagnosticGui')

        # Create QWidget
        self._widget = QWidget()
        
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('mobility_base_tools'), 'resource', 'DiagnosticGuiTabs.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('DiagnosticGui' + str(self.widget_count))
        self.widget_count += 1
        # Add widget to the user interface
        self.context_.add_widget(self._widget)
        
    def update_right_stick_indicator(self, lat_val, forward_val):
        horiz_val = -self.stick_ind_range_factor * (lat_val - self.stick_ind_range_mid)
        vert_val = -self.stick_ind_range_factor * (forward_val - self.stick_ind_range_mid)
        r = sqrt(horiz_val * horiz_val + vert_val * vert_val)
        if r > self.stick_ind_range_pix / 2:
            r = self.stick_ind_range_pix / 2
        ang = atan2(vert_val, horiz_val)    
        px = r * cos(ang)
        py = r * sin(ang)
        self.stick_ind_r.setPos(QPoint(px, py))
        self.stick_line_r.setLine(self.stick_ind_rox, self.stick_ind_roy, self.stick_ind_rox + px, self.stick_ind_roy + py)
    
    def update_left_stick_indicator(self, yaw_val, enable_val):
        horiz_val = -self.stick_ind_range_factor * (yaw_val - self.stick_ind_range_mid)
        vert_val = -self.stick_ind_range_factor * (enable_val - self.stick_ind_range_mid)
        r = sqrt(horiz_val * horiz_val + vert_val * vert_val)
        if r > self.stick_ind_range_pix / 2:
            r = self.stick_ind_range_pix / 2
        ang = atan2(vert_val, horiz_val)    
        px = r * cos(ang)
        py = r * sin(ang)
        self.stick_ind_l.setPos(QPoint(px, py))
        self.stick_line_l.setLine(self.stick_ind_lox, self.stick_ind_loy, self.stick_ind_lox + px, self.stick_ind_loy + py)        
    
    def shutdown_plugin(self):
        pass
