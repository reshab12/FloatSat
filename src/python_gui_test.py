#!/bin/python3

import sys
import random
import matplotlib
matplotlib.use('QtAgg')

from PyQt6 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import time
import struct

import rodosmwinterface as rodos
rodos.printTopicInit(enable=True)

import numpy

python2rodos = rodos.Topic(1002)
rodos2python = rodos.Topic(1003)

luart = rodos.LinkinterfaceUART(path="/dev/rfcomm0")
gwUart = rodos.Gateway(luart)



class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)


class PlotWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.number_of_plots = 5
        self.datasize = 33
        self.canvasss = [MplCanvas(self, width=5, height=4, dpi=100) for i in range(self.number_of_plots)]
        self.plot_refs = [[None for i in range(5)] for j in range(self.number_of_plots)]#5=Max number of lines in one plot
        
        self.dataNames = ["Time",                       #0
                          "pose_estimation_mode",       #1
                          "control_mode",               #2
                          "mission_mode",               #3
                          "wx","wy","wz",               #4
                          "mx","my","mz",               #7
                          "ax","ay","az",               #10
                          "headingMagneto",             #13
                          "headingGyro",                #14
                          "heading",                    #15
                          "moving",                     #16
                          "motorSpeed",                 #17
                          "omega_wheel",                #18
                          "control value",              #19
                          "requested_angle",            #20
                          "requested_rot_speed",        #21
                          "user_requested_angle",       #22
                          "user_requested_rot_speed",   #23
                          "error",                      #24
                          "Ierror",                     #25
                          "error_change",               #26
                          "Last_error",                 #27
                          "error",                      #28
                          "Ierror",                     #29
                          "error_change",               #30
                          "Last_error",                 #31
                          "motorCurrent",               #32
                          "magTorquerCurrent",          #33
                          "boardCurrent",               #34
                          "batterieVoltage",            #35
                          "boardVoltage",               #36
                          ""
                          "",
                          ""
                          ]
        self.dataFormat = [
                        #[1,3],
                        [4,3],
                        #[7,3],
                        #[10,3],
                        [13,3],
                        [17,1],
                        [18,1],
                        #[19,1],
                        #[20,4],
                        #[24,4],
                        [28,4],
                        [32,5],
                        [-1,0],
                        [-1,0],
                        [-1,0]
                        ]

        layout = QtWidgets.QVBoxLayout()
        for i in range(self.number_of_plots):
            layout.addWidget(self.canvasss[i])

        layout2 = QtWidgets.QVBoxLayout()

        self.text_box =  [QtWidgets.QPushButton(self.dataNames[j])for j in range(self.datasize)]
        for i in range(self.datasize):
            self.text_box[i].setCheckable(False)
            self.text_box[i].setMaximumWidth(300)
            self.text_box[i].setText(self.dataNames[i])
            layout2.addWidget(self.text_box[i])

        h_layout = QtWidgets.QHBoxLayout()
        h_layout.addLayout(layout2)
        h_layout.addLayout(layout)

        widget = QtWidgets.QWidget()
        widget.setLayout(h_layout)
        self.setCentralWidget(widget)

        self.n_data = 300
        self.xdata = list(range(self.n_data))
        self.ydata = [[0 for i in range(self.n_data)] for j in range(self.datasize)]

        self.show()

        self.counter1 = 0
        self.counter2 = 0

    def update_plot(self,data):
        # Drop off the first y element, append a new one.
        for i in range(self.datasize):
            self.ydata[i] = self.ydata[i][1:] + [data[i]]

        #self.ydata[18][self.n_data-1]=0
        self.counter1 = self.counter1+1 
        self.counter2 = self.counter2+1
        if self.counter1 > 4:
            self.counter1 = 0

            #textbox:
            for i in range(self.datasize):
                self.text_box[i].setText(self.dataNames[i]+": "+"{:.2f}".format(data[i]))
       
            #scaling factors for plots not used (autoscale)
            scaling = [5,20,5,4,200,10,10,10,10,10,10]

            data_line_count = 1
                
            #redraw
            if self.counter2 > 39: 
                self.counter2 = 0
                for i in range(self.number_of_plots):
                    for j in range(self.dataFormat[i][1]):
                        self.plot_refs[i][j].axes.clear()
                        self.plot_refs[i][j] = None
                
            if self.plot_refs[0][0] == None:
                #print("=None")
                for i in range(self.number_of_plots):
                    if self.dataFormat[i][0] >= 0:
                        data_line_count= self.dataFormat[i][0]
                    for j in range(self.dataFormat[i][1]):
                        match j:
                            case 0:
                                temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'r',label=self.dataNames[data_line_count])
                            case 1:
                                temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'g',label=self.dataNames[data_line_count])
                            case 2:
                                temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'b',label=self.dataNames[data_line_count])
                            case 3:
                                temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'c',label=self.dataNames[data_line_count])
                            case 4:
                                temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'm',label=self.dataNames[data_line_count])
                                

                        self.plot_refs[i][j] = temp_plot_refs[0] 
                        
                        self.plot_refs[i][j].axes.autoscale()
                        #self.plot_refs[i][j].axes.set_ylim(-scaling[i], scaling[i]) 

                        data_line_count = data_line_count + 1
                    self.plot_refs[i][0].axes.legend(loc="upper left")
            #update data
            else:
                #print("!=None")
                for i in range(self.number_of_plots):
                    if self.dataFormat[i][0] >= 0:
                        data_line_count= self.dataFormat[i][0]
                    for j in range(self.dataFormat[i][1]):
                        self.plot_refs[i][j].set_ydata(self.ydata[data_line_count])

                        data_line_count = data_line_count + 1

            #update plot
            for i in range(self.number_of_plots):
                self.canvasss[i].draw()
      
      
            
class MainWindow(QtWidgets.QMainWindow):
    
    def __init__(self):
        super().__init__()

        self.setWindowTitle("My App")

        self.selected_command = 0
        self.command_var = 0

    #widgets------------------------------------------------------------------------------
        self.send_button = QtWidgets.QPushButton("Send command")
        self.send_button.setCheckable(True)
        self.send_button.clicked.connect(self.send_button_was_clicked)

        self.command_selection = QtWidgets.QComboBox()
        self.command_selection.addItems(
            ["start mission", 
             "change control mode",
             "change pose estimation mode"])
        self.command_selection.currentIndexChanged.connect( self.command_selection_changed )
     
        self.command_var_selection = QtWidgets.QComboBox()
        self.command_var_selection.addItems(
            ["standby (stops current mission)", 
             "hibernation",
             "star mapper",
             "object detection"]) 
        self.command_var_selection.currentIndexChanged.connect( self.command_var_changed )

        self.abort_mission_button = QtWidgets.QPushButton("Abort Mission")
        self.abort_mission_button.setCheckable(True)
        self.abort_mission_button.clicked.connect(self.abort_mission_button_was_clicked)

        self.reboot_button = QtWidgets.QPushButton("Reboot")
        self.reboot_button.setCheckable(True)
        self.reboot_button.clicked.connect(self.reboot_button_was_clicked)

        self.command_angle_dial = QtWidgets.QDial()
        self.command_angle_dial.setRange(-180, 180)
        self.command_angle_dial.setSingleStep(1)
        #self.command_angle_dial.valueChanged.connect(self.value_changed)
        #self.command_angle_dial.sliderMoved.connect(self.slider_position)
        #self.command_angle_dial.sliderPressed.connect(self.slider_pressed)
        self.command_angle_dial.sliderReleased.connect(self.ang_slider_released)
        

        self.command_speed_slider = QtWidgets.QSlider()
        self.command_speed_slider.setRange(1000,5000)
        self.command_speed_slider.sliderReleased.connect(self.speed_slider_released)

    #layout--------------------------------------------------------------------------------
        layoutH1 = QtWidgets.QHBoxLayout()
        layoutH1.addWidget(self.send_button)
        layoutH1.addWidget(self.command_selection)
        layoutH1.addWidget(self.command_var_selection)

        layoutH2 = QtWidgets.QHBoxLayout()
        layoutH2.addWidget(self.command_angle_dial)
        layoutH2.addWidget(self.command_speed_slider)

        self.layoutV = QtWidgets.QVBoxLayout()
        self.layoutV.addWidget(self.abort_mission_button)
        self.layoutV.addWidget(self.reboot_button)
        self.layoutV.addLayout(layoutH1)
        self.layoutV.addLayout(layoutH2)

        widget = QtWidgets.QWidget()
        widget.setLayout(self.layoutV)
        self.setCentralWidget(widget)

    #functions-----------------------------------------------------------------------------
    def send_button_was_clicked(self):
        print("Sending command:" + str(self.selected_command) + " " + str(self.command_var))
        # Pack sensor data to a struct that RODOS recognizes
        command_struct = struct.pack("B3xi",self.selected_command, self.command_var)
        python2rodos.publish(command_struct)

    def abort_mission_button_was_clicked(self):
        print("Abort Mission")
        # Pack sensor data to a struct that RODOS recognizes
        command_struct = struct.pack("B3xi",0xf0, 0)
        python2rodos.publish(command_struct)

    def reboot_button_was_clicked(self):
        print("reboot")
        # Pack sensor data to a struct that RODOS recognizes
        command_struct = struct.pack("B3xi",0xf1, 0)
        python2rodos.publish(command_struct)
    
    def command_selection_changed(self, i): # i is an int
        #print(i)
        self.selected_command = i
        if self.selected_command == 0:
            #self.command_var_selection =  QtWidgets.QComboBox()
            for i in range(5):
                self.command_var_selection.removeItem(0)
            self.command_var_selection.addItems(
            ["standby", 
             "hibernation",
             "star mapper",
             "object detection"]) 
        elif self.selected_command == 1:
            #self.command_var_selection =  QtWidgets.QComboBox()
            for i in range(5):
                self.command_var_selection.removeItem(0)   
            self.command_var_selection.addItems(
            ["standby", 
             "position control",
             "velocity control",
             "ai control"]) 
        elif self.selected_command == 2:
            #self.command_var_selection =  QtWidgets.QComboBox()
            for i in range(5):
                self.command_var_selection.removeItem(0)   
            self.command_var_selection.addItems(
            ["imu", 
             "mag interference",
             "star mapper"]) 
        else:
            for i in range(5):
                self.command_var_selection.removeItem(0)   
        self.command_var = 0

    def command_var_changed(self, i): # i is an int
        #print(i)
        self.command_var = i

    def ang_slider_released(self):
        print("Move to",self.command_angle_dial.sliderPosition())
        # Pack sensor data to a struct that RODOS recognizes
        command_struct = struct.pack("B3xi",0x03, self.command_angle_dial.sliderPosition())
        python2rodos.publish(command_struct)

    def speed_slider_released(self):
        print("Move to",self.command_speed_slider.sliderPosition())
        # Pack sensor data to a struct that RODOS recognizes
        command_struct = struct.pack("B3xi",0x04, self.command_speed_slider.sliderPosition())
        python2rodos.publish(command_struct)
        
app = QtWidgets.QApplication(sys.argv)

window2 = MainWindow()
window2.show()

window = PlotWindow()
window.show()

# Callback
def topicHandler(data):
  try:
    #unpacked = struct.unpack("=lBBBffff", data)
    unpacked = struct.unpack("=q3Bx9f3fB3xlfl2f2f4f4f5f", data)
    #for i in unpacked:
    #    print(i)
    #print()
    #print("stm sends telemetry: {} {} {}".format(unpacked[4],unpacked[5],unpacked[6]))
    window.update_plot(unpacked)
  except Exception as e:
    print(e)
    print(data)
    print(len(data))


rodos2python.addSubscriber(topicHandler)
gwUart.forwardTopic(python2rodos)

gwUart.run()
app.exec()