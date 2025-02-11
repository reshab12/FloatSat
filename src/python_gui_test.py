#!/bin/python3

import sys
import random
import matplotlib
matplotlib.use('QtAgg')

from PyQt6 import QtCore, QtWidgets, QtGui

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

        self.number_of_plots = 3
        self.datasize = 40
        self.number_of_boxes = 27
         
        self.dataNames = [["Time",True],                       #0
                          ["pose_estimation_mode",True],       #1
                          ["control_mode",True],               #2
                          ["mission_mode",True],               #3
                          ["wx",True],["wy",True],["wz",True],               #4
                          ["mx",True],["my",True],["mz",True],               #7
                          ["ax",False],["ay",False],["az",False],               #10
                          ["headingMagneto",False],             #13
                          ["headingGyro",False],                #14
                          ["heading",True],                    #15
                          ["speed",True],                      #16
                          ["motorSpeed",True],                 #17
                          ["omega_wheel",False],                #18
                          ["control value",False],              #19
                          ["requested_angle",False],            #20
                          ["requested_rot_speed",False],        #21
                          ["user_requested_angle",True],       #22
                          ["user_requested_rot_speed",True],   #23
                          ["error_vel",True],                      #24
                          ["Ierror_vel",True],                     #25
                          ["error_change_vel",True],               #26
                          ["Last_error_vel",False],                 #27
                          ["error_mot",True],                      #28
                          ["Ierror_mot",True],                     #29
                          ["error_change_mot",True],               #30
                          ["Last_error_mot",False],                 #31
                          ["motorCurrent",True],               #32
                          ["magTorquerCurrent",True],          #33
                          ["boardCurrent",True],               #34
                          ["batterieVoltage",True],            #35
                          ["boardVoltage",True],               #36
                          ["increments",False],                 #37
                          ["turnDirection",False],              #38
                          ["raspberry_attitude",True],         #39
                          ["",False],
                          ["",False],
                          ["",False],
                          ["",False],
                          ["",False],
                          ["",False],
                          ["",False],
                          ["",False],
                          ["",False]
                          ]

        self.dataFormat = [
                        [1,3,False,"modes"],
                        [4,3,True,"gyr"],
                        [7,3,False,"mag"],
                        [10,3,False,"acc"],
                        [13,4,True,"heading"],
                        [17,1,False,"motor speed"],
                        [18,1,False,"omega_wheel"],
                        [19,1,False,"control value"],
                        [20,4,False,"requested values"],
                        [24,4,False,"vel errors"],
                        [28,4,True,"mot errors"],
                        [32,5,False,"currents"],
                        [37,1,False,"increments"],
                        [39,1,False,"raspberry_attitude"]
                        ]


        self.createLayout()

        self.createMenu()

        self.n_data = 100
        self.xdata = list(range(self.n_data))
        self.ydata = [[0 for i in range(self.n_data)] for j in range(self.datasize)]

        self.show()

        self.counter1 = 0
        self.counter2 = 0

    def createLayout(self):
        self.layout1 = QtWidgets.QVBoxLayout()
        self.canvasss = [MplCanvas(self, width=5, height=4, dpi=100) for i in range(self.number_of_plots)]
        self.plot_refs = [[None for i in range(5)] for j in range(self.number_of_plots)]#5=Max number of lines in one plot
       
        for i in range(self.number_of_plots):
            self.layout1.addWidget(self.canvasss[i])

        self.layout2 = QtWidgets.QVBoxLayout()

        self.layout3 = QtWidgets.QVBoxLayout()

        self.text_box =  [QtWidgets.QPushButton("")for j in range(self.number_of_boxes)]
        for i in range(self.number_of_boxes):
            self.text_box[i].setCheckable(False)
            self.text_box[i].setMaximumWidth(250)
            self.text_box[i].setText("")
            self.layout2.addWidget(self.text_box[i])
        
        self.data_box =  [QtWidgets.QPushButton("")for j in range(self.number_of_boxes)]
        for i in range(self.number_of_boxes):
            self.data_box[i].setCheckable(False)
            self.data_box[i].setMaximumWidth(100)
            self.data_box[i].setText("")
            self.layout3.addWidget(self.data_box[i])

        self.h_layout = QtWidgets.QHBoxLayout()
        self.h_layout.addLayout(self.layout2)
        self.h_layout.addLayout(self.layout3)
        self.h_layout.addLayout(self.layout1)

        widget = QtWidgets.QWidget()
        widget.setLayout(self.h_layout)
        self.setCentralWidget(widget)

    def removeLayouts(self):
        for i in range(self.number_of_boxes):
            self.layout2.removeWidget(self.text_box[i])
            self.layout3.removeWidget(self.data_box[i])
        self.clearPlotRefs()
        for i in range(self.number_of_plots):
            self.layout1.removeWidget(self.canvasss[i])
        self.h_layout.removeItem(self.layout2)
        self.h_layout.removeItem(self.layout3)
        self.h_layout.removeItem(self.layout1)
        self.takeCentralWidget()

    def createMenu(self):
        menu = self.menuBar()

        file_menu = menu.addMenu("&View")

        self.menuButtons = [QtGui.QAction(self.dataNames[i][0], self) for i in range(self.datasize)]
        for i in range(self.datasize):
            self.menuButtons[i].setCheckable(True)
            if self.dataNames[i][1] == True:
                self.menuButtons[i].setChecked(True)
            self.menuButtons[i].triggered.connect(self.menuButtonsClicked)

        for i in range(self.datasize):  
            file_menu.addAction(self.menuButtons[i])

        file_menu2 = menu.addMenu("&Plots")

        self.menuPlots = []
        j=0
        for i in self.dataFormat:
            self.menuPlots.append(QtGui.QAction(i[3], self))
            self.menuPlots[j].setCheckable(True)
            if i[2] == True:
                self.menuPlots[j].setChecked(True)
            self.menuPlots[j].triggered.connect(self.menuButtonsClicked)
            file_menu2.addAction(self.menuPlots[j])
            j += 1

    def menuButtonsClicked(self):
        self.removeLayouts()
        #get new number of boxes:
        self.number_of_boxes = 0
        for i in range(self.datasize):
            if self.menuButtons[i].isChecked():
                self.dataNames[i][1] = True
                self.number_of_boxes += 1
            else: 
                self.dataNames[i][1] = False
        #print(self.number_of_boxes)

        #get new number of plots:
        j=0
        self.number_of_plots = 0
        for i in self.menuPlots:
            if i.isChecked():
                self.dataFormat[j][2] = True
                self.number_of_plots += 1
            else:
                self.dataFormat[j][2] = False
            j += 1

        self.createLayout()
        j=0
        for i in range(self.number_of_boxes):
            while((self.dataNames[j][1])==False):
                j=j+1
            self.text_box[i].setText(self.dataNames[j][0]+": ")
            self.data_box[i].setText("{:.2f}".format(0))
            j=j+1
    
    def clearPlotRefs(self):
        self.counter2 = 0
        k=0
        for i in self.dataFormat:
            if i[2]:
                for j in range(i[1]):
                    self.plot_refs[k][j].axes.clear()
                    self.plot_refs[k][j] = None
                k += 1


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
            j=0
            for i in range(self.number_of_boxes):
                while((self.dataNames[j][1])==False):
                    j=j+1
                self.text_box[i].setText(self.dataNames[j][0]+": ")
                self.data_box[i].setText("{:.2f}".format(data[j]))
                j=j+1
       
            scaling = [5,20,5,4,200,10,10,10,10,10,10]

            data_line_count = 1
                
            #redraw
            if self.counter2 > 39: 
                self.counter2 = 0
                self.clearPlotRefs()

            if self.number_of_plots > 0:   
                if self.plot_refs[0][0] == None:
                    #print("=None")
                    k=0
                    for i in range(self.number_of_plots):
                        while self.dataFormat[k][2] == False:
                                k += 1
                        data_line_count= self.dataFormat[k][0]
                        for j in range(self.dataFormat[k][1]):
                            match j:
                                case 0:
                                    temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'r',label=self.dataNames[data_line_count][0])
                                case 1:
                                    temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'g',label=self.dataNames[data_line_count][0])
                                case 2:
                                    temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'b',label=self.dataNames[data_line_count][0])
                                case 3:
                                    temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'c',label=self.dataNames[data_line_count][0])
                                case 4:
                                    temp_plot_refs = self.canvasss[i].axes.plot(self.xdata, self.ydata[data_line_count], 'm',label=self.dataNames[data_line_count][0])
                                    

                            self.plot_refs[i][j] = temp_plot_refs[0] 
                            
                            self.plot_refs[i][j].axes.autoscale()
                            
                            data_line_count = data_line_count + 1
                        self.plot_refs[i][0].axes.legend(loc="upper left")
                        k += 1
                #update data
                else:
                    #print("!=None")
                    k=0
                    for i in range(self.number_of_plots):
                        while self.dataFormat[k][2] == False:
                                k += 1
                        data_line_count= self.dataFormat[k][0]
                        for j in range(self.dataFormat[k][1]):
                            self.plot_refs[i][j].set_ydata(self.ydata[data_line_count])

                            data_line_count = data_line_count + 1
                        k += 1

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
        self.command_speed_slider.setRange(-200,200)
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
    unpacked = struct.unpack("=q3Bx9f3fflfl2f2f4f4f5fHi2x", data)
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