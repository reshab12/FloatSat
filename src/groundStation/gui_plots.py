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

        self.number_of_plots = 6
        self.datasize = 45
        self.number_of_boxes = 17
         
        self.dataNames = [["Time",True],                       #0
                          ["pose_estimation_mode",True],       #1
                          ["control_mode",True],               #2
                          ["mission_mode",True],               #3
                          ["wx",False],["wy",False],["wz",False],               #4
                          ["mx",False],["my",False],["mz",False],               #7
                          ["ax",False],["ay",False],["az",False],               #10
                          ["headingMagneto",False],             #13
                          ["headingGyro",False],                #14
                          ["heading",True],                    #15
                          ["speed",True],                      #16
                          ["motorSpeed",True],                 #17
                          ["omega_wheel",False],                #18
                          ["control value",False],              #19
                          ["requested_angle",True],            #20
                          ["requested_rot_speed",True],        #21
                          ["user_requested_angle",True],       #22
                          ["user_requested_rot_speed",True],   #23
                          ["error_vel",False],                      #24
                          ["Ierror_vel",False],                     #25
                          ["error_change_vel",False],               #26
                          ["Last_error_vel",False],                 #27
                          ["error_mot",False],                      #28
                          ["Ierror_mot",False],                     #29
                          ["error_change_mot",False],               #30
                          ["Last_error_mot",False],                 #31
                          ["motorCurrent",True],               #32
                          ["magTorquerCurrent",True],          #33
                          ["boardCurrent",True],               #34
                          ["batterieVoltage",True],            #35
                          ["boardVoltage",True],               #36
                          ["increments",False],                 #37
                          ["turnDirection",False],              #38
                          ["raspberry_attitude",True],         #39
                          ["error_pos",False],                      #40
                          ["Ierror_pos",False],                     #41
                          ["error_change_pos",False],               #42
                          ["Last_error_pos",False],                 #43
                          ["torque",False],
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
                        [17,1,True,"motor speed"],
                        [18,1,False,"omega_wheel"],
                        [19,1,False,"control value"],
                        [20,4,False,"requested values"],
                        [40,4,True,"pos errors"],
                        [24,4,True,"vel errors"],
                        [28,4,True,"mot errors"],
                        [32,5,False,"currents"],
                        [37,1,False,"increments"],
                        [39,1,False,"raspberry_attitude"],
                        [44,1,False,"torque"]
                        ]


        self.createLayout()

        self.createMenu()

        self.n_data = 300
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

        #commands

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
             "change pose estimation mode",
             "set pos",
             "set vel"])
        self.command_selection.currentIndexChanged.connect( self.command_selection_changed )
     
        self.command_var_selection = QtWidgets.QComboBox()
        self.command_var_selection.addItems(
            ["standby (stops current mission)",
             "mmag torquers",
             "star mapper",
             "object detection"]) 
        self.command_var_selection.currentIndexChanged.connect( self.command_var_changed )

        self.command_angle_selection = QtWidgets.QSpinBox()
        self.command_angle_selection.setMinimum(-180)
        self.command_angle_selection.setMaximum(180)
        self.command_angle_selection.setSingleStep(1) 
        self.command_angle_selection.valueChanged.connect(self.command_angle_selection_value_changed)

        self.abort_mission_button = QtWidgets.QPushButton("Abort Mission")
        self.abort_mission_button.setCheckable(True)
        self.abort_mission_button.clicked.connect(self.abort_mission_button_was_clicked)

        self.reboot_button = QtWidgets.QPushButton("Reboot")
        self.reboot_button.setCheckable(True)
        self.reboot_button.clicked.connect(self.reboot_button_was_clicked)


        #layout--------------------------------------------------------------------------------
        self.layoutH1 = QtWidgets.QHBoxLayout()
        self.layoutH1.addWidget(self.send_button)
        self.layoutH1.addWidget(self.command_selection)
        self.layoutH1.addWidget(self.command_var_selection)

        self.layoutV = QtWidgets.QVBoxLayout()
        self.layoutV.addWidget(self.abort_mission_button)
        self.layoutV.addWidget(self.reboot_button)
        self.layoutV.addLayout(self.layoutH1)
        #self.layoutV.addLayout(self.layoutH2)

        self.v_layout = QtWidgets.QVBoxLayout()
        self.v_layout.addLayout(self.layoutV)
        self.v_layout.addLayout(self.h_layout)

        widget = QtWidgets.QWidget()
        widget.setLayout(self.v_layout)
        self.setCentralWidget(widget)

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
    
    def clearPlotRefs(self):
        self.counter2 = 0
        k=0
        for i in self.dataFormat:
            if i[2]:
                for j in range(i[1]):
                    if self.plot_refs[k][j] != None:
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
    
    #connected functions-----------------------------------------------------------------------------    
    def menuButtonsClicked(self):
        #get new number of boxes:
        temp_number_of_boxes = 0
        for i in range(self.datasize):
            if self.menuButtons[i].isChecked():
                self.dataNames[i][1] = True
                temp_number_of_boxes += 1
            else: 
                self.dataNames[i][1] = False
        if(temp_number_of_boxes != self.number_of_boxes):
            #delete old boxes
            for i in range(self.number_of_boxes):
                self.layout2.removeWidget(self.text_box[i])
                self.layout3.removeWidget(self.data_box[i])
                self.data_box[i].close()
                self.text_box[i].close()
            #create new boxes
            self.number_of_boxes = temp_number_of_boxes
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

        #get new number of plots:
        j=0
        temp_number_of_plots = 0
        for i in self.menuPlots:
            if i.isChecked():
                self.dataFormat[j][2] = True
                temp_number_of_plots += 1
            else:
                self.dataFormat[j][2] = False
            j += 1
        if self.number_of_plots != temp_number_of_plots:
            for i in range(self.number_of_plots):
                self.layout1.removeWidget(self.canvasss[i])
                self.canvasss[i].close()
            self.number_of_plots = temp_number_of_plots
            self.canvasss = [MplCanvas(self, width=5, height=4, dpi=100) for i in range(self.number_of_plots)]
            self.plot_refs = [[None for i in range(5)] for j in range(self.number_of_plots)]#5=Max number of lines in one plot
            for i in range(self.number_of_plots):
                self.layout1.addWidget(self.canvasss[i])

        j=0
        for i in range(self.number_of_boxes):
            while((self.dataNames[j][1])==False):
                j=j+1
            self.text_box[i].setText(self.dataNames[j][0]+": ")
            self.data_box[i].setText("{:.2f}".format(0))
            j=j+1
    
    def command_angle_selection_value_changed(self, i):
        self.command_var = i
    
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
    
    def command_selection_changed(self, i): # updatelayoutH1
        #print(i)
        self.selected_command = i

        if self.selected_command == 0:
            self.layoutH1.replaceWidget(self.command_angle_selection,self.command_var_selection)
            
            self.command_angle_selection.close()
            self.command_angle_selection = QtWidgets.QSpinBox()
            self.command_angle_selection.setSingleStep(1) 
            self.command_angle_selection.valueChanged.connect(self.command_angle_selection_value_changed)

            #self.command_var_selection =  QtWidgets.QComboBox()
            for i in range(5):
                self.command_var_selection.removeItem(0)
            self.command_var_selection.addItems(
            ["standby (stops current mission)", 
             "mission_mode_mag_torquers",
             "star mapper",
             "object detection"]) 
        elif self.selected_command == 1:
            self.layoutH1.replaceWidget(self.command_angle_selection,self.command_var_selection)
           
            self.command_angle_selection.close()
            self.command_angle_selection = QtWidgets.QSpinBox()
            self.command_angle_selection.setSingleStep(1) 
            self.command_angle_selection.valueChanged.connect(self.command_angle_selection_value_changed)

            #self.command_var_selection =  QtWidgets.QComboBox()
            for i in range(5):
                self.command_var_selection.removeItem(0)   
            self.command_var_selection.addItems(
            ["standby", 
             "position control",
             "velocity control",
             "ai control",
             "stop wheel"]) 
        elif self.selected_command == 2:
            self.layoutH1.replaceWidget(self.command_angle_selection,self.command_var_selection)
            
            self.command_angle_selection.close()
            self.command_angle_selection = QtWidgets.QSpinBox()
            self.command_angle_selection.setSingleStep(1) 
            self.command_angle_selection.valueChanged.connect(self.command_angle_selection_value_changed)

            #self.command_var_selection =  QtWidgets.QComboBox()
            for i in range(5):
                self.command_var_selection.removeItem(0)   
            self.command_var_selection.addItems(
            ["imu", 
             "mag interference",
             "star mapper"]) 
        elif self.selected_command == 3:
            self.layoutH1.replaceWidget(self.command_var_selection,self.command_angle_selection)

            self.command_var_selection.close()
            self.command_var_selection = QtWidgets.QComboBox()
            self.command_var_selection.currentIndexChanged.connect( self.command_var_changed )

            self.command_angle_selection.setMinimum(-180)
            self.command_angle_selection.setMaximum(180)
            #self.layoutH1.update()
            #self.layoutV.update()
        else :
            self.layoutH1.replaceWidget(self.command_var_selection,self.command_angle_selection)
            
            self.command_var_selection.close()
            self.command_var_selection = QtWidgets.QComboBox()
            self.command_var_selection.currentIndexChanged.connect( self.command_var_changed )

            self.command_angle_selection.setMinimum(-50)
            self.command_angle_selection.setMaximum(50)
            #self.layoutH1.update()
            #self.layoutV.update()
        self.command_var = 0

    def command_var_changed(self, i): # i is an int
        #print(i)
        self.command_var = i
    
app = QtWidgets.QApplication(sys.argv)


window = PlotWindow()
window.show()

# Callback
def topicHandler(data):
  try:
    #unpacked = struct.unpack("=lBBBffff", data)
    unpacked = struct.unpack("=q3Bx9f3fflfl2f2f4f4f5fHhf4ff4x", data)
    #for i in unpacked:
    #    print(i)
    #print()
    #print("stm sends telemetry: {} {} {}".format(unpacked[4],unpacked[5],unpacked[6]))
    window.update_plot(unpacked)
  except Exception as e:
    print(e)
    #print(data)
    #print(len(data))


rodos2python.addSubscriber(topicHandler)
gwUart.forwardTopic(python2rodos)

gwUart.run()
app.exec()