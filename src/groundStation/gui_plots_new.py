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

import pyqtgraph as pg

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

        self.number_of_plots = 0
        self.datasize = 45
        self.number_of_boxes = 0
         

        # [name,bool_show_in_box]
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

        # [data_start, data_length, bool_show_plot, plot_name]
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

        self.n_data = 300
        self.xdata = list(range(self.n_data))
        self.ydata = [[0 for i in range(self.n_data)] for j in range(self.datasize)]

        self.createMenu()
        self.createLayout()

        self.show()

        self.counter1 = 0
        self.counter2 = 0

    def createLayout(self):
        self.layout1 = QtWidgets.QVBoxLayout()
        self.layout2 = QtWidgets.QVBoxLayout()
        self.layout3 = QtWidgets.QVBoxLayout()

        self.h_layout = QtWidgets.QHBoxLayout()
        self.h_layout.addLayout(self.layout2)
        self.h_layout.addLayout(self.layout3)
        self.h_layout.addLayout(self.layout1)

        self.menuButtonsClicked()

        #commands
        self.selected_command = 0
        self.command_var = 0

        #widgets------------------------------------------------------------------------------
        self.send_button = QtWidgets.QPushButton("Send command")
        self.send_button.setCheckable(False)
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
        self.abort_mission_button.setCheckable(False)
        self.abort_mission_button.clicked.connect(self.abort_mission_button_was_clicked)

        self.reboot_button = QtWidgets.QPushButton("Reboot")
        self.reboot_button.setCheckable(False)
        self.reboot_button.clicked.connect(self.reboot_button_was_clicked)

        self.safetyPin_button = QtWidgets.QPushButton("safetyPin")
        self.safetyPin_button.setCheckable(False)
        self.safetyPin_button.clicked.connect(self.safetyPin_button_was_clicked)


        #commands layout--------------------------------------------------------------------------------
        self.layoutH1 = QtWidgets.QHBoxLayout()
        self.layoutH1.addWidget(self.send_button)
        self.layoutH1.addWidget(self.command_selection)
        self.layoutH1.addWidget(self.command_var_selection)

        self.layoutH2 = QtWidgets.QHBoxLayout()
        self.layoutH2.addWidget(self.abort_mission_button)
        self.layoutH2.addWidget(self.reboot_button)
        self.layoutH2.addWidget(self.safetyPin_button)

        self.layoutV = QtWidgets.QVBoxLayout()
        self.layoutV.addLayout(self.layoutH2)
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
    
    def create_graphs(self):
        self.line = [[None for i in range(5)] for j in range(self.number_of_plots)]
        self.plot_graph = [pg.PlotWidget() for j in range(self.number_of_plots)]

        i = 0
        for format in self.dataFormat:
            if format[2]:
                self.plot_graph[i].setTitle(format[3], color="w", size="20pt")
                styles = {"color": "white", "font-size": "18px"}
                #self.plot_graph[i].setLabel("left", "Temperature (°C)", **styles)
                self.plot_graph[i].setLabel("bottom", "Time [1/10 s]", **styles)
                self.plot_graph[i].addLegend(brush=pg.mkBrush(0, 0, 0, 200),pen=pg.mkPen(255,255,255,200))
                #self.plot_graph.showGrid(x=True, y=True)
                self.plot_graph[i].enableAutoRange(axis='y')
                
                data_line_count = format[0]
                for k in range(format[1]):
                    match k:
                        case 0:
                            pen = pg.mkPen(color="white")
                        case 1:
                            pen = pg.mkPen(color="blue")
                        case 2:
                            pen = pg.mkPen(color="green")
                        case 3:
                            pen = pg.mkPen(color="red")
                        case 4:
                            pen = pg.mkPen(color="orange")

                    self.line[i][k] = self.plot_graph[i].plot(
                        self.xdata,
                        self.ydata[data_line_count + k],
                        name=self.dataNames[data_line_count + k][0],
                        pen=pen
                    )
                #print("create line:" +str(i)+" "+str(k) + " at " + str(data_line_count))
                i += 1

    def create_graphs(self):
        self.line = [[None for i in range(5)] for j in range(self.number_of_plots)]
        self.plot_graph = [pg.PlotWidget() for j in range(self.number_of_plots)]

        i = 0
        for format in self.dataFormat:
            if format[2]:
                self.plot_graph[i].setTitle(format[3], color="w", size="20pt")
                styles = {"color": "white", "font-size": "18px"}
                #self.plot_graph[i].setLabel("left", "Temperature (°C)", **styles)
                self.plot_graph[i].setLabel("bottom", "Time [1/10 s]", **styles)
                self.plot_graph[i].addLegend(brush=pg.mkBrush(0, 0, 0, 200),pen=pg.mkPen(255,255,255,200))
                #self.plot_graph.showGrid(x=True, y=True)
                self.plot_graph[i].enableAutoRange(axis='y')
                
                data_line_count = format[0]
                for k in range(format[1]):
                    match k:
                        case 0:
                            pen = pg.mkPen(color="white")
                        case 1:
                            pen = pg.mkPen(color="blue")
                        case 2:
                            pen = pg.mkPen(color="green")
                        case 3:
                            pen = pg.mkPen(color="red")
                        case 4:
                            pen = pg.mkPen(color="orange")

                    self.line[i][k] = self.plot_graph[i].plot(
                        self.xdata,
                        self.ydata[data_line_count + k],
                        name=self.dataNames[data_line_count + k][0],
                        pen=pen
                    )
                #print("create line:" +str(i)+" "+str(k) + " at " + str(data_line_count))
                i += 1

    def update_plot(self,data):

        # Drop off the first y element, append a new one.
        for i in range(self.datasize):
            self.ydata[i] = self.ydata[i][1:] + [data[i]]

        #update plots
        j=0
        for i in range(self.number_of_plots):
            while((self.dataFormat[j][2])==False):
                j += 1
            data_line_count = self.dataFormat[j][0]
            for k in range(self.dataFormat[j][1]):
                #print("i: " + str(i) + " j: " + str(j) )
                self.line[i][k].setData(self.xdata, self.ydata[data_line_count+k])
            j += 1

        #update textbox
        j=0
        for i in range(self.number_of_boxes):
            while((self.dataNames[j][1])==False):
                j += 1
            self.text_box[i].setText(self.dataNames[j][0]+": ")
            self.data_box[i].setText("{:.2f}".format(data[j]))
            j += 1
    
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
            j=0
            for i in range(self.number_of_boxes):
                while((self.dataNames[j][1])==False):
                    j=j+1
                self.text_box[i].setText(self.dataNames[j][0]+": ")
                self.data_box[i].setText("{:.2f}".format(0))
                j=j+1

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
                self.layout1.removeWidget(self.plot_graph[i])
                self.plot_graph[i].close()
            self.number_of_plots = temp_number_of_plots
            
            self.create_graphs()


            for i in range(self.number_of_plots):
                self.layout1.addWidget(self.plot_graph[i])
    
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

    def safetyPin_button_was_clicked(self):
        print("activate safetyPin")
        # Pack sensor data to a struct that RODOS recognizes
        command_struct = struct.pack("B3xi",0xf2, 0)
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