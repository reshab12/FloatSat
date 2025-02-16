#!/bin/python3

import sys
import random
import matplotlib
matplotlib.use('QtAgg')

from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6.QtWidgets import QFrame

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import time
import struct


import numpy

import pyqtgraph as pg



class MplCanvas(FigureCanvas):

    def __init__(self, parent=None, width=5, height=4, dpi=100):
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = self.fig.add_subplot(111)
        super().__init__(self.fig)

class QHLine(QFrame):
    def __init__(self):
        super(QHLine, self).__init__()
        self.setFrameShape(QtWidgets.QFrame.Shape.HLine)
        self.setFrameShadow(QtWidgets.QFrame.Shadow.Sunken)


data = [i for i in range(40)]
class PlotWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self.number_of_plots = 0
        self.datasize = 40
        self.number_of_boxes = 0
         
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

        self.n_data = 300
        self.xdata = list(range(self.n_data))
        self.ydata = [[0 for i in range(self.n_data)] for j in range(self.datasize)]

        self.createMenu()   
        self.createLayout()
        self.show()

        self.counter1 = 0
        self.counter2 = 0

        self.count = 0
        self.timer = QtCore.QTimer()
        self.timer.setInterval(10)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def createLayout(self):
        self.layout1 = QtWidgets.QVBoxLayout()
        self.layout2 = QtWidgets.QVBoxLayout()
        self.layout3 = QtWidgets.QVBoxLayout()
        self.layout1.setSpacing(2)

        self.h_layout = QtWidgets.QHBoxLayout()
        self.h_layout.addLayout(self.layout2)
        self.h_layout.addLayout(self.layout3)
        #self.h_layout.addLayout(self.layout1)
        self.h_layout.setSpacing(0)

        self.h2_layout = QtWidgets.QHBoxLayout()
        self.h2_layout.addLayout(self.h_layout)
        self.h2_layout.addLayout(self.layout1)
        self.h2_layout.setSpacing(20)

        #self.h_layout.addWidget(self.plot_graph[0])

        widget = QtWidgets.QWidget()
        widget.setLayout(self.h2_layout)
        self.setCentralWidget(widget)

        self.menuButtonsClicked()


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
            for i in range(self.number_of_boxes*2):
                self.layout2.removeWidget(self.lines[i])
                self.layout3.removeWidget(self.lines[i])
                self.lines[i].close()
            #create new boxes
            self.number_of_boxes = temp_number_of_boxes
            self.text_box =  [QtWidgets.QLabel("")for j in range(self.number_of_boxes)]
            self.data_box =  [QtWidgets.QLabel("")for j in range(self.number_of_boxes)]
            

            j=0
            lastj=0
            self.number_of_lines = 0
            self.lines = [QHLine() for x in range(self.number_of_boxes*2)]
            for i in range(self.number_of_boxes):
                #self.text_box[i].setCheckable(False)
                self.text_box[i].setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter)
                self.text_box[i].setAutoFillBackground(False)
                self.text_box[i].setMaximumWidth(250)

                self.data_box[i].setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignVCenter)
                self.data_box[i].setMaximumWidth(100)
                
                while((self.dataNames[j][1])==False):
                    j += 1
                self.text_box[i].setText(self.dataNames[j][0]+": ")
                self.data_box[i].setText("{:.2f}".format(data[j]))
                for a in self.dataFormat:
                    if  (j >= a[0] and lastj < a[0]):
                        self.layout2.addWidget(self.lines[self.number_of_lines])
                        self.number_of_lines += 1
                        self.layout3.addWidget(self.lines[self.number_of_lines])
                        self.number_of_lines += 1
                        break
                self.layout2.addWidget(self.text_box[i])
                self.layout3.addWidget(self.data_box[i])
                lastj=j
                j += 1
            self.layout2.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignTop)
            self.layout3.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft | QtCore.Qt.AlignmentFlag.AlignTop)

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

    def update_plot(self):
        self.count += 1

        # Drop off the first y element, append a new one.
        for i in range(self.datasize):
            self.ydata[i] = self.ydata[i][1:] + [random.randint(20*self.count, 40*self.count)]

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
        
app = QtWidgets.QApplication(sys.argv)

window = PlotWindow()
window.show()

app.exec()


