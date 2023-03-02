# GUI used to test sensory data of tesla suit
#  ----  Maarten Afschrift ---
import sys
from PyQt5.QtWidgets import (QMainWindow, QApplication,
    QPushButton, QWidget, QAction,
    QTabWidget, QVBoxLayout, QLabel,
    QSlider, QComboBox, QGroupBox, QHBoxLayout, QFormLayout,
    QLineEdit, QCheckBox)
from PyQt5.QtCore import Qt
import pyqtgraph as pg
import numpy as np
from PyQt5 import QtCore
from src.teslasuit_joint_control.control_system import *
from teslasuit_sdk import ts_api

# Creating the main window
class App(QMainWindow):
    def __init__(self):
        super().__init__()
        # connect to the tesla suit
        self.suit = Teslasuit()
        self.suit.connect_suit()
        self.suit.start_mocap_streaming()
        #self.model = SkeletalModel()
        #self.model_updater = SkeletalModelUpdater(self.suit, self.model)
        #self.model_updater.calibrate()

        # control elbow joint here
        #self.controlled_joint = self.model.left_elbow_joint

        # create GUI
        self.title = 'Plot Raw Data TeslaSuit'
        self.setWindowTitle(self.title)
        self.tab_widget = MyTabWidget(self)
        self.setCentralWidget(self.tab_widget)
        self.show()

        # initiate 3 tracking angles
        self.tracking_angle1 = [0, 0, 0]
        self.tracking_angle2 = [0, 0, 0]
        self.tracking_angle3 = [0, 0, 0]

        # initiate one timer that updates all the plots
        self.timer = QtCore.QTimer()
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.view_update)
        self.timer.start()

        #initiate timer that checks for FES control signal
        self.fes_timer = QtCore.QTimer()
        self.fes_timer.setInterval(20)
        self.fes_timer.timeout.connect(self.fes_update)
        self.fes_timer.start()

        # main timer since start
        self.maintimer = QtCore.QElapsedTimer()
        self.maintimer.start()
        self.t0 = self.maintimer.elapsed()

        # get id of electrodes
        self.api = ts_api.TsApi()
        self.device_manager = self.api.get_device_manager()
        self.suit_api = self.device_manager.get_or_wait_last_device_attached()
        self.mapper = self.api.mapper
        self.mapping_handle = self.suit_api.get_mapping()
        layouts = self.mapper.get_layouts(self.mapping_handle)
        for i in range(0, len(layouts)):
            if self.mapper.get_layout_element_type(layouts[i]) == 2 and self.mapper.get_layout_type(layouts[i]) == 1:
                break
        self.bones = self.mapper.get_layout_bones(layouts[i])


    def view_update(self):
        #self.model_updater.update_angles()
        # tracking angle should be based on current index in GUI
        last_imu_data = self.suit.streamer.get_raw_data_on_ready()
        index1 = self.tab_widget.PlotIndex1
        index2 = self.tab_widget.PlotIndex2
        index3 = self.tab_widget.PlotIndex3
        if (self.tab_widget.indexoutput == 0):
            y1 = [last_imu_data[index1].gyro.x,
                         last_imu_data[index1].gyro.y,
                         last_imu_data[index1].gyro.z]
            y2 = [last_imu_data[index2].gyro.x,
                   last_imu_data[index2].gyro.y,
                   last_imu_data[index2].gyro.z]
            y3 = [last_imu_data[index3].gyro.x,
                   last_imu_data[index3].gyro.y,
                   last_imu_data[index3].gyro.z]
        elif (self.tab_widget.indexoutput == 1):
            y1 = [last_imu_data[index1].accel.x,
                  last_imu_data[index1].accel.y,
                  last_imu_data[index1].accel.z]
            y2 = [last_imu_data[index2].accel.x,
                  last_imu_data[index2].accel.y,
                  last_imu_data[index2].accel.z]
            y3 = [last_imu_data[index3].accel.x,
                  last_imu_data[index3].accel.y,
                  last_imu_data[index3].accel.z]
        else:
            y1 = [last_imu_data[index1].linear_accel.x,
                  last_imu_data[index1].linear_accel.y,
                  last_imu_data[index1].linear_accel.z]
            y2 = [last_imu_data[index2].linear_accel.x,
                  last_imu_data[index2].linear_accel.y,
                  last_imu_data[index2].linear_accel.z]
            y3 = [last_imu_data[index3].linear_accel.x,
                  last_imu_data[index3].linear_accel.y,
                  last_imu_data[index3].linear_accel.z]
        #index1 = self.tab_widget.PlotIndex1
        #qd1 = self.suit.get_gyro(index1)
        #index2 = self.tab_widget.PlotIndex2
        #qd2 = self.suit.get_gyro(index2)
        #index3 = self.tab_widget.PlotIndex3
        #qd3 = self.suit.get_gyro(index3)
        self.tracking_angle1 = y1
        self.tracking_angle2 = y2
        self.tracking_angle3 = y3
        self.tab_widget.UpdatePlots(self.tracking_angle1, self.tracking_angle2, self.tracking_angle3)

    def fes_update(self):
        # we want to update the FES when the button is clicked
        #fes_channel1 = self.mapper.get_bone_contents(self.bones[self.tab_widget.FESchannel1_value])[1]
        # this should be done in another loop. this is not very efficient now
        fes_channel1 = [self.mapper.get_bone_contents(self.bones[14])[1]]
        fes_channel2 = [self.mapper.get_bone_contents(self.bones[self.tab_widget.FESchannel2_value])[1]]
        fes_channel3 = [self.mapper.get_bone_contents(self.bones[self.tab_widget.FESchannel3_value])[1]]
        amp = self.tab_widget.slider_fes_amp.value()

        # send an haptic touch (only when button is pushed)
        # current approach is to start a timer when button is pushed. We give a command during first 100ms, but not
        # during the first 1s after start-up
        if (self.tab_widget.checkbox_fes.checkState() == 2):
            if (self.tab_widget.fes_timer1.elapsed() < 100 and  self.maintimer.elapsed()>1000):
                #self.suit.haptic_play_touch(fes_channel1, pw=100, duration=100)
                #self.suit.haptic_play_touch(fes_channel1)
                self.suit.haptic_play_touch(fes_channel1, pw=200, duration=200, ampl=amp)
            if (self.tab_widget.fes_timer2.elapsed() < 100 and  self.maintimer.elapsed()>1000):
                self.suit.haptic_play_touch(fes_channel2, pw=100, duration=100, ampl=amp)
            if (self.tab_widget.fes_timer3.elapsed() < 100 and  self.maintimer.elapsed()>1000):
                self.suit.haptic_play_touch(fes_channel3, pw=100, duration=100, ampl=amp)

# class for a xy plot
class CustomPlot(pg.PlotWidget):
    def __init__(self):
        pg.PlotWidget.__init__(self)
        self.x = list(range(300))  # n time points
        self.y1 = list(0 for i in range(0, 300))
        self.y2 = list(0 for i in range(0, 300))
        self.y3 = list(0 for i in range(0, 300))
        self.PlotValue = [0, 0, 0]
        # plot data: x, y values
        self.pen1 = pg.mkPen(color=(230,10,10))
        self.data_line1 = self.plot(self.x, self.y1, pen=self.pen1)
        self.pen2 = pg.mkPen(color=(10, 230, 10))
        self.data_line2 = self.plot(self.x, self.y2, pen=self.pen2)
        self.pen3 = pg.mkPen(color=(10, 10, 230))
        self.data_line3 = self.plot(self.x, self.y3, pen=self.pen3)
        # time since start
        self.elaptimer = QtCore.QElapsedTimer()
        self.elaptimer.start()

    # updates plot with new input
    def update_plot_data(self):
        self.x = self.x[1:]  # Remove the first x element.
        self.x.append(self.x[-1] + 1)  # Add a new value 1 higher than the last.
        self.y1 = self.y1[1:]  # Remove the first y element
        self.y2 = self.y2[1:]  # Remove the first y element
        self.y3 = self.y3[1:]  # Remove the first y element

        self.y1.append(self.PlotValue[0])  # Add a new random value.
        self.y2.append(self.PlotValue[1])  # Add a new random value.
        self.y3.append(self.PlotValue[2])  # Add a new random value.

        self.data_line1.setData(self.x, self.y1)  # Update the data.
        self.data_line2.setData(self.x, self.y2)  # Update the data.
        self.data_line3.setData(self.x, self.y3)  # Update the data.

    def setPlotValue(self, inputvalues):
        self.PlotValue = inputvalues

# Creating tab widgets
class MyTabWidget(QWidget):
    def __init__(self, parent):
        super(QWidget, self).__init__(parent)

        self.layout = QVBoxLayout(self)

        # Initialize tab screen
        self.tabs = QTabWidget()
        self.tab1 = QWidget()
        self.tab2 = QWidget()
        self.tab3 = QWidget()

        # Add tabs
        self.tabs.addTab(self.tab1, "Raw Data")
        self.tabs.addTab(self.tab2, "Control")
        self.tabs.addTab(self.tab3, "Misc")

        #-----------------------------------
        # create first tab -- Plot Raw data
        #----------------------------------
        self.tab1.layout = QVBoxLayout(self)

        # channel selection box
        box_params = QGroupBox("Select channels")
        box_params_layout = QHBoxLayout(box_params)
        box_params_forms = [QFormLayout(), QFormLayout()]

        nChannels = 30
        str_channels = []
        for i in range(0, nChannels+1):
            str_channels.append(str(i))

        # create two boxes to select a channel
        self.selectchannel1 = QComboBox()
        self.selectchannel1.addItems(str_channels)
        self.PlotIndex1 = 1
        self.selectchannel1.currentIndexChanged.connect(self.box1_index_changed)
        self.selectchannel2 = QComboBox()
        self.selectchannel2.addItems(str_channels)
        self.PlotIndex2 = 2
        self.selectchannel2.currentIndexChanged.connect(self.box2_index_changed)
        self.selectchannel3 = QComboBox()
        self.selectchannel3.addItems(str_channels)
        self.PlotIndex3 = 3
        self.selectchannel3.currentIndexChanged.connect(self.box3_index_changed)

        self.SelectOutput = QComboBox()
        self.SelectOutput.addItems(["Gyroscope", "acceleration", "linear acceleration"])
        self.indexoutput = 0
        self.SelectOutput.currentIndexChanged.connect(self.setindexOutput)
        # add the boxes to the layout
        box_params_forms[0].addRow(QLabel("Select Output"), self.SelectOutput)
        box_params_forms[0].addRow(QLabel("select channel first plot"), self.selectchannel1)
        box_params_forms[0].addRow(QLabel("select channel second plot"), self.selectchannel2)
        box_params_forms[0].addRow(QLabel("select channel third plot"), self.selectchannel3)
        for form_layout in box_params_forms:
            box_params_layout.addLayout(form_layout)
        self.tab1.layout.addWidget(box_params)

        # create three plots
        self.pgcustom1 = CustomPlot()
        self.pgcustom2 = CustomPlot()
        self.pgcustom3 = CustomPlot()

        # add the widgets
        box_datavis = QGroupBox("Data visualisation")
        box_datavis_layout = QHBoxLayout(box_datavis)
        box_datavis_forms = [QFormLayout()]
        box_datavis_forms[0].addRow(QLabel("Fig 1"), self.pgcustom1)
        box_datavis_forms[0].addRow(QLabel("Fig 2"), self.pgcustom2)
        box_datavis_forms[0].addRow(QLabel("Fig 3"), self.pgcustom3)
        for form_layout in box_datavis_forms:
            box_datavis_layout.addLayout(form_layout)
        self.tab1.layout.addWidget(box_datavis)
        self.tab1.setLayout(self.tab1.layout)

        # -----------------------------------
        # create second tab -- Control FES
        # ----------------------------------

        self.tab2.layout = QVBoxLayout(self)

        # channel selection box
        FES_params = QGroupBox("Select channels")
        box_FES_layout = QHBoxLayout(FES_params)
        box_FES_forms = [QFormLayout(), QFormLayout(), QFormLayout()]

        self.FESchannel1 = QComboBox()
        self.FESchannel1.addItems(str_channels)
        self.FESchannel1_value = 0
        self.FESchannel1.currentIndexChanged.connect(self.FESchannel1_changed)

        self.FESchannel2 = QComboBox()
        self.FESchannel2.addItems(str_channels)
        self.FESchannel2_value = 0
        self.FESchannel2.currentIndexChanged.connect(self.FESchannel2_changed)


        self.FESchannel3 = QComboBox()
        self.FESchannel3.addItems(str_channels)
        self.FESchannel3_value = 0
        self.FESchannel3.currentIndexChanged.connect(self.FESchannel3_changed)
        self.fes_timer3 = QtCore.QElapsedTimer()
        self.fes_timer3.start()

        box_FES_forms[0].addRow(QLabel("FES channel 1"), self.FESchannel1)
        box_FES_forms[1].addRow(QLabel("FES channel 2"), self.FESchannel2)
        box_FES_forms[2].addRow(QLabel("FES channel 3"), self.FESchannel3)

        for form_layout in box_FES_forms:
            box_FES_layout.addLayout(form_layout)
        self.tab2.layout.addWidget(FES_params)


        FES_button_params = QGroupBox("Activate FES")
        box_FES_button_layout = QHBoxLayout(FES_button_params)
        box_FES_button_forms = [QFormLayout(), QFormLayout(), QFormLayout()]

        self.FES_button1 = QPushButton(self.tab2)
        self.FES_button1.setText("Pulse - Channel 1")
        self.fes_timer1 = QtCore.QElapsedTimer()
        self.fes_timer1.start()
        self.FES_button1.clicked.connect(self.fes1_activate)

        self.FES_button2 = QPushButton(self.tab2)
        self.FES_button2.setText("Pulse - Channel 2")
        self.fes_timer2 = QtCore.QElapsedTimer()
        self.fes_timer2.start()
        self.FES_button2.clicked.connect(self.fes2_activate)

        self.FES_button3 = QPushButton(self.tab2)
        self.FES_button3.setText("Pulse - Channel 3")
        self.fes_timer3 = QtCore.QElapsedTimer()
        self.fes_timer3.start()
        self.FES_button3.clicked.connect(self.fes3_activate)

        self.checkbox_fes = QCheckBox(self.tab2)
        self.checkbox_fes.setCheckState(False)

        self.slider_fes_amp = QSlider(Qt.Horizontal)
        self.slider_fes_amp.setMinimum(0)
        self.slider_fes_amp.setMaximum(800)
        self.slider_fes_amp.setValue(200)
        self.slider_fes_amp.setTickPosition(QSlider.TicksBelow)
        self.slider_fes_amp.setTickInterval(50)


        box_FES_button_forms[0].addRow(QLabel("Enable FES"), self.checkbox_fes)
        box_FES_button_forms[0].addRow(QLabel("Amplitude"), self.slider_fes_amp)
        box_FES_button_forms[0].addRow(QLabel("Channel 1"), self.FES_button1)
        box_FES_button_forms[0].addRow(QLabel("Channel 2"), self.FES_button2)
        box_FES_button_forms[0].addRow(QLabel("Channel 3"), self.FES_button3)



        for form_layout in box_FES_button_forms:
            box_FES_button_layout.addLayout(form_layout)
        self.tab2.layout.addWidget(FES_button_params)
        self.tab2.setLayout(self.tab2.layout)

        # -----------------------------------
        # create third tab -- Misc
        # ----------------------------------



        # Add tabs to widget
        self.layout.addWidget(self.tabs)
        self.setLayout(self.layout)

    # callback functions for each widget
    def box1_index_changed(self, i):  # i is an int
        self.PlotIndex1 = i
    def box2_index_changed(self, i):  # i is an int
        self.PlotIndex2 = i
    def box3_index_changed(self, i):  # i is an int
        self.PlotIndex3 = i
    def setindexOutput(self, i):  # i is an int
        self.indexoutput = i
    def FESchannel1_changed(self,i):
        self.FESchannel1_value = i
    def FESchannel2_changed(self,i):
        self.FESchannel2_value = i
    def FESchannel3_changed(self,i):
        self.FESchannel3_value = i
    def fes1_activate(self):
        print('start channel 1')
        self.fes_timer1.restart()
    def fes2_activate(self):
        print('start channel 2')
        self.fes_timer2.restart()
    def fes3_activate(self):
        print('start channel 3')
        self.fes_timer3.restart()

    # update function for the 2D real-time plots
    def UpdatePlots(self, yval1, yval2, yval3):
        #plotmat1 = [yval1, 0.8 * yval1, 0.6 * yval1]
        #plotmat2 = [yval2, 0.8 * yval2, 0.6 * yval2]
        #plotmat3 = [yval3, 0.8 * yval3, 0.6 * yval3]
        #update plot value
        self.pgcustom1.setPlotValue(yval1)
        self.pgcustom2.setPlotValue(yval2)
        self.pgcustom3.setPlotValue(yval3)
        #update plot data
        self.pgcustom1.update_plot_data()
        self.pgcustom2.update_plot_data()
        self.pgcustom3.update_plot_data()


# start main system
if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = App()
    sys.exit(app.exec_())


