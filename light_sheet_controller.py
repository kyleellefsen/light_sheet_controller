# -*- coding: utf-8 -*-
"""
Created on Fri Jun 20 19:06:55 2014
@author: Kyle Ellefsen


Make sure that the National Instruments DAC USB-6001 is "Dev1" with the National Instruments Measurement and Automation tool.  

To Use:
Run this program with python.  

"""

import sys, os, time
import numpy as np
try:
    os.chdir(os.path.split(os.path.realpath(__file__))[0])
except NameError:
    pass
import PyDAQmx
from PyDAQmx.DAQmxCallBack import *
from PyDAQmx_helper import getNIDevInfo
from calc_waveforms import calcPiezoWaveform, calcCameraTTL, calcDitherWaveform, calc_ttl
from qtpy import QtCore
from qtpy import QtWidgets
from qtpy.QtCore import Signal, Slot
import pickle
from os.path import expanduser
import pyqtgraph as pg
from collections import OrderedDict
from ctypes import byref, c_int32


def warning(text):
    msgBox = QtWidgets.QMessageBox()
    msgBox.setText(str(text))
    msgBox.addButton(QtWidgets.QMessageBox.Ok)
    msgBox.setDefaultButton(QtWidgets.QMessageBox.Ok)
    ret = msgBox.exec_()
    return ret

dac_present = False
def check_if_NI_devs_are_present():
    global dac_present
    NIDevInfo = getNIDevInfo()
    if 'Dev1' not in NIDevInfo.keys(): # and 'Dev2' not in NIDevInfo.keys():
        warning('Neither National Instruments Dev1 nor Dev2 can be detected. Open NI MAX and make sure the devices are present. Continuing in test mode.')
    elif 'Dev1' not in NIDevInfo.keys():
        warning('National Instruments Dev1 cannot be detected. Open NI MAX and make sure the device is present. Continuing in test mode.')
    #elif 'Dev2' not in NIDevInfo.keys():
    #    warning('National Instruments Dev2 cannot be detected. Open NI MAX and make sure the device is present. Continuing in test mode.')
    elif NIDevInfo['Dev1']['product_type'] != 'USB-6211':
        print('')
        warning('National Instruments Dev1 is not USB-6211. Continuing in test mode.')
    #elif NIDevInfo['Dev2']['product_type'] != 'USB-6001':
    #    warning('National Instruments Dev2 is not USB-6001. Continuing in test mode.')
    else:
        dac_present=True
    return dac_present

def ipython_qt_event_loop_setup():
    try:
        __IPYTHON__
    except NameError:
        return #  If __IPYTHON__ is not defined, we are not in ipython
    else:
        print("Starting flika inside IPython")
        from IPython import get_ipython
        ipython = get_ipython()
        ipython.magic("gui qt")

class Settings:
    ''' This class saves all the settings as you adjust them.  This way, when you close the program and reopen it, all your settings will automatically load as they were just after the last adjustement'''
    def __init__(self):
        self.i = 0
        self.config_file = os.path.join(expanduser("~"),'.lightsheet','config.p')
        try:
            self.d = pickle.load(open(self.config_file, "rb" ))
        except IOError:
            a=dict()
            a['sample_rate']=5000 # Maximum for the NI USB-6001 is 5kS/s/ch
            a['mV_per_pixel'] = 1
            a['maximum_displacement'] = 1
            a['nSteps'] = 5
            a['step_duration'] = 10
            a['flyback_duration'] = 20
            a['total_cycle_period'] = .2
            a['dither_amp'] = 1
            a['triangle_scan'] = False
            a['offset'] = 0
            a['alternate_lasers'] = False
            a['blue_laser_on'] = False
            self.d = [a, a.copy(), a.copy(), a.copy()]

    def __getitem__(self, item):
        try:
            val = self.d[self.i][item]
        except KeyError:
            val = 1
            self.d[self.i][item] = val
        return val

    def __setitem__(self, key, item):
        self.d[self.i][key] = item

    def save(self):
        """save to a config file."""
        if not os.path.exists(os.path.dirname(self.config_file)):
            os.makedirs(os.path.dirname(self.config_file))
        pickle.dump(self.d, open(self.config_file, "wb"))

    def keys(self):
        return self.d[self.i].keys()




        
class LightSheetDriver(QtWidgets.QWidget):
    ''' This class sends creates the signal which will control the piezo, and sends it to the DAQ.'''
    def __init__(self,settings):
        QtWidgets.QWidget.__init__(self)
        self.data = None
        self.data2 = None
        self.settings = settings
        self.sample_rate = settings['sample_rate']
        self.sampsPerPeriod = 1  # samples per channel
        self.calculate()
        self.read = c_int32()
        self.digital_read = c_int32()
        self.createTask()
        self.hide()

    def createTask(self):
        self.analog_output = PyDAQmx.Task()
        self.analog_output.CreateAOVoltageChan("Dev1/ao0", "", -10.0, 10.0, PyDAQmx.DAQmx_Val_Volts, None) #  This is the piezo channel
        self.analog_output.CreateAOVoltageChan("Dev1/ao1", "", -10.0, 10.0, PyDAQmx.DAQmx_Val_Volts, None) #  This is the camera ttl channel
        self.analog_output.CfgSampClkTiming("",self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_ContSamps,self.sampsPerPeriod) #  CfgSampClkTiming(source, rate, activeEdge, sampleMode, sampsPerChan)
        self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1, PyDAQmx.DAQmx_Val_GroupByChannel,self.data, byref(self.read), None) #  WriteAnalogF64(numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)

        #self.analog_output2 = PyDAQmx.Task()
        #self.analog_output2.CreateAOVoltageChan("Dev2/ao0", "", -10.0, 10.0, PyDAQmx.DAQmx_Val_Volts, None) #  This is the dither
        #self.analog_output2.CreateAOVoltageChan("Dev2/ao1", "", -10.0, 10.0, PyDAQmx.DAQmx_Val_Volts, None)  # This is the ttl channel
        #self.analog_output2.CfgSampClkTiming("",self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_ContSamps,self.sampsPerPeriod)
        #self.analog_output2.WriteAnalogF64(self.sampsPerPeriod,0,-1, PyDAQmx.DAQmx_Val_GroupByChannel,self.data2, byref(self.read), None)
        self.analog_output.StartTask()
        #self.analog_output2.StartTask()
        self.stopped = False
        self.acquiring = False

    def calculate(self):
        _, piezoWaveform = calcPiezoWaveform(self.settings)
        _, cameraTTL = calcCameraTTL(self.settings)
        _, ditherWaveform = calcDitherWaveform(self.settings)
        _, ttlWaveform = calc_ttl(self.settings)
        self.data = np.concatenate((piezoWaveform, cameraTTL))
        self.data2 = np.concatenate((ditherWaveform, ttlWaveform))
        self.sampsPerPeriod=len(piezoWaveform)

    def startstop(self):
        if self.stopped:
            self.calculate()
            self.analog_output.CfgSampClkTiming( "", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_ContSamps, self.sampsPerPeriod)
            #self.analog_output2.CfgSampClkTiming("", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_ContSamps, self.sampsPerPeriod)
            self.analog_output.WriteAnalogF64( self.sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data,  byref(self.read), None)
            #self.analog_output2.WriteAnalogF64(self.sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data2, byref(self.read), None)
            self.analog_output.StartTask()
            #self.analog_output2.StartTask()
            self.stopped = False
        else:
            self.analog_output.StopTask()
            #self.analog_output2.StopTask()
            self.dither_gotozero()
            self.stopped = True

    def refresh(self):
        if self.stopped is False:
            self.calculate()
            self.analog_output.StopTask()
            #self.analog_output2.StopTask()
            self.analog_output.CfgSampClkTiming( "", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_ContSamps, self.sampsPerPeriod)
            #self.analog_output2.CfgSampClkTiming("", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_ContSamps, self.sampsPerPeriod)
            self.analog_output.WriteAnalogF64( self.sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data,  byref(self.read), None)
            #self.analog_output2.WriteAnalogF64(self.sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data2, byref(self.read), None)
            #self.digital_output.StopTask()
            #self.digital_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.sampsPerPeriod)
            #self.digital_output.WriteDigitalU8(self.sampsPerPeriod, 0, -1, DAQmx_Val_GroupByChannel,self.digital_data, byref(self.digital_read) ,None) #https://www.quark.kj.yamagata-u.ac.jp/~miyachi/nidaqmxbase-3.4.0/documentation/docsource/daqmxbasecfunc.chm/DAQmxWriteDigitalU8.html
            self.analog_output.StartTask()
            #self.analog_output2.StartTask()

    def dither_gotozero(self):
        s = self.settings
        t = np.arange(0, .01, 1 / s['sample_rate'])
        sampsPerPeriod = len(t)
        ditherWaveform = np.zeros(len(t))
        _, ttlWaveform = calc_ttl(self.settings)
        self.data2 = np.concatenate([ditherWaveform, ttlWaveform])
        #self.analog_output2.StopTask()
        #self.analog_output2.CfgSampClkTiming("", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_FiniteSamps,sampsPerPeriod)
        #self.analog_output2.WriteAnalogF64(sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data2, byref(self.read), None)
        #self.analog_output2.StartTask()
        time.sleep(.02)
        #self.analog_output2.StopTask()

    def gotozero(self):
        s = self.settings
        t = np.arange(0, .1, 1 / s['sample_rate'])
        piezoWaveform=np.zeros(len(t))
        cameraTTL=np.zeros(len(t))
        self.data=np.concatenate((piezoWaveform,cameraTTL))
        self.sampsPerPeriod=len(piezoWaveform)
        self.analog_output.StopTask()
        self.analog_output.CfgSampClkTiming("",self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_FiniteSamps,self.sampsPerPeriod)
        self.analog_output.WriteAnalogF64(self.sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data, byref(self.read), None)
        self.analog_output.StartTask()
        time.sleep(.2)
        self.analog_output.StopTask()
        self.dither_gotozero()
        self.stopped=True

    def gotoend(self):
        s = self.settings
        t = np.arange(0, .1, 1 / s['sample_rate'])
        maxV=s['maximum_displacement']*s['mV_per_pixel']/1000
        piezoWaveform=np.ones(len(t)) * maxV
        cameraTTL=np.zeros(len(t))
        self.data=np.concatenate((piezoWaveform,cameraTTL))
        self.sampsPerPeriod=len(piezoWaveform)
        self.analog_output.StopTask()
        self.analog_output.CfgSampClkTiming("",self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_FiniteSamps,self.sampsPerPeriod)
        self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1, PyDAQmx.DAQmx_Val_GroupByChannel,self.data,byref(self.read),None)
        self.analog_output.StartTask()
        time.sleep(.2)
        self.analog_output.StopTask()
        self.dither_gotozero()
        self.stopped = True

    def gotobluelaser(self):
        s = self.settings
        if s['blue_laser_on']:
            return
        s['blue_laser_on'] = True
        running = not self.stopped
        if running: #if we are free running
            self.startstop()

        t = np.arange(0, .01, 1 / s['sample_rate'])
        sampsPerPeriod = len(t)
        ditherWaveform = np.zeros(len(t))
        _, ttlWaveform = calc_ttl(self.settings)
        ttlWaveform[4:10] = 0
        self.data2 = np.concatenate([ditherWaveform, ttlWaveform])
        #self.analog_output2.StopTask()
        #self.analog_output2.CfgSampClkTiming("", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_FiniteSamps,sampsPerPeriod)
        #self.analog_output2.WriteAnalogF64(sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data2, byref(self.read), None)
        #self.analog_output2.StartTask()
        time.sleep(.02)
        #self.analog_output2.StopTask()
        if running:
            self.startstop()

    def gotoyellowlaser(self):
        s = self.settings
        if not s['blue_laser_on']:
            return
        s['blue_laser_on'] = False
        running = not self.stopped
        if running: #if we are free running
            self.startstop()
        t = np.arange(0, .01, 1 / s['sample_rate'])
        sampsPerPeriod = len(t)
        ditherWaveform = np.zeros(len(t))
        _, ttlWaveform = calc_ttl(self.settings)
        self.data2 = np.concatenate([ditherWaveform, ttlWaveform])
        #self.analog_output2.StopTask()
        #self.analog_output2.CfgSampClkTiming("", self.sample_rate, PyDAQmx.DAQmx_Val_Rising, PyDAQmx.DAQmx_Val_FiniteSamps,sampsPerPeriod)
        #self.analog_output2.WriteAnalogF64(sampsPerPeriod, 0, -1, PyDAQmx.DAQmx_Val_GroupByChannel, self.data2, byref(self.read), None)
        #self.analog_output2.StartTask()
        time.sleep(.02)
        #self.analog_output2.StopTask()
        if running:
            self.startstop()


##############################################################################
####   GRAPHICAL USER INTERFACE ##############################################
##############################################################################


class SliderLabel(QtWidgets.QWidget):
    """
    SliderLabel is a widget containing a QSlider and a QSpinBox (or QDoubleSpinBox if decimals are required)
    The QSlider and SpinBox are connected so that a change in one causes the other to change. 
    """
    changeSignal=Signal(int)

    def __init__(self,decimals=0): #decimals specifies the resolution of the slider.  0 means only integers,  1 means the tens place, etc.
        QtWidgets.QWidget.__init__(self)
        self.slider=QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.decimals=decimals
        if self.decimals <= 0:
            self.label = QtWidgets.QSpinBox()
        else:
            self.label=QtWidgets.QDoubleSpinBox()
            self.label.setDecimals(self.decimals)
        self.layout=QtWidgets.QHBoxLayout()
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.label)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)
        self.slider.valueChanged.connect(lambda val: self.updateLabel(val/10**self.decimals))
        self.label.valueChanged.connect(self.updateSlider)
        self.valueChanged = self.label.valueChanged

    def updateSlider(self, value):
        self.slider.setValue(int(value*10**self.decimals))

    def updateLabel(self,value):
        self.label.setValue(value)

    def value(self):
        return self.label.value()

    def setRange(self,minn,maxx):
        self.slider.setRange(minn*10**self.decimals,maxx*10**self.decimals)
        self.label.setRange(minn,maxx)

    def setMinimum(self,minn):
        self.slider.setMinimum(minn*10**self.decimals)
        self.label.setMinimum(minn)

    def setMaximum(self,maxx):
        self.slider.setMaximum(maxx*10**self.decimals)
        self.label.setMaximum(maxx)

    def setValue(self,value):
        self.slider.setValue(value*10**self.decimals)
        self.label.setValue(value)

    def setEnabled(self, bool):
        self.slider.setEnabled(bool)
        self.label.setEnabled(bool)
    def setSingleStep(self,value):
        self.label.setSingleStep(value)
        
        
class CheckBox(QtWidgets.QCheckBox):
    """ I overwrote the QCheckBox class so that every graphical element has the method 'setValue'. """
    def __init__(self,parent=None):
        QtWidgets.QCheckBox.__init__(self,parent)

    def setValue(self,value):
        self.setChecked(value)

class Triangle_Scan_Checkbox(CheckBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.stateChanged.connect(self.changed)
    def changed(self, state):
        if state == 0: #unchecked
            self.parent().items['flyback_duration']['object'].setEnabled(True)
        if state == 2: #checked
            self.parent().items['flyback_duration']['object'].setEnabled(False)

class MainGui(QtWidgets.QWidget):
    """ This class creates and controls the GUI. """
    changeSignal=Signal()
    def __init__(self):
        QtWidgets.QWidget.__init__(self)
        #self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        self.setWindowTitle('Light Sheet Controller')
        formlayout=QtWidgets.QFormLayout()
        self.settings=Settings()
        if dac_present:
            self.lightSheetDriver=LightSheetDriver(self.settings)

        mV_per_pixel         = SliderLabel(2);   mV_per_pixel.setRange(0,100)
        maximum_displacement = SliderLabel(0);   maximum_displacement.setRange(0,1000)
        nSteps               = SliderLabel(0);   nSteps.setRange(1,1000)
        step_duration        = SliderLabel(0);   step_duration.setRange(1,200)
        flyback_duration     = SliderLabel(0);   flyback_duration.setRange(0, 1000)
        total_cycle_period   = SliderLabel(3);   total_cycle_period.setRange(0,10)
        dither_amp           = SliderLabel(0);   dither_amp.setRange(0,1000)
        offset               = SliderLabel(0);   offset.setRange(0, 1000)
        triangle_scan        = Triangle_Scan_Checkbox(self)
        alternate_lasers     = CheckBox(self)

        self.items = OrderedDict()
        self.items['mV_per_pixel']              = {'name': 'mV_per_pixel',          'string': 'mV/pixel',                       'object': mV_per_pixel}
        self.items['maximum_displacement']      = {'name': 'maximum_displacement',  'string': 'Maximum displacement (pixels)',  'object': maximum_displacement}
        self.items['nSteps']                    = {'name': 'nSteps',                'string': 'Number of steps per cycle',      'object': nSteps}
        self.items['step_duration']             = {'name': 'step_duration',         'string': 'Step duration (ms)',             'object': step_duration}
        self.items['flyback_duration']          = {'name': 'flyback_duration',      'string': 'Flyback Duration (ms)',          'object': flyback_duration}
        self.items['total_cycle_period']        = {'name': 'total_cycle_period',    'string': 'Total cycle Period (s)',         'object': total_cycle_period}
        self.items['dither_amp']                = {'name': 'dither_amp',            'string': 'Dither amplitude mV',            'object': dither_amp}
        self.items['offset']                    = {'name': 'offset',                'string': 'Offset (pixels)',                 'object': offset}
        self.items['triangle_scan']             = {'name': 'triangle_scan',         'string': 'Triangle Scan',                  'object': triangle_scan}
        self.items['alternate_lasers']          = {'name': 'alternate_lasers',      'string': 'Alternate Lasers', 'object': alternate_lasers}

        for item in self.items.values():
            formlayout.addRow(item['string'],item['object'])
            item['object'].setValue(self.settings[item['name']])
        self.save1 = QtWidgets.QPushButton('Save'); self.save1.setStyleSheet("background-color: red"); self.save1.clicked.connect(lambda: self.memstore(1))
        self.save2 = QtWidgets.QPushButton('Save'); self.save2.setStyleSheet("background-color: red"); self.save2.clicked.connect(lambda: self.memstore(2))
        self.save3 = QtWidgets.QPushButton('Save'); self.save3.setStyleSheet("background-color: red"); self.save3.clicked.connect(lambda: self.memstore(3))
        self.recall1 = QtWidgets.QPushButton('Recall'); self.recall1.setStyleSheet("background-color: green"); self.recall1.clicked.connect(lambda: self.memrecall(1))
        self.recall2 = QtWidgets.QPushButton('Recall'); self.recall2.setStyleSheet("background-color: green"); self.recall2.clicked.connect(lambda: self.memrecall(2))
        self.recall3 = QtWidgets.QPushButton('Recall'); self.recall3.setStyleSheet("background-color: green"); self.recall3.clicked.connect(lambda: self.memrecall(3))
        memlayout = QtWidgets.QGridLayout()
        memlayout.setHorizontalSpacing(70)
        memlayout.addWidget(QtWidgets.QLabel('Setting #1'),0,0); memlayout.addWidget(self.save1,1,0); memlayout.addWidget(self.recall1,2,0)
        memlayout.addWidget(QtWidgets.QLabel('Setting #2'),0,1); memlayout.addWidget(self.save2,1,1); memlayout.addWidget(self.recall2,2,1)
        memlayout.addWidget(QtWidgets.QLabel('Setting #3'),0,2); memlayout.addWidget(self.save3,1,2); memlayout.addWidget(self.recall3,2,2)
        membox=QtWidgets.QGroupBox("Settings")
        membox.setLayout(memlayout)
        self.stopButton=QtWidgets.QPushButton('Stop'); self.stopButton.setStyleSheet("background-color: red"); self.stopButton.clicked.connect(self.startstop)
        self.zeroButton=QtWidgets.QPushButton('Go to Offset'); self.zeroButton.setStyleSheet("background-color: #fff"); self.zeroButton.clicked.connect(self.gotozero)
        self.gotoEndButton=QtWidgets.QPushButton('Go to End'); self.gotoEndButton.setStyleSheet("background-color: #fff"); self.gotoEndButton.clicked.connect(self.gotoend)
        self.blueButton=QtWidgets.QPushButton('Switch to blue laser'); self.blueButton.setStyleSheet("background-color: #fff"); self.blueButton.clicked.connect(self.gotobluelaser)
        self.yellowButton=QtWidgets.QPushButton('Switch to yellow laser'); self.yellowButton.setStyleSheet("background-color: #fff"); self.yellowButton.clicked.connect(self.gotoyellowlaser)
        self.viewWaveFormButton=QtWidgets.QPushButton('View Waveforms'); self.viewWaveFormButton.setStyleSheet("background-color: #fff"); self.viewWaveFormButton.clicked.connect(self.viewWaveForms)
        stopacquirebox=QtWidgets.QGridLayout()
        stopacquirebox.addWidget(self.stopButton,0,0)
        stopacquirebox.addWidget(self.viewWaveFormButton,1,0)
        stopacquirebox.addWidget(self.zeroButton,0,1)
        stopacquirebox.addWidget(self.gotoEndButton,1,1)
        
        stopacquirebox.addWidget(self.blueButton,0,2)
        stopacquirebox.addWidget(self.yellowButton,1,2)
        self.layout=QtWidgets.QVBoxLayout()
        self.layout.addLayout(formlayout)
        self.layout.addWidget(membox)
        self.layout.addSpacing(50)
        self.layout.addLayout(stopacquirebox)
        self.setLayout(self.layout)
        self.connectToChangeSignal()
        self.changeSignal.connect(self.updateValues)
        self.setGeometry(QtCore.QRect(488, 390, 704, 376))
        self.show()

    def connectToChangeSignal(self):
        for item in self.items.values():
            methods=[method for method in dir(item['object']) if callable(getattr(item['object'], method))]
            if 'valueChanged' in methods:
                item['object'].valueChanged.connect(self.changeSignal)
            elif 'stateChanged' in methods:
                item['object'].stateChanged.connect(self.changeSignal)
            elif 'currentIndexChanged' in methods:
                item['object'].currentIndexChanged.connect(self.changeSignal)

    def updateValues(self):
        for item in self.items.values():
            methods=[method for method in dir(item['object']) if callable(getattr(item['object'], method))]
            if 'value' in methods:
                item['value']=item['object'].value()
            elif 'currentText' in methods:
                item['value']=item['object'].currentText()
            elif 'isChecked' in methods:
                item['value']=item['object'].isChecked()
            self.settings[item['name']] = item['value']
        if dac_present:
            self.lightSheetDriver.refresh()

    def memrecall(self,i):
        '''i is the setting number we are recalling'''
        try:
            self.changeSignal.disconnect(self.updateValues)
        except TypeError:
            pass
        s=self.settings
        s.d[0]=s.d[i].copy()
        for item in self.items.values():
            item['object'].setValue(s.d[0][item['name']])
        self.changeSignal.connect(self.updateValues)
        if dac_present:
            self.lightSheetDriver.refresh()

    def memstore(self,i):
        '''i is the setting number we are storing.  settings.d[0] is always the current setting.'''
        self.settings.d[i]=self.settings.d[0].copy()
        self.settings.save()
    
    def startstop(self):
        if dac_present:
            if self.lightSheetDriver.stopped is False: #if we are free running
                self.lightSheetDriver.startstop()
                self.stopButton.setText('Free Run')
                self.stopButton.setStyleSheet("background-color: green")
            else:
                self.updateValues()
                self.lightSheetDriver.startstop()
                self.stopButton.setText('Stop Free Run')
                self.stopButton.setStyleSheet("background-color: red")

    def gotozero(self):
        if dac_present:
            if self.lightSheetDriver.stopped is False: #if we are free running
                self.lightSheetDriver.startstop()
                self.stopButton.setText('Free Run')
                self.stopButton.setStyleSheet("background-color: green")                
            self.lightSheetDriver.gotozero()

    def gotoend(self):
        if dac_present:
            if self.lightSheetDriver.stopped is False: #if we are free running
                self.lightSheetDriver.startstop()
                self.stopButton.setText('Free Run')
                self.stopButton.setStyleSheet("background-color: green")                
            self.lightSheetDriver.gotoend()

    def gotobluelaser(self):
        if dac_present:             
            self.lightSheetDriver.gotobluelaser()

    def gotoyellowlaser(self):
        if dac_present:            
            self.lightSheetDriver.gotoyellowlaser()

    def viewWaveForms(self):
        s=self.settings
        t, piezoWaveform=calcPiezoWaveform(s)

        t, cameraTTL = calcCameraTTL(s)
        t, ditherWaveform = calcDitherWaveform(s)
        t, ttl = calc_ttl(s)

        pw = pg.PlotWidget(name = 'Piezo Waveform')
        
        pw.plot(t, piezoWaveform, pen=pg.mkPen('r'))
        pw.plot(t, ditherWaveform, pen=pg.mkPen('g'))
        pw.plot(t, cameraTTL, pen=pg.mkPen('w'))
        pw.plot(t, ttl, pen=pg.mkPen('g'))

        pw.setLabel('bottom', 'Time', units='s')
        pw.setLabel('left', 'Voltage', units='V')
        pw.show()
        self.waveform_plot = pw
        s.save()

def start_light_sheet_controller():
    app = QtWidgets.QApplication(sys.argv)
    check_if_NI_devs_are_present()
    ipython_qt_event_loop_setup()
    maingui = MainGui()
    return maingui, app

if __name__ == '__main__':
    '''
    Code to run every inside PyCharm every time you open a console:
    import sys, os
    from matplotlib import pyplot  # This is totally unneccessary but it turns on 'interactive mode', which makes debugging in PyCharm way easier.
    sys.path.append(os.path.join(os.path.expanduser('~'), 'Documents', 'GitHub', 'light_sheet_controller'))
    from light_sheet_controller import *
    app = QtWidgets.QApplication(sys.argv)
    check_if_NI_devs_are_present()
    maingui = MainGui()
    '''
    maingui, app = start_light_sheet_controller()
    sys.exit(app.exec_())

