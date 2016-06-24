# -*- coding: utf-8 -*-
"""
Created on Fri Jun 20 19:06:55 2014
@author: Kyle Ellefsen


Make sure that the National Instruments DAC USB-6001 is "Dev1" with the National Instruments Measurement and Automation tool.  

To Use:
Run this program with python.  

"""

import sys, os, time
os.chdir(os.path.split(os.path.realpath(__file__))[0])
from dependency_check import check_dependencies
check_dependencies('PyDAQmx','pyqtgraph', 'PyQt4','numpy','scipy')
from PyDAQmx import *
from PyDAQmx.DAQmxCallBack import *
import numpy as np
from PyQt4.QtGui import * # Qt is Nokias GUI rendering code written in C++.  PyQt4 is a library in python which binds to Qt
from PyQt4.QtCore import *
from PyQt4.QtCore import pyqtSignal as Signal
from PyQt4.QtCore import pyqtSlot  as Slot
import pickle
from os.path import expanduser
from pyqtgraph import plot
import pyqtgraph as pg
import scipy.ndimage
import time


dac_present = True


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


def calcPiezoWaveform(settings):
    s = settings
    step_duration_seconds = s['step_duration']/1000
    flyback_duration_seconds = s['flyback_duration']/1000
    total_cycle_period = s['total_cycle_period']
    if total_cycle_period < s['nSteps']*step_duration_seconds+flyback_duration_seconds:
        total_cycle_period = s['nSteps']*step_duration_seconds+flyback_duration_seconds
    t = np.arange(0, total_cycle_period, 1/s['sample_rate'])
    maxV = s['maximum_displacement']*s['mV_per_pixel']/1000
    step_end_times=np.linspace(step_duration_seconds,s['nSteps']*step_duration_seconds,s['nSteps'])
    step_start_times=step_end_times-step_duration_seconds
    step_values=np.linspace(0,maxV,s['nSteps'])
    V = np.zeros(len(t))
    for step in np.arange(s['nSteps']):
        idx=np.logical_and(t>=step_start_times[step], t<=step_end_times[step])
        V[idx]=step_values[step]
  
    
    nSamps_ramp = np.count_nonzero(t<step_end_times[-1])
    nSamps_reset = int(flyback_duration_seconds*s['sample_rate'])
    theta = np.linspace(0,np.pi,nSamps_reset)
    reposition_sig = maxV*(np.cos(theta)+1)/2
    V[nSamps_ramp:] = maxV

    sigma = 3
    V = scipy.ndimage.filters.gaussian_filter1d(V, sigma)
    V[nSamps_ramp:nSamps_ramp + nSamps_reset] = reposition_sig
    V[nSamps_ramp+nSamps_reset:] = 0
    return t, V


def calcCameraTTL(settings):
    s=settings
    t=np.arange(0,s['total_cycle_period'],1/s['sample_rate'])
    step_duration_seconds=s['step_duration']/1000
    step_end_times=np.linspace(step_duration_seconds,s['nSteps']*step_duration_seconds,s['nSteps'])
    step_start_times=step_end_times-step_duration_seconds
    V=np.zeros(len(t),dtype=np.uint8)
    for step in np.arange(s['nSteps']):
        idx=np.argmax(t>=step_start_times[step])
        V[idx]=5
    return t, V


def calcDitherWaveform(settings):
    s=settings
    t=np.arange(0,s['total_cycle_period'],1/s['sample_rate'])
    amp=s['dither_amp']/1000
    freq=s['dither_freq']
    period=1/freq
    max_periods=int(np.floor(s['total_cycle_period']/period))
    if max_periods==0:
        max_periods=1
    new_freq=max_periods/s['total_cycle_period']
    V=amp*np.sin(2*np.pi*new_freq*t)
    V-=np.min(V)
    return t, V

        
class LightSheetDriver(QWidget):
    ''' This class sends creates the signal which will control the piezo, and sends it to the DAQ.'''
    def __init__(self,settings):
        QWidget.__init__(self)
        self.settings=settings
        self.sample_rate=settings['sample_rate']
        self.sampsPerPeriod=1 # samples per channel
        self.calculate()
        self.read = int32()
        self.digital_read = int32()
        self.createTask()
        self.hide()
    def createTask(self):
        self.analog_output = Task()
        self.analog_output.CreateAOVoltageChan("Dev1/ao0","",-10.0,10.0,DAQmx_Val_Volts,None)
        self.analog_output.CreateAOVoltageChan("Dev1/ao1","",-10.0,10.0,DAQmx_Val_Volts,None) 

                        #  CfgSampClkTiming(source, rate, activeEdge, sampleMode, sampsPerChan)
        self.analog_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.sampsPerPeriod)
                        #  WriteAnalogF64(numSampsPerChan, autoStart, timeout, dataLayout, writeArray, sampsPerChanWritten, reserved)
        self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1,DAQmx_Val_GroupByChannel,self.data,byref(self.read),None) 
        self.analog_output.StartTask()
        
        # self.digital_output = Task()
        # self.digital_output.CreateDOChan(b'Dev1/port0',b'',DAQmx_Val_ChanForAllLines)
        # self.digital_output.CfgSampClkTiming('/Dev1/PFI0',10.0,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,8)
        # self.digital_output.WriteDigitalU8(self.sampsPerPeriod, 0, -1, DAQmx_Val_GroupByChannel,self.digital_data, byref(self.digital_read) ,None) #https://www.quark.kj.yamagata-u.ac.jp/~miyachi/nidaqmxbase-3.4.0/documentation/docsource/daqmxbasecfunc.chm/DAQmxWriteDigitalU8.html
        self.digital_data=np.array([0,1,0,1,0,1,1,0],dtype=np.uint8)    
        # self.digital_data=np.array([1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],dtype=np.uint8)
        # self.digital_output.WriteDigitalU8(8,1,10.0,DAQmx_Val_GroupByChannel,self.digital_data,byref(self.digital_read),None)
        self.stopped=False
        self.acquiring=False

    def calculate(self):
        t, piezoWaveform = calcPiezoWaveform(self.settings)
        t, cameraTTL = calcCameraTTL(self.settings)
        self.data=np.concatenate((piezoWaveform,cameraTTL))
        self.sampsPerPeriod=len(piezoWaveform)
        #self.digital_data=np.zeros(8*len(t),dtype=np.uint8)
        #self.digital_data[:len(t)]=cameraTTL

    def startstop(self):
        if self.stopped:
            self.calculate()
            self.analog_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.sampsPerPeriod)
            self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1,DAQmx_Val_GroupByChannel,self.data,byref(self.read),None) 
            self.analog_output.StartTask()
            self.stopped=False
        else:
            self.analog_output.StopTask()
            self.stopped=True

    def refresh(self):
        if self.stopped is False:
            self.calculate()
            self.analog_output.StopTask()
            self.analog_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.sampsPerPeriod)
            self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1,DAQmx_Val_GroupByChannel,self.data,byref(self.read),None)
            #self.digital_output.StopTask()
            #self.digital_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_ContSamps,self.sampsPerPeriod)
            #self.digital_output.WriteDigitalU8(self.sampsPerPeriod, 0, -1, DAQmx_Val_GroupByChannel,self.digital_data, byref(self.digital_read) ,None) #https://www.quark.kj.yamagata-u.ac.jp/~miyachi/nidaqmxbase-3.4.0/documentation/docsource/daqmxbasecfunc.chm/DAQmxWriteDigitalU8.html
            self.analog_output.StartTask()

    def gotozero(self):
        s=self.settings
        t=np.arange(0,s['total_cycle_period'],1/s['sample_rate'])
        piezoWaveform=np.zeros(len(t))
        cameraTTL=np.zeros(len(t))
        self.data=np.concatenate((piezoWaveform,cameraTTL))
        self.sampsPerPeriod=len(piezoWaveform)
        self.analog_output.StopTask()
        self.analog_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,self.sampsPerPeriod)
        self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1,DAQmx_Val_GroupByChannel,self.data,byref(self.read),None) 
        self.analog_output.StartTask()
        time.sleep(1)
        self.analog_output.StopTask()
        self.stopped=True

    def gotoend(self):
        s=self.settings
        t=np.arange(0,s['total_cycle_period'],1/s['sample_rate'])
        maxV=s['maximum_displacement']*s['mV_per_pixel']/1000
        piezoWaveform=np.ones(len(t)) * maxV
        cameraTTL=np.zeros(len(t))
        self.data=np.concatenate((piezoWaveform,cameraTTL))
        self.sampsPerPeriod=len(piezoWaveform)
        self.analog_output.StopTask()
        self.analog_output.CfgSampClkTiming("",self.sample_rate,DAQmx_Val_Rising,DAQmx_Val_FiniteSamps,self.sampsPerPeriod)
        self.analog_output.WriteAnalogF64(self.sampsPerPeriod,0,-1,DAQmx_Val_GroupByChannel,self.data,byref(self.read),None) 
        self.analog_output.StartTask()
        time.sleep(1)
        self.analog_output.StopTask()
        self.stopped = True


##############################################################################
####   GRAPHICAL USER INTERFACE ##############################################
##############################################################################


class SliderLabel(QWidget):
    """
    SliderLabel is a widget containing a QSlider and a QSpinBox (or QDoubleSpinBox if decimals are required)
    The QSlider and SpinBox are connected so that a change in one causes the other to change. 
    """
    changeSignal=Signal(int)

    def __init__(self,decimals=0): #decimals specifies the resolution of the slider.  0 means only integers,  1 means the tens place, etc.
        QWidget.__init__(self)
        self.slider=QSlider(Qt.Horizontal)
        self.decimals=decimals
        if self.decimals<=0:
            self.label=QSpinBox()
        else:
            self.label=QDoubleSpinBox()
            self.label.setDecimals(self.decimals)
        self.layout=QHBoxLayout()
        self.layout.addWidget(self.slider)
        self.layout.addWidget(self.label)
        self.layout.setContentsMargins(0,0,0,0)
        self.setLayout(self.layout)
        self.slider.valueChanged.connect(lambda val: self.updateLabel(val/10**self.decimals))
        self.label.valueChanged.connect(self.updateSlider)
        self.valueChanged=self.label.valueChanged

    @Slot(int, float)
    def updateSlider(self,value):
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

    def setSingleStep(self,value):
        self.label.setSingleStep(value)
        
        
class CheckBox(QCheckBox):
    """ I overwrote the QCheckBox class so that every graphical element has the method 'setValue'. """
    def __init__(self,parent=None):
        QCheckBox.__init__(self,parent)

    def setValue(self,value):
        self.setChecked(value)


class MainGui(QWidget):
    """ This class creates and controls the GUI. """
    changeSignal=Signal()
    def __init__(self):
        QWidget.__init__(self)
        self.setWindowFlags(Qt.WindowStaysOnTopHint)
        self.setWindowTitle('Light Sheet Controller')
        formlayout=QFormLayout()
        self.settings=Settings()
        if dac_present:
            self.lightSheetDriver=LightSheetDriver(self.settings)
        mV_per_pixel         = SliderLabel(2);   mV_per_pixel.setRange(0,100)
        maximum_displacement = SliderLabel(0);   maximum_displacement.setRange(0,1000)
        nSteps               = SliderLabel(0);   nSteps.setRange(1,1000)
        step_duration        = SliderLabel(0);   step_duration.setRange(1,200)
        flyback_duration     = SliderLabel(0);   flyback_duration.setRange(1, 100)
        total_cycle_period   = SliderLabel(3);   total_cycle_period.setRange(0,10)
        dither_amp           = SliderLabel(0);   dither_amp.setRange(0,1000)
        dither_freq          = SliderLabel(3);   dither_freq.setRange(0,50)
        total_Ncycles        = SliderLabel(0);   total_Ncycles.setRange(0,100)
        self.items=[]
        self.items.append({'name':'mV_per_pixel','string':'mV/pixel','object':mV_per_pixel})
        self.items.append({'name':'maximum_displacement','string':'Maximum displacement (pixels)','object':maximum_displacement})
        self.items.append({'name':'nSteps','string':'Number of steps per cycle','object':nSteps})
        self.items.append({'name':'step_duration','string':'Step duration (ms)','object':step_duration})
        self.items.append({'name': 'flyback_duration', 'string': 'Flyback Duration (ms)', 'object': flyback_duration})
        self.items.append({'name':'total_cycle_period','string':'Total cycle Period (s)','object':total_cycle_period})
        self.items.append({'name':'dither_amp','string':'Dither amplitude mV','object':dither_amp})
        self.items.append({'name':'dither_freq','string':'Dither Frequency','object':dither_freq})
        self.items.append({'name':'total_Ncycles','string':'Total number of cycles','object':total_Ncycles})
        for item in self.items:
            formlayout.addRow(item['string'],item['object'])
            item['object'].setValue(self.settings[item['name']])
        self.save1=QPushButton('Save'); self.save1.setStyleSheet("background-color: red"); self.save1.clicked.connect(lambda: self.memstore(1))
        self.save2=QPushButton('Save'); self.save2.setStyleSheet("background-color: red"); self.save2.clicked.connect(lambda: self.memstore(2))
        self.save3=QPushButton('Save'); self.save3.setStyleSheet("background-color: red"); self.save3.clicked.connect(lambda: self.memstore(3))
        self.recall1=QPushButton('Recall'); self.recall1.setStyleSheet("background-color: green"); self.recall1.clicked.connect(lambda: self.memrecall(1))
        self.recall2=QPushButton('Recall'); self.recall2.setStyleSheet("background-color: green"); self.recall2.clicked.connect(lambda: self.memrecall(2))
        self.recall3=QPushButton('Recall'); self.recall3.setStyleSheet("background-color: green"); self.recall3.clicked.connect(lambda: self.memrecall(3))
        memlayout=QGridLayout()
        memlayout.setHorizontalSpacing(70)
        memlayout.addWidget(QLabel('Setting #1'),0,0); memlayout.addWidget(self.save1,1,0); memlayout.addWidget(self.recall1,2,0)
        memlayout.addWidget(QLabel('Setting #2'),0,1); memlayout.addWidget(self.save2,1,1); memlayout.addWidget(self.recall2,2,1)
        memlayout.addWidget(QLabel('Setting #3'),0,2); memlayout.addWidget(self.save3,1,2); memlayout.addWidget(self.recall3,2,2)
        membox=QGroupBox("Settings")
        membox.setLayout(memlayout)
        self.stopButton=QPushButton('Stop'); self.stopButton.setStyleSheet("background-color: red"); self.stopButton.clicked.connect(self.startstop)
        self.zeroButton=QPushButton('Go to Zero'); self.zeroButton.setStyleSheet("background-color: #fff"); self.zeroButton.clicked.connect(self.gotozero)
        self.gotoEndButton=QPushButton('Go to End'); self.gotoEndButton.setStyleSheet("background-color: #fff"); self.gotoEndButton.clicked.connect(self.gotoend)
        self.viewDitherWaveFormButton=QPushButton('View Dither Waveform'); self.viewDitherWaveFormButton.setStyleSheet("background-color: #fff"); self.viewDitherWaveFormButton.clicked.connect(self.viewDitherWaveForm)
        self.viewPiezoWaveFormButton=QPushButton('View Piezo Waveform'); self.viewPiezoWaveFormButton.setStyleSheet("background-color: #fff"); self.viewPiezoWaveFormButton.clicked.connect(self.viewPiezoWaveForm)
        stopacquirebox=QGridLayout()
        stopacquirebox.addWidget(self.stopButton,0,0)
        stopacquirebox.addWidget(self.zeroButton,0,3)
        stopacquirebox.addWidget(self.gotoEndButton,1,3)
        stopacquirebox.addWidget(self.viewDitherWaveFormButton,0,4)
        stopacquirebox.addWidget(self.viewPiezoWaveFormButton,1,4)
        self.layout=QVBoxLayout()
        self.layout.addLayout(formlayout)
        self.layout.addWidget(membox)
        self.layout.addSpacing(50)
        self.layout.addLayout(stopacquirebox)
        self.setLayout(self.layout)
        self.connectToChangeSignal()
        self.changeSignal.connect(self.updateValues)
        self.setGeometry(QRect(488, 390, 704, 376))
        self.show()

    def connectToChangeSignal(self):
        for item in self.items:
            methods=[method for method in dir(item['object']) if callable(getattr(item['object'], method))]
            if 'valueChanged' in methods:
                item['object'].valueChanged.connect(self.changeSignal)
            elif 'stateChanged' in methods:
                item['object'].stateChanged.connect(self.changeSignal)
            elif 'currentIndexChanged' in methods:
                item['object'].currentIndexChanged.connect(self.changeSignal)

    def updateValues(self):
        for item in self.items:
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
        self.changeSignal.disconnect(self.updateValues)
        s=self.settings
        s.d[0]=s.d[i].copy()
        for item in self.items:
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

    def viewPiezoWaveForm(self):
        s=self.settings
        t, waveform=calcPiezoWaveform(s)
        pw = pg.PlotWidget(name='Piezo Waveform')
        
        pw.plot(t,waveform)
        pw.setLabel('bottom','Time',units='s')
        pw.setLabel('left','Voltage',units='V')
        pw.show()
        self.pizeowaveform=pw
        s.save()

    def viewDitherWaveForm(self):
        s=self.settings
        t, waveform=calcDitherWaveform(s)
        pw = pg.PlotWidget(name='Dither Waveform')
        pw.plot(t,waveform)
        pw.setLabel('bottom','Time',units='s')
        pw.setLabel('left','Voltage',units='V')
        pw.show()
        self.ditherwaveform=pw

    
if __name__ == '__main__':
    app = QApplication(sys.argv)
    maingui = MainGui()
    self = maingui
    sys.exit(app.exec_())
