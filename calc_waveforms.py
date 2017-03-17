import numpy as np
import scipy.ndimage
from qtpy import QtWidgets

def warning(text):
    msgBox = QtWidgets.QMessageBox()
    msgBox.setText(str(text))
    msgBox.addButton(QtWidgets.QMessageBox.Ok)
    msgBox.setDefaultButton(QtWidgets.QMessageBox.Ok)
    ret = msgBox.exec_()
    return ret

def get_time_array(s):
    '''
    If triangle scan is on, this will return a trace that lasts two cycles (up and back down). If triangle scan is off, it will return only one cycles.
    :param s:
    :return:
    '''
    step_duration_seconds = s['step_duration'] / 1000
    if s['triangle_scan']:
        flyback_duration_seconds = 0
    else:
        flyback_duration_seconds = s['flyback_duration'] / 1000

    total_cycle_period = s['total_cycle_period']
    if total_cycle_period < s['nSteps'] * step_duration_seconds + flyback_duration_seconds:
        total_cycle_period = s['nSteps'] * step_duration_seconds + flyback_duration_seconds
    if not s['triangle_scan']:
        t = np.arange(0, total_cycle_period, 1 / s['sample_rate'])
    else:
        t = np.arange(0, 2*total_cycle_period, 1 / s['sample_rate'])
    return t

def get_offset_volts(s):
    offset_pixels = s['offset']
    offset_volts = offset_pixels * (s['mV_per_pixel'] / 1000.0)
    return offset_volts

def calcPiezoWaveform(settings):
    s = settings
    t = get_time_array(s)
    step_duration_seconds = s['step_duration'] / 1000
    maxV = s['maximum_displacement'] * s['mV_per_pixel'] / 1000
    step_end_times = np.linspace(step_duration_seconds, s['nSteps'] * step_duration_seconds, s['nSteps'])
    step_start_times = step_end_times - step_duration_seconds
    step_values = np.linspace(0, maxV, s['nSteps'])
    V = np.zeros(len(t))
    for step in np.arange(s['nSteps']):
        idx = np.logical_and(t >= step_start_times[step], t <= step_end_times[step])
        V[idx] = step_values[step]
    nSamps_ramp = np.count_nonzero(t < step_end_times[-1]) # This is how many time steps the ramp up lasts, including the leading zeros.
    V[nSamps_ramp+1:] = maxV
    if not s['triangle_scan']:
        flyback_duration_seconds = s['flyback_duration'] / 1000
        nSamps_reset = int(flyback_duration_seconds*s['sample_rate'])
        theta = np.linspace(0,np.pi,nSamps_reset)
        reposition_sig = maxV*(np.cos(theta)+1)/2
    else: #if triangle scan is on
        midpoint = int(len(t)/2)
        V[midpoint:] = V[midpoint:0:-1]

    sigma = 3
    V = scipy.ndimage.filters.gaussian_filter1d(V, sigma)
    if not s['triangle_scan']:
        V[nSamps_ramp:nSamps_ramp + nSamps_reset] = reposition_sig
        V[nSamps_ramp+nSamps_reset:] = 0


    V = V + get_offset_voltage(s)

    if np.max(V) >= 10:
        warning('The voltage of the piezo waveform can not exceed 10. The current max voltage is {}.  Adjust controls to fix this error. '.format(np.max(V)))
        V[V > 10] = 10
    if np.min(V) <= -10:
        warning('The voltage of the piezo waveform can not exceed -10. The current max voltage is {}. Adjust controls to fix this error. '.format(np.min(V)))
        V[V < -10] = -10
    assert np.max(V) <= 10
    assert np.min(V) >= -10
    return t, V


def calcCameraTTL(settings):
    s = settings
    step_duration_seconds = s['step_duration']/1000
    step_end_times = np.linspace(step_duration_seconds,s['nSteps']*step_duration_seconds,s['nSteps'])
    if s['triangle_scan']:
        total_cycle_period = s['total_cycle_period']
        if total_cycle_period < s['nSteps'] * step_duration_seconds:
            total_cycle_period = s['nSteps'] * step_duration_seconds
        step_end_times = np.concatenate([step_end_times,step_end_times+total_cycle_period])
    step_start_times=step_end_times-step_duration_seconds
    t = get_time_array(s)
    V = np.zeros(len(t),dtype=np.uint8)
    for step in np.arange(len(step_start_times)):
        idx=np.argmax(t >= step_start_times[step])
        V[idx]=5
    return t, V


def calcDitherWaveform(settings):
    s = settings
    t = get_time_array(s)
    amp = s['dither_amp'] / 1000
    step_duration_seconds = s['step_duration'] / 1000
    freq = 1/step_duration_seconds
    V = amp*np.cos(2*np.pi*freq*t+np.pi)
    V -= np.min(V)
    if not s['triangle_scan']:
        steps_end_time = s['nSteps'] * step_duration_seconds
        V[t > steps_end_time] = 0
    return t, V