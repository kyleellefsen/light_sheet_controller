import PyDAQmx

import ctypes

def DAQmxGetSysDevNames():
    import sys, io
    old_stdout = sys.stdout
    sys.stdout = io.StringIO() # dummy field to catch print() output
    try:
        numBytesNeeded = PyDAQmx.DAQmxGetSysDevNames(None, 0)
    except PyDAQmx.DAQError as e:
        numBytesNeeded = e.error
    if numBytesNeeded == 0:
        return []
    # Restore stdout
    sys.stdout = old_stdout
    # We now know how many bytes are needed.
    # Allocate the buffer
    stringBuffer = ctypes.create_string_buffer(numBytesNeeded)
    # Get the device names
    PyDAQmx.DAQmxGetSysDevNames(stringBuffer, numBytesNeeded)
    # Extract the device name string
    names = ctypes.string_at(stringBuffer)
    names = str(names,'utf-8')
    names = names.split(', ')
    return names


def getNIDevInfo():
    names = DAQmxGetSysDevNames()
    info = {dev_name:{} for dev_name in names}
    for dev_name in names:
        numBytesNeeded = PyDAQmx.DAQmxGetDevProductType(dev_name, None, 0)
        stringBuffer = ctypes.create_string_buffer(numBytesNeeded)
        PyDAQmx.DAQmxGetDevProductType(dev_name, stringBuffer, numBytesNeeded)
        product_type = ctypes.string_at(stringBuffer)
        product_type = str(product_type, 'utf-8')
        info[dev_name]['product_type'] = product_type
    for dev_name in names:
        serial_num = ctypes.c_ulong()
        PyDAQmx.DAQmxGetDevSerialNum(dev_name, serial_num)
        serial_num = serial_num.value
        info[dev_name]['serial_number'] = serial_num
        info[dev_name]['serial_number_hex'] = "0x%X" % serial_num
    return info
