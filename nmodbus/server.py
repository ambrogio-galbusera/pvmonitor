#!/usr/bin/env python3
"""
Pymodbus Synchronous Server Example
--------------------------------------------------------------------------

The synchronous server is implemented in pure python without any third
party libraries (unless you need to use the serial protocols which require
pyserial). This is helpful in constrained or old environments where using
twisted is just not feasible. What follows is an example of its use:
"""

import time, subprocess,serial,sys,string,binascii
import struct
from time import localtime, strftime
from datetime import datetime

# --------------------------------------------------------------------------- #
# import the various server implementations
# --------------------------------------------------------------------------- #
from pymodbus.version import version

from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.server.asynchronous import StartTcpServer

# --------------------------------------------------------------------------- #
# import the twisted libraries we need
# --------------------------------------------------------------------------- #
from twisted.internet.task import LoopingCall

#---------------------------------------------------------------------------- #
# Delta Solivia libraries
#---------------------------------------------------------------------------- #
from deltaInv import DeltaInverter

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
FORMAT = ('%(asctime)-15s %(threadName)-15s'
          ' %(levelname)-8s %(module)-15s:%(lineno)-8s %(message)s')
logging.basicConfig(format=FORMAT)
log = logging.getLogger()
log.setLevel(logging.INFO)


# --------------------------------------------------------------------------- #
# Solivia reader
# --------------------------------------------------------------------------- #
solivia_simFlag = 0
solivia_simValue = 1.5

if not solivia_simFlag:
    solivia_port="/dev/ttyUSB0"
    solivia_id=1
    solivia_connection = serial.Serial(solivia_port,9600,timeout=0.2);
    solivia_inverter = DeltaInverter(solivia_id)

def solivia_isFloat(string):
    try:
        float(string)
        return True
    except ValueError:
        return False

def solivia_getFloat (connection,inv,string):
    theReturn = float("0")
    cmd = inv.getCmdStringFor(string)
    connection.write(cmd)
    response = connection.read(100)
    value = inv.getValueFromResponse(response)
    if solivia_isFloat(value):
       theReturn = float(value)
    return theReturn

def solivia_floatToHex(f):
    return hex(struct.unpack('<I', struct.pack('<f', f))[0]).ljust(8, '0')

def solivia_updateChannel(ch, value, ts):
    """ Registers are as follow
    0-1 Input A (IEE754)
    2-3 Input B
    4-5 Temperature
    6-14 Timestamp "dd/mm/yy hh:mm:ss"
    15  Flags
    """
    vHex = solivia_floatToHex(value)[2:]
    log.debug("value=%f, vHex=%s"%(value,vHex))
    stime = ts.strftime("%d/%m/%y %H:%M:%S").encode('utf-8').hex().ljust(36, '0')

    values = [int(vHex[0:4],16), int(vHex[4:8],16),
                 0, 0,
                 0, 0,
              int(stime[0:4],16), int(stime[4:8],16), int(stime[8:12],16), int(stime[12:16],16), int(stime[16:20],16), int(stime[20:24],16), int(stime[24:28],16), int(stime[28:32],16), int(stime[32:36],16),
                 3]
    return values

def solivia_sim(a):
    global solivia_simValue

    log.info("Simulating inverter data")

    context = a[0]
    register = 3
    slaveId = 0x00

    address = 0x00
    values = solivia_updateChannel(0, solivia_simValue, datetime.now())
    context[slaveId].setValues(register, address, values)

    solivia_simValue += 0.1

def solivia_readData(s):
    log.debug("Reading %s"%(s))

    cmd = solivia_inverter.getCmdStringFor(s)
    log.debug("Command: %s"%binascii.hexlify(cmd))
    solivia_connection.write(cmd)
    rawResponse = solivia_connection.read(100)

    cntr = 0
    while (cntr < len(rawResponse) and rawResponse[cntr] == 0xff):
       cntr += 1

    #response = bytes.fromhex('0206010410020194332e03')
    response = rawResponse[cntr:]

    floatValue = float("0")
    if response:
        log.debug("Received response %s\n"%binascii.hexlify(response))

        value = solivia_inverter.getValueFromResponse(response)
        #log.debug("Value %s"%value)
        if solivia_isFloat(value):
            floatValue = float(value)

    return floatValue

def solivia_reader(a):
    """ A worker process that runs every so often and
    updates live values of the context. It should be noted
    that there is a race condition for the update.
    :param arguments: The input arguments to the call
    """
    log.info("Updating inverter data\n")

    dcVolts = solivia_readData('DC Volts1')
    dcCur = solivia_readData('DC Cur1')
    dcPow = solivia_readData('DC Pwr1')
    acVolts = solivia_readData('AC Volts')
    acCur = solivia_readData('AC Current')
    acPow = solivia_readData('AC Power')
    acTemp = solivia_readData('AC Temp')
    dcTemp = solivia_readData('DC Temp')

    context = a[0]
    register = 3
    slaveId = 0x00

    address = 0
    values = solivia_updateChannel(0, dcPow, datetime.now())
    context[slaveId].setValues(register, address, values)

    address = 16
    values = solivia_updateChannel(0, acPow, datetime.now())
    context[slaveId].setValues(register, address, values)

    address = 32
    values = solivia_updateChannel(0, acTemp, datetime.now())
    context[slaveId].setValues(register, address, values)

    address = 48
    values = solivia_updateChannel(0, dcTemp, datetime.now())
    context[slaveId].setValues(register, address, values)

    currDate = datetime.now().strftime("%d/%m/%y").encode('utf-8').hex().ljust(10, '0')
    currTime = datetime.now().strftime("%H:%M:%S").encode('utf-8').hex().ljust(8, '0')

    log.debug(currDate+"-"+currTime+" %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f OK\n" %(dcVolts, dcCur, dcPow, acVolts, acCur, acPow, dcTemp, acTemp ))

def run_server():
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #
    # The datastores only respond to the addresses that they are initialized to
    # Therefore, if you initialize a DataBlock to addresses of 0x00 to 0xFF, a
    # request to 0x100 will respond with an invalid address exception. This is
    # because many devices exhibit this kind of behavior (but not all)::
    #
    #     block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #
    # Continuing, you can choose to use a sequential or a sparse DataBlock in
    # your data context.  The difference is that the sequential has no gaps in
    # the data while the sparse can. Once again, there are devices that exhibit
    # both forms of behavior::
    #
    #     block = ModbusSparseDataBlock({0x00: 0, 0x05: 1})
    #     block = ModbusSequentialDataBlock(0x00, [0]*5)
    #
    # Alternately, you can use the factory methods to initialize the DataBlocks
    # or simply do not pass them to have them initialized to 0x00 on the full
    # address range::
    #
    #     store = ModbusSlaveContext(di = ModbusSequentialDataBlock.create())
    #     store = ModbusSlaveContext()
    #
    # Finally, you are allowed to use the same DataBlock reference for every
    # table or you may use a separate DataBlock for each table.
    # This depends if you would like functions to be able to access and modify
    # the same data or not::
    #
    #     block = ModbusSequentialDataBlock(0x00, [0]*0xff)
    #     store = ModbusSlaveContext(di=block, co=block, hr=block, ir=block)
    #
    # The server then makes use of a server context that allows the server to
    # respond with different slave contexts for different unit ids. By default
    # it will return the same context for every unit id supplied (broadcast
    # mode).
    # However, this can be overloaded by setting the single flag to False and
    # then supplying a dictionary of unit id to context mapping::
    #
    #     slaves  = {
    #         0x01: ModbusSlaveContext(...),
    #         0x02: ModbusSlaveContext(...),
    #         0x03: ModbusSlaveContext(...),
    #     }
    #     context = ModbusServerContext(slaves=slaves, single=False)
    #
    # The slave context can also be initialized in zero_mode which means that a
    # request to address(0-7) will map to the address (0-7). The default is
    # False which is based on section 4.4 of the specification, so address(0-7)
    # will map to (1-8)::
    #
    #     store = ModbusSlaveContext(..., zero_mode=True)
    # ----------------------------------------------------------------------- #
    store = ModbusSlaveContext(
        di=ModbusSequentialDataBlock(0, [17] * 100),
        co=ModbusSequentialDataBlock(0, [17] * 100),
        hr=ModbusSequentialDataBlock(0, [0] * 0xffff),
        ir=ModbusSequentialDataBlock(0, [17] * 100))

    context = ModbusServerContext(slaves=store, single=True)

    # ----------------------------------------------------------------------- #
    # initialize the server information
    # ----------------------------------------------------------------------- #
    # If you don't set this or any fields, they are defaulted to empty strings.
    # ----------------------------------------------------------------------- #
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'Pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/riptideio/pymodbus/'
    identity.ProductName = 'Pymodbus Server'
    identity.ModelName = 'Pymodbus Server'
    identity.MajorMinorRevision = version.short()

    # Background thread will read from Solivia inverter
    log.info("Scheduling task")
    time = 5.0
    if solivia_simFlag == 0:
       loop = LoopingCall(f=solivia_reader, a=(context,))
    else:
        loop = LoopingCall(f=solivia_sim, a=(context,))
    loop.start(time, now=True)  # initially delay by time

    # ----------------------------------------------------------------------- #
    # run the server you want
    # ----------------------------------------------------------------------- #
    # Tcp:
    # StartTcpServer(context, identity=identity, address=("", 5020))
    #
    # TCP with different framer
    # StartTcpServer(context, identity=identity,
    #                framer=ModbusRtuFramer, address=("0.0.0.0", 5020))

    # TLS
    # StartTlsServer(context, identity=identity,
    #                certfile="server.crt", keyfile="server.key", password="pwd",
    #                address=("0.0.0.0", 8020))

    # Tls and force require client's certificate for TLS full handshake:
    # StartTlsServer(context, identity=identity,
    #                certfile="server.crt", keyfile="server.key", password="pwd", reqclicert=True,
    #                address=("0.0.0.0", 8020))

    # Udp:
    # StartUdpServer(context, identity=identity, address=("0.0.0.0", 5020))

    # socat -d -d PTY,link=/tmp/ptyp0,raw,echo=0,ispeed=9600 PTY,link=/tmp/ttyp0,raw,echo=0,ospeed=9600
    # Ascii:
    # StartSerialServer(context, identity=identity,
    #                    port='/dev/ttyp0', timeout=1)

    # RTU:
    # StartSerialServer(context, framer=ModbusRtuFramer, identity=identity,
    #                   port='/tmp/ttyp0', timeout=.005, baudrate=9600)

    # Binary
    # StartSerialServer(context,
    #                   identity=identity,
    #                   framer=ModbusBinaryFramer,
    #                   port='/dev/ttyp0',
    #                   timeout=1)
    log.info("Starting server")
    StartTcpServer(context, identity=identity, address=("", 502))

if __name__ == "__main__":
    run_server()
