from nio.block.base import Block
from nio.signal.base import Signal
from nio.util.discovery import discoverable
from nio.properties import StringProperty, BoolProperty, IntProperty
from nio.properties import VersionProperty
from enum import Enum
import time
import sched
import bisect
import re
import serial
import tenacity

#/* ================== Based on Arduino SDI-12 Code =========================
#*/

def debugThis(msg):
  if True:
    print(msg)
  else:
    pass

class CirBuffer:
  def __init__(self,size):
    self._maxsize = size
    self._bufferOverflow = False
    self.data = []

  def append(self,x):
    if len(self.data)+1 > self._maxsize:
      self._bufferOverflow = True
    else:
      self.data.append(x)

  def read(self):
    self._bufferOverflow = False
    try:
      return self.data.pop(0)
    except:
      return -1

  def readline(self):
    self._bufferOverflow = False
    try:
      i = self.data.index(b'\n')+1
      returnValue = self.data[0:i]
      self.data = self.data[i:]
      return returnValue
    except:
      return -1

  def flush(self):
    self.data = []
    self._bufferOverflow = False

  def peek(self):
    try:
      return self.data[0]
    except:
      return -1

  def available(self):
    if(self._bufferOverflow):
      return -1
    else:
      return len(self.data)

  def get(self):
    datacopy = self.data.copy()
    self.data = []
    return datacopy

# Class for the SDI-12 object. 
class SDI12:
  _BUFFER_SIZE = 64           # max RX buffer size    
  SPACING = 830               # bit timing in microseconds

  class SDIState(Enum):
    DISABLED = 0              # value for "DISABLED" state
    ENABLED = 1               # value for "ENABLED" state
    HOLDING = 2               # value for "DISABLED" state
    TRANSMITTING = 3          # value for "TRANSMITTING" state
    LISTENING = 4             # value for "LISTENING" state

  def __init__(self, uartPort, sendMarking=True):
    self._activeObject = False
    self._bufferOverflow = False
    self._rxBuffer = CirBuffer(self._BUFFER_SIZE)  # Buff for incoming
    self._sendMarking = sendMarking
    self.state = self.SDIState.DISABLED
    if isinstance(uartPort, serial.Serial):
      self.uart = uartPort
    else:
      self.uart = serial.Serial(port=None, baudrate=1200,
                                bytesize=serial.SEVENBITS, 
                                parity=serial.PARITY_EVEN, 
                                stopbits=serial.STOPBITS_ONE, 
                                timeout=3.5)
      self.uart.port = uartPort # Uart is not opened automatically if
                                # initialized with port=None and port 
                                # defined elsewhere, see serial docs
    self.setState(self.SDIState.DISABLED)

# ==================== Data Line States ===============================
  #Set state
  def setState(self, state):
    if(state == self.SDIState.HOLDING):
      if not self.uart.is_open:
        self.uart.open()
      self.state = self.SDIState.HOLDING
    elif(state == self.SDIState.TRANSMITTING):
      if not self.uart.is_open:
        self.uart.open()
      self.state = self.SDIState.TRANSMITTING
    elif(state == self.SDIState.LISTENING):
      if not self.uart.is_open:
        self.uart.open()
      self.state = self.SDIState.LISTENING
    else:           # implies state=="DISABLED" 
      if self.uart.is_open:
        self.uart.close()
      self.state = self.SDIState.DISABLED
      self._activeObject = False

  # Force a "HOLDING" state. 
  def forceHold(self):
    self.setState(self.SDIState.HOLDING)

# ===== Constructor, Destructor, SDI12.begin(), and SDI12.end()  ======

  # Begin
  def begin(self): 
    self.setState(self.SDIState.HOLDING)
    self.setActive() 

  # End
  def end(self):
    self.setState(self.SDIState.DISABLED)

# ========== Waking up, and talking to, the sensors. ================== 

  # This function wakes up the entire sensor bus
  def wakeSensors(self):
    self.setState(self.SDIState.TRANSMITTING) 
    if self._sendMarking:
      self.uart.baudrate = 600
      self.uart.write(b'\x00')
      self.uart.baudrate = 1200

  # This function sends out the characters of the String cmd, one by one
  def sendCommand(self, cmd):
    self.wakeSensors()              # Wake up sensors
    self.uart.write(cmd.encode())   # This sends the command as byte array, 
                                    #  since RX is connected to TX we will see
                                    #  command echoed in input buffer
    self.uart.reset_input_buffer()  # Needed, not exactly sure why
    self.listen(1, cmd=cmd)         # 16.7ms is the max time for a response to
                                    #  be received after a command is sent.    
                                    #  However Command gets buffered in the
                                    #  UART so this timing doesn't work
                                    #  perfectly

  # This command reads the UART RX buffer for response
  def listen(self, listenTimeout, cmd=''):
    self.setState(self.SDIState.LISTENING)
    self.uart.timeout = (listenTimeout)   # Listen for upto 'listenTimeout'
                                          #  seconds after each char for a 
                                          #  subsequent char
    dataRaw = [self.uart.read()]
    while dataRaw[-1] is not b'':
      self._rxBuffer.append(dataRaw[-1])
      dataRaw.append(self.uart.read())
    else:
      dataRaw.pop()

    #Here we subtract the cmd from the response, if applicable
    # TODO: If multiple commands are executed before the buffer is depleted
    #  then the command will not be subtracted from the response
    if cmd is not '':
      rxdata = self._rxBuffer.get()
      for x in range(0,2):
        if b''.join(rxdata[x:len(cmd)+x]) == cmd.encode():
          rxdata = rxdata[len(cmd)+x:]
          break
      for datum in rxdata:
        self._rxBuffer.append(datum)

# ============ Reading from the SDI-12 object buffer.  ================

  # Reveals the number of characters available in the buffer
  def available(self):
    return self._rxBuffer.available()

  # Reveals the next character in the buffer without consuming
  def peek(self):
    return self._rxBuffer.peek()    

  # A public function that clears the buffer contents and
  # resets the status of the buffer overflow. 
  def flush(self):
    self._rxBuffer.flush()

  # Reads in the next character from the buffer (and moves the index ahead)
  def read(self):
    return self._rxBuffer.read()

# ============= Using more than one SDI-12 object.  ===================

  # A method for setting the current object as the active object
  def setActive(self):
    if (self._activeObject != True):
      self.setState(self.SDIState.HOLDING) 
      self._activeObject = True
      return True
    return False

  # A method for checking if this object is the active object
  def isActive(self):
    return self._activeObject


# /* ========== AquaCheck Soil Moisture Probe =========
#  * Mike Killian, 2017
#  *
#  *  TODO: More robust retries, move error logging to n.io.
#  */


class SDI12AquaCheck:
  RETRIES = 2   # Number of retries before giving up on data from the probe
# /*
#  *  Constructor:
#  * 
#  *    dataBus     - uart used for the SDI12 bus
#  *    sendMarking - issue marking to wake up sensors at start?
#  *    rs485       - using hardware rs485?
#  */ 
  def __init__(self, dataBus, sendMarking=True, rs485=False):
    self.dataBus = serial.Serial(port=None, baudrate=1200,
                                 bytesize=serial.SEVENBITS, 
                                 parity=serial.PARITY_EVEN, 
                                 stopbits=serial.STOPBITS_ONE, 
                                 timeout=3.5)
    self.dataBus.port    = dataBus
    if rs485:
      self.dataBus.rs485_mode = serial.rs485.RS485Settings()
    self.moistureRaw     = ""     # Raw response of probe with all sensors
    self.moistureData    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.temperatureRaw  = ""
    self.temperatureData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.aquaCheckSDI12  = SDI12(self.dataBus, sendMarking=sendMarking)
    self.aquaCheckSDI12.begin()

# /* 
#  *  pollProbe(readingType, address) - samples the AquaCheck sensor
#  *
#  *    readingType - 0 for moisture readings, 1 for temperature readings
#  *    address     - address of sensor to read from, default is '0'
#  *
#  *  The AquaCheck Moisture Probe is polled for either Moisture or 
#  *  Temperature readings depending on readingType. The address allows for
#  *  more than one sensor to be connected to the same bus. (Sensors must 
#  *  have unique addresses when connected to the bus.) Data is stored in 
#  *  internal variables that can then be read. Moisture and temperature data
#  *  are stored independently and overwritten when a new poll request is
#  *  made. The function returns 0 for a successful run, any other value
#  *  represents an error.
#  *
#  *  Returned errors:
#  *    1 - Incorrect usage / Unknown failure
#  *    2 - Incorrect or no response received from the AquaCheck
#  *    3 - Sensor reading was aborted by the AquaCheck
#  *    4 - Sensor set 0 failed in some way
#  *    5 - Sensor set 1 failed in some way
#  *    
#  */
  def pollProbe(self, readingType, address = '0'):
    sdiCommand = ""
    self.sdiResponse   = ""
    self.dataResponse  = ["","","","","",""]
    self.sdiAddress      = -1  # Address of the sensor responding
    self.sdiTimeToCheck  = -1  # Time to wait before requesting data
    self.sdiMeasurements = -1  # Number of measurements expected
    
    self.aquaCheckSDI12.flush()

    if(readingType == 0):
      self.moistureRaw = ""
      self.moistureData = [0.0]*6
      sdiCommand = str(address) + "M0!"
    elif(readingType == 1):
      self.temperatureRaw = ""
      self.temperatureData = [0.0]*6
      sdiCommand = str(address) + "M1!"
    else:
      return -1

    try:
      self._issueCommand(sdiCommand)
    except tenacity.RetryError:
      debugThis("Error issuing command")
      debugThis(self._issueCommand.retry.statistics)
      return -1

    try:
      self._gatherData(readingType)
    except tenacity.RetryError:
      debugThis("Error gathering data")
      debugThis(self._gatherFirstData.retry.statistics)
      debugThis(self._gatherSecondData.retry.statistics)
      debugThis(self._issueFirstData.retry.statistics)
      debugThis(self._issueSecondData.retry.statistics)
      return -1

    self.aquaCheckSDI12.flush()

    return 0

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _issueCommand(self, sdiCommand):
    self.sdiResponse = "" 
    self.aquaCheckSDI12.sendCommand(sdiCommand)

    while(self.aquaCheckSDI12.available()):
      self.sdiResponse += self.aquaCheckSDI12.read().decode()

    # break response into corrisponding components
    try:
      if (len(self.sdiResponse) >= 7 and 
              self.sdiResponse[-2] == '\r' and 
              self.sdiResponse[-1] == '\n'):
        self.sdiMeasurements = int(self.sdiResponse[-3])
        self.sdiTimeToCheck = int(self.sdiResponse[-6:-3])
        self.sdiAddress = self.sdiResponse[-7]
      else:
        debugThis("_issueCommand did not get a good response: "
                  "{}".format(self.sdiResponse))
        raise tenacity.TryAgain  
    except Exception as err:
      debugThis("_issueCommand failed somehow: {}".format(err))
      raise tenacity.TryAgain  

    # Wait for interrupt to notify us data is ready
    timestamp = time.perf_counter()
    while(time.perf_counter() - timestamp <= self.sdiTimeToCheck):
      self.sdiResponse = ""
      self.aquaCheckSDI12.listen(0.1)
      while(self.aquaCheckSDI12.available()):
        self.sdiResponse += self.aquaCheckSDI12.read().decode()
      try:
        if (self.sdiResponse[0] == self.sdiAddress and 
            self.sdiResponse[1] == '\r' and 
            self.sdiResponse[2] == '\n'):
          break  # Upon receipt of a service request drop out of for loop
      except IndexError:
        pass

  def _gatherData(self, readingType):
    try:
      self._issueFirstData()
    except:
      debugThis("_gatherData failed, returned: {}".format(self.sdiResponse))
    try:
      self._issueSecondData()
    except:
      debugThis("_gatherData failed, returned: {}".format(self.sdiResponse))

    try:
      if(readingType == 0):  # 0 for Moisture, 1 for Temperature
        self.moistureRaw = self.sdiAddress
        for i in range(0,6):
          self.moistureData[i] = float(self.dataResponse[i])
          self.moistureRaw += "+{}".format(self.dataResponse[i])
      elif(readingType == 1):
        self.temperatureRaw = self.sdiAddress
        for i in range(0,6):
          self.temperatureData[i] = float(self.dataResponse[i])
          self.temperatureRaw += "+{}".format(self.dataResponse[i])
      else:
        return -1
    except Exception as err:
      debugThis("Error assigning values, {}".format(err))
      self.moistureData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.moistureRaw  = ''
      self.temperatureData = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.temperatureRaw  = ''

    self.aquaCheckSDI12.flush()

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _issueFirstData(self):
    self.aquaCheckSDI12.flush()
    self.aquaCheckSDI12.sendCommand("0D0!")  # ask for data from set 0
    self.sdiResponse = ""

    while(self.aquaCheckSDI12.available()):  # build a string of the response
      self.sdiResponse += self.aquaCheckSDI12.read().decode()

    try:
      if (len(self.sdiResponse) > 3):
        self.dataResponse[0] = self.sdiResponse[2:10]
        self.dataResponse[1] = self.sdiResponse[11:19]
        self.dataResponse[2] = self.sdiResponse[20:28]
        if ((self.sdiResponse[-2] != '\r' and self.sdiResponse[-1] != '\n') or
             self.dataResponse[0][0] == '\0' or 
             self.dataResponse[1][0] == '\0' or 
             self.dataResponse[2][0] == '\0'):
          debugThis("_issueFirstData bad response: {}".format(self.sdiResponse))
          raise tenacity.TryAgain
      elif (self.sdiResponse[0] == self.sdiAddress and 
            self.sdiResponse[1] == '\r' and 
            self.sdiResponse[2] == '\n'):
        debugThis("_issueFirstData null response")
        raise tenacity.TryAgain                   
      else:
        debugThis("_isseFirstData no response")
        raise tenacity.TryAgain  
    except Exception as err:
      debugThis("_issueFirstData failed somehow: {}".format(err))
      raise tenacity.TryAgain

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _issueSecondData(self):
    self.aquaCheckSDI12.flush()
    self.aquaCheckSDI12.sendCommand("0D1!")  # ask for data from set 1 
    self.sdiResponse = ""

    while(self.aquaCheckSDI12.available()):  # build a string of the response
      self.sdiResponse += self.aquaCheckSDI12.read().decode()

    try:
      if (len(self.sdiResponse) > 3):
        self.dataResponse[3] = self.sdiResponse[2:10]
        self.dataResponse[4] = self.sdiResponse[11:19]
        self.dataResponse[5] = self.sdiResponse[20:28]
        if ((self.sdiResponse[-2] != '\r' and self.sdiResponse[-1] != '\n') or
             self.dataResponse[3][0] == '\0' or 
             self.dataResponse[4][0] == '\0' or 
             self.dataResponse[5][0] == '\0'):
          raise tenacity.TryAgain
      elif (self.sdiResponse[0] == self.sdiAddress and 
            self.sdiResponse[1] == '\r' and 
            self.sdiResponse[2] == '\n'):
        raise tenacity.TryAgain                     
      else:
        raise tenacity.TryAgain                   
    except:
      raise tenacity.TryAgain


@discoverable
class AquaCheck(Block):

    signalName = StringProperty(title='Signal Name', default='default')
    portNumber = StringProperty(title='UART Port', default='/dev/ttymxc4')
    sendMarking = BoolProperty(default=False, title='Send Marking')
    rs485 = BoolProperty(default=False, title='Hardware RS485 Port')
    version = VersionProperty('0.0.1')

    def configure(self,context):
        super().configure(context)
        self.logger.debug("Got here with {}".format(self.portNumber()))
        self.AQ = SDI12AquaCheck(self.portNumber(), 
                                 sendMarking=self.sendMarking(), 
                                 rs485=self.rs485())

    def process_signals(self, signals):
        for signal in signals:
            if self.AQ.pollProbe(0) == 0:
                #TODO: Add polling for temperature
                value = self.AQ.moistureData
                results = {self.signalName():value}
                self.logger.debug("Got results: {}".format(results))
                try:
                    self.notify_signals([Signal(results)])
                except:
                    self.logger.exception("Signal is not valid:"
                                          " {}".format(results))

