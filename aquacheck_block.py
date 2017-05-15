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
      return -1;
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
    self.uart = serial.Serial(port=None, baudrate=1200,
                              bytesize=serial.SEVENBITS, 
                              parity=serial.PARITY_EVEN, 
                              stopbits=serial.STOPBITS_ONE, 
                              timeout=3.5)
    self.uart.port = uartPort  # Uart is not opened automatically if
                               # initialized with port=None and port 
                               # defined elsewhere, see serial documentation
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
    self.setState(self.SDIState.TRANSMITTING); 
    if self._sendMarking:
      self.uart.baudrate = 600
      self.uart.write(b'\x00')
      self.uart.baudrate = 1200

  # This function sends out the characters of the String cmd, one by one
  def sendCommand(self, cmd):
    self.wakeSensors();             # Wake up sensors
    self.uart.write(cmd.encode())   # This sends the command as byte array, 
                                    #  since RX is connected to TX we will see
                                    #  command echoed in input buffer
    self.uart.reset_input_buffer()  # Needed, not exactly sure why
    self.listen(1, cmd=cmd);        # 16.7ms is the max time for a response to
                                    #  be received after a command is sent.    
                                    #  However Command gets buffered in the
                                    #  UART so this timing doesn't work
                                    #  perfectly

  # This command reads the UART RX buffer for response
  def listen(self, listenTimeout, cmd=''): # Time to wait in seconds
    self.setState(self.SDIState.LISTENING)
    self.uart.timeout = (listenTimeout)     # Listen for upto 'listenTimeout'
                                            #  seconds after each char for a 
                                            #  subsequent char
    dataRaw = [self.uart.read()]
    while dataRaw[-1] is not b'':
      self._rxBuffer.append(dataRaw[-1])
      dataRaw.append(self.uart.read())
    else:
      dataRaw.pop()

    #Here we subtract the cmd from the response, if applicable
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
      self.setState(self.SDIState.HOLDING); 
      self._activeObject = True;
      return True;
    return False;

  # A method for checking if this object is the active object
  def isActive(self):
    return self._activeObject


# /* ========== AquaCheck Soil Moisture Probe =========
#  * Mike Killian, 2017
#  *
#  *  TODO: More robust retries, for example not bailing is less than 3 chars is returned before retrying.
#  */


class SDI12AquaCheck:
  RETRIES = 2                   #// Define number of retries before giving up on data from the probe       
# /*
#  *  Constructor - dataPin is the Arduino pin connected to the SDI12 bus
#  */ 
  def __init__(self, dataPin):
    self.dataPin = dataPin
    self.sdiMoisture = ""          #// Six 7 digit (8 char) numbers starting with and seperated by a + sign (1 additional char)
    self.sdiTemperature = ""
    self.dataMoisture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.dataTemperature = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    self.aquaCheckSDI12 = SDI12(self.dataPin)
    self.aquaCheckSDI12.begin()

# /* 
#  *  pollProbe(readingType, address) - samples the AquaCheck sensor
#  *
#  *    readingType - 0 for moisture readings, 1 for temperature readings
#  *    address     - address of sensor to read from, valid addresses are 0-9, a-z, & A-Z
#  *
#  *  The AquaCheck Moisture Probe is polled for either Moisture or Temperature readings depending
#  *  on readingType. The address allows for more than one sensor to be connected to the same bus. 
#  *  (Sensors must have unique addresses when connected to the bus) Data is stored in internal 
#  *  variables that can then be read. Moisture and temperature data are stored independently and 
#  *  overwritten when a new poll request is made. The function returns 0 for a successful run, any 
#  *  other value represents an error.
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
    sdiCommand = ""         #// String to send, largest command that can be sent is aMC1! (crc not implemented in aquaCheck)
    self.sdiResponse = ""       #// String for generic responses, max value in response to a D command (non concurrent) is 35
    self.dataResponse0 = ""      #// Due to a compiler bug these can't all be in an array and used with sscanf
    self.dataResponse1 = ""      #// Also Arduino sscanf doesn't support floats, so either I mess with the compiler
    self.dataResponse2 = ""      #// Or I use atof on a char array
    self.dataResponse3 = ""    
    self.dataResponse4 = ""
    self.dataResponse5 = ""
    self.sdiAddress = -1            #// Address of the sensor responding, easier to work with as a char vs as an int
    self.sdiTimeToCheck = -1         #// Time to wait before requesting data, in seconds
    self.sdiMeasurements = -1        #// Number of measurements expected, should always be 6 for AquaCheck

    retries = self.RETRIES           #// Number of retries if error is detected in response
    
    self.aquaCheckSDI12.flush();  #//clear the line

    if(readingType == 0):
      self.sdiMoisture = ""
      sdiCommand = str(address) + "M0!"
      self.dataMoisture = [0.0]*6
    elif(readingType == 1):
      self.sdiTemperature = ""
      sdiCommand = str(address) + "M1!"
      self.dataTemperature = [0.0]*6
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

    self.aquaCheckSDI12.flush();  #//clear the line

    return 0;

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _issueCommand(self, sdiCommand):
    self.sdiResponse = "" 
    self.aquaCheckSDI12.sendCommand(sdiCommand);      #// command SDI-12 Moisture Probe to take a reading of all it's sensors 

    while(self.aquaCheckSDI12.available()):
      self.sdiResponse += self.aquaCheckSDI12.read().decode();

     #//break response into corrisponding components
    try:
      if (len(self.sdiResponse) >= 7 and self.sdiResponse[-2] == '\r' and self.sdiResponse[-1] == '\n'):   
        self.sdiMeasurements = int(self.sdiResponse[-3])
        self.sdiTimeToCheck = int(self.sdiResponse[-6:-3])
        self.sdiAddress = self.sdiResponse[-7]
      else:
        debugThis("_issueCommand did not get a good response")
        raise tenacity.TryAgain  
    except:
      debugThis("_issueCommand failed somehow")
      raise tenacity.TryAgain  

    timestamp = time.perf_counter()
    while(time.perf_counter() - timestamp <= self.sdiTimeToCheck):
      # /* With the new polling flow of the SDI12 library this needs to be reworked to call it
      #  * right now we just drop through after the timeout because no interrupts. Technically it 
      #  * still works, but it would be more responsive if we update it
      #  */
      self.sdiResponse = ""

      while(self.aquaCheckSDI12.available()):
        self.sdiResponse += self.aquaCheckSDI12.read().decode();
      try:
        if (self.sdiResponse[0] == self.sdiAddress and self.sdiResponse[1] == '\r' and self.sdiResponse[2] == '\n'):
          break;                             #// Upon receipt of a service request drop out of for loop 
      except IndexError:
        pass

  def _gatherData(self, readingType):
    dataBackup = [[], [], [], [], [], []]
    try:
      self._gatherFirstData(dataBackup)
    except:
      debugThis("_gatherFirstData failed, returned: {}".format(dataBackup))
      dataBackup[0:3] = [[0.0],[0.0],[0.0]]
    try:
      self._gatherSecondData(dataBackup)
    except:
      debugThis("_gatherSecondData failed, returned: {}".format(dataBackup))
      dataBackup[3:6] = [[0.0],[0.0],[0.0]]

    try:
      if(readingType == 0):            #// 0 for Moisture, 1 for Temperature
        self.sdiMoisture = self.sdiAddress
        for i,x in enumerate(dataBackup):
          y = max(set(x),key=x.count)
          self.dataMoisture[i] = float(y)
          self.sdiMoisture += "+" + y
      elif(readingType == 1):
        self.sdiTemperature = self.sdiAddress
        for i,x in enumerate(dataBackup):
          y = max(set(x),key=x.count)
          self.dataTemperature[i] = float(y)
          self.sdiTemperature += "+" + y
      else:
        return -1;
    except:
      debugThis("Error assigning values")
      self.dataMoisture = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.sdiMoisture = ''
      self.dataTemperature = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      self.sdiTemperature = ''

    self.aquaCheckSDI12.flush()

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _gatherFirstData(self, dataBackup):
    self._issueFirstData()
    dataBackup[0:3] = [[self.dataResponse0], [self.dataResponse1], [self.dataResponse2]]
#    r = [re.sub(r'[^0-9\.]','0',self.dataResponse0), re.sub(r'[^0-9\.]','0',self.dataResponse1), re.sub(r'[^0-9\.]','0',self.dataResponse2)]
#    if len(dataBackup[0])<3 or len(dataBackup[1])<3 or len(dataBackup[2])<3:
#      for i,x in enumerate(r):
#        try:
#          if float(x) < 100.0 and float(x) > 0.0 and len(x) == 8:
#            dataBackup[i].append(x)
#        except ValueError:
#          pass
#      raise tenacity.TryAgain
    
  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _gatherSecondData(self, dataBackup):
    self._issueSecondData()
    dataBackup[3:6] = [[self.dataResponse3], [self.dataResponse4], [self.dataResponse5]]
#    r = [re.sub(r'[^0-9\.]','0',self.dataResponse3), re.sub(r'[^0-9\.]','0',self.dataResponse4), re.sub(r'[^0-9\.]','0',self.dataResponse5)]
#    if len(dataBackup[3])<3 or len(dataBackup[4])<3 or len(dataBackup[5])<3:
#      for i,x in enumerate(r):
#        try:
#          if float(x) < 100.0 and float(x) > 0.0 and len(x) == 8:
#            dataBackup[i+3].append(x)
#        except ValueError:
#          pass
#      raise tenacity.TryAgain

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _issueFirstData(self):
    self.aquaCheckSDI12.flush();                #// clear the line
    self.aquaCheckSDI12.sendCommand("0D0!");    #// ask for data from set 0
    self.sdiResponse = ""

    while(self.aquaCheckSDI12.available()):     #// build a string of the response
      self.sdiResponse += self.aquaCheckSDI12.read().decode();

    try:
      if (len(self.sdiResponse) > 3):
        self.dataResponse0 = self.sdiResponse[2:10]
        self.dataResponse1 = self.sdiResponse[11:19]
        self.dataResponse2 = self.sdiResponse[20:28]
        if ((self.sdiResponse[-2] != '\r' and self.sdiResponse[-1] != '\n') or self.dataResponse0[0] == '\0' or self.dataResponse1[0] == '\0' or self.dataResponse2[0] == '\0'):
          debugThis("_issueFirstData bad response")
          raise tenacity.TryAgain
      elif (self.sdiResponse[0] == self.sdiAddress and self.sdiResponse[1] == '\r' and self.sdiResponse[2] == '\n'):
        debugThis("_issueFirstData null response")
        raise tenacity.TryAgain                   
      else:
        debugThis("_isseFirstData no response")
        raise tenacity.TryAgain  
    except:
      debugThis("_issueFirstData failed somehow")
      raise tenacity.TryAgain

  @tenacity.retry(stop=tenacity.stop_after_attempt(RETRIES))
  def _issueSecondData(self):
    self.aquaCheckSDI12.flush();                #// clear the line
    self.aquaCheckSDI12.sendCommand("0D1!");    #// ask for data from set 1 
    self.sdiResponse = ""

    while(self.aquaCheckSDI12.available()):     #// build a string of the response
      self.sdiResponse += self.aquaCheckSDI12.read().decode();

    try:
      if (len(self.sdiResponse) > 3):
        self.dataResponse3 = self.sdiResponse[2:10]
        self.dataResponse4 = self.sdiResponse[11:19]
        self.dataResponse5 = self.sdiResponse[20:28]
        if ((self.sdiResponse[-2] != '\r' and self.sdiResponse[-1] != '\n') or self.dataResponse3[0] == '\0' or self.dataResponse4[0] == '\0' or self.dataResponse5[0] == '\0'):
          raise tenacity.TryAgain
      elif (self.sdiResponse[0] == self.sdiAddress and self.sdiResponse[1] == '\r' and self.sdiResponse[2] == '\n'):
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
        """Overrideable method to be called when the block configures.
        """
        super().configure(context)
        self.logger.debug("Got here with {}".format(self.portNumber()))
        self.AQ = SDI12AquaCheck(self.portNumber())

    def process_signals(self, signals):
        """Overrideable method to be called when signals are delivered.
        This method will be called by the block router whenever signals
        are sent to the block. The method should not return the modified
        signals, but rather call `notify_signals` so that the router
        can route them properly.
        Args:
            signals (list): A list of signals to be processed by the block
            input_id: The identifier of the input terminal the signals are
                being delivered to
        """
        for signal in signals:
            if self.AQ.pollProbe(0) == 0:
                #value = self.AQ.sdiMoisture
                value = self.AQ.dataMoisture
                results = {self.signalName():value}
                self.logger.debug("Got results: {}".format(results))
                try:
                    self.notify_signals([Signal(results)])
                except:
                    self.logger.exception("Signal is not valid:"
                                          " {}".format(results))

