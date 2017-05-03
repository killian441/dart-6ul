from nio.block.base import Block
from nio.signal.base import Signal
from nio.util.discovery import discoverable
from nio.properties import StringProperty, BoolProperty, IntProperty
from nio.properties import VersionProperty
import time
from periphery import GPIO
import sched
import bisect
import re
import serial
import tenacity

#/* ======================== Arduino SDI-12 =================================
#*/



#/* =========== 1. Buffer Setup ============================================
#*/

# Helper Functions:
#parity_is_odd - returns 0 if even number of bits, 1 if odd number
def parity_isOdd(int_type):
  parity = 0
  while (int_type):
    parity = ~parity
    int_type = int_type & (int_type - 1)
  return(parity&1)

def delayMicroseconds(microDelay):
  start = time.perf_counter()
  _microDelay = microDelay / 1000000.0
  now = time.perf_counter()
  while (now-start < _microDelay):
    now = time.perf_counter()

def debugThis(msg):
  if True:
    print(msg)
  else:
    pass



# /* =========== 2. Data Line States ===============================
# */

#  # #// 2.1 - sets the state of the SDI-12 object. 
class SDI12:
  _BUFFER_SIZE = 64         # # #// 0.2 max RX buffer size    
  SPACING = 830             # # #// 0.8 bit timing in microseconds
  POLLING = 1               # # #// 0.9 to poll or not to poll?

  DISABLED = 0              # # #// 0.3 value for "DISABLED" state
  ENABLED = 1               # # #// 0.4 value for "ENABLED" state
  HOLDING = 2               # # #// 0.5 value for "DISABLED" state
  TRANSMITTING = 3          # # #// 0.6 value for "TRANSMITTING" state
  LISTENING = 4             # # #// 0.7 value for "LISTENING" state

  def __init__(self, uartPort):
    self._activeObject = False
    self._bufferOverflow = False
    self.uart = serial.Serial(port=None, baudrate=1200, bytesize=serial.SEVENBITS, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE, timeout=3.5)
    self.uart.port = uartPort  # Uart is not opened if initialized with port=None and port defined elsewhere, see serial documentation
    self.state = "DISABLED"
    self._rxBuffer = [0]*self._BUFFER_SIZE   # # #// 1.2 - buff for incoming
    self._rxBufferHead = 0              # # #// 1.3 - index of buff head
    self._rxBufferTail = 0              # # #// 1.4 - index of buff tail
    self.setState("DISABLED")

  def setState(self, state):
    state = state.upper()
    if(state == "HOLDING"):
      #set dataPin to output and low
      if not self.uart.is_open:
        self.uart.open()
      self.state = "HOLDING"
    elif(state == "TRANSMITTING"):
      #set dataPin to output, prevent interrupts
      if not self.uart.is_open:
        self.uart.open()
      self.state = "TRANSMITTING"
    elif(state == "LISTENING"):
      #set dataPin low and input
      if not self.uart.is_open:
        self.uart.open()
      self.state = "LISTENING"
    else:            # # #// implies state=="DISABLED" 
      #set dataPin low and input
      if self.uart.is_open:
        self.uart.close()
      self.state = "DISABLED"

# # #// 2.2 - forces a "HOLDING" state. 
  def forceHold(self):
    self.setState("HOLDING")

# /* ======= 3. Constructor, Destructor, SDI12.begin(), and SDI12.end()  =======
# */

# # #//  3.3 Begin
  def begin(self): 
    self.setState("HOLDING")
    self.setActive() 

# # #//  3.4 End
  def end(self):
    self.setState("DISABLED")

# /* ============= 4. Waking up, and talking to, the sensors. ===================
# */ 

#  # #// 4.1 - this function wakes up the entire sensor bus
  def wakeSensors(self):
    self.setState("TRANSMITTING"); 
    self.uart.baudrate = 600
    self.uart.write(b'\x00')
    self.uart.baudrate = 1200

#  # #// 4.2 - this function writes a character out on the data line
  def genChar(self,out):
    #Unused now that we are using a uart
    pass

# # #// 4.3 - this function sends out the characters of the String cmd, one by one
  def sendCommand(self, cmd):
    self.wakeSensors();                         # # #// wake up sensors
    self.uart.write(cmd.encode())   # This sends the command as byte array, since RX is connected to TX we will see command echoed in input buffer
    self.uart.reset_input_buffer()  # Ideally this flushes echoed command from input buffer, but there is a delay and it probably doesn't

    self.listen(16700, cmd=cmd);  #  # #// 16.7ms is the max time for a response to be received 
                         #  # #//    after a command is sent    
                         # However Command gets buffered in the UART so this timing doesn't work perfectly

# /* Here is polling code
#   This command will poll the bus for data coming back rather than use interrupts
# */
  def listen(self, listenTimeout, cmd=''): # # #//time to wait in microseconds
    self.setState("LISTENING")
    self.uart.timeout = (1)     # Listen for upto 1 second after each char for a subsequent char

    self.uart.read(len(cmd)+1)  # Throw out echoed command (Additional chars will still be stored in RX buffer)
    dataRaw = [self.uart.read()]
    while dataRaw[-1] is not b'':
      if ((self._rxBufferTail + 1) % self._BUFFER_SIZE == self._rxBufferHead):  #Buffer full?
        self._bufferOverflow = True; 
      else:              # #// 7.2.8 - Save char, advance tail. 
        self._rxBuffer[self._rxBufferTail] = dataRaw[-1] 
        self._rxBufferTail = (self._rxBufferTail + 1) % self._BUFFER_SIZE;
      dataRaw.append(self.uart.read())
    else:
      dataRaw.pop()

    
#  # #// Reads a new character. 
  def receiveChar(self):
  #Unused now that we are using a UART
    pass

# /* ============= 5. Reading from the SDI-12 object.  ===================
# */

#  # #// 5.1 - reveals the number of characters available in the buffer
  def available(self):
    if(self._bufferOverflow):
      return -1; 
    return (self._rxBufferTail + self._BUFFER_SIZE - self._rxBufferHead) % self._BUFFER_SIZE;

#  # #// 5.2 - reveals the next character in the buffer without consuming
  def peek(self):
    if (self._rxBufferHead == self._rxBufferTail):
      return -1;     # # #// Empty buffer? If yes, -1
    return self._rxBuffer[self._rxBufferHead];              # # #// Otherwise, read from "head"

#  # #// 5.3 - a public function that clears the buffer contents and
#  # #// resets the status of the buffer overflow. 
  def flush(self):
    self._rxBufferHead = 0
    self._rxBufferTail = 0;
    self._bufferOverflow = False; 

#  # #// 5.4 - reads in the next character from the buffer (and moves the index ahead)
  def read(self):
    self._bufferOverflow = False;           # # #//reading makes room in the buffer
    if (self._rxBufferHead == self._rxBufferTail):
      return -1;     # # #// Empty buffer? If yes, -1
    nextChar = self._rxBuffer[self._rxBufferHead];  # # #// Otherwise, grab char at head
    self._rxBufferHead = (self._rxBufferHead + 1) % self._BUFFER_SIZE;      # # #// increment head
    return nextChar;                                        # # #// return the char

# /* ============= 6. Using more than one SDI-12 object.  ===================
# */

#  # #// 6.1 - a method for setting the current object as the active object
  def setActive(self):
    if (self._activeObject != True):
      self.setState("HOLDING"); 
      self._activeObject = True;
      return True;
    return False;

#  # #// 6.2 - a method for checking if this object is the active object
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

