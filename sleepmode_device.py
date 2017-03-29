from nio.block.base import Block
from nio.util.discovery import discoverable
from nio.signal.base import Signal
from nio.properties import Property, IntProperty, StringProperty, \
    VersionProperty
from subprocess import call

@discoverable
class LowPowerSleepMode(Block):

    rtcdevice = StringProperty(title='RTC Device', default='1')
    sleeptime = IntProperty(title='Sleep Time (in seconds)', default=10)
    version = VersionProperty('0.0.1')

    """Upon receipt of a signal, sleep"""

    def process_signals(self, signals):
        try:
            rtc_device = self.rtcdevice().strip('rtc')
            call(['rtcwake','-m','mem','-d','rtc'+rtc_device,'-s',str(self.sleeptime())])
        except:
            pass
        self.notify_signals([Signal({'sleeptime':self.sleeptime()})])
