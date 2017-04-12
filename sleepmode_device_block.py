from nio.block.base import Block
from nio.util.discovery import discoverable
from nio.signal.base import Signal
from nio.properties import Property, IntProperty, StringProperty, \
    VersionProperty
from subprocess import call, check_call, CalledProcessError
import sys
import time

@discoverable
class LowPowerSleepMode(Block):

    rtcdevice = StringProperty(title='RTC Device', default='1')
    sleeptime = IntProperty(title='Sleep Time (in seconds)', default=10)
    version = VersionProperty('0.0.1')

    """Upon receipt of a signal, sleep"""

    def process_signals(self, signals):
        t = time.time()
        try:
            rtc_device = self.rtcdevice().strip('rtc')
            call(['rtcwake','-m','mem','-d','rtc'+rtc_device,'-s',str(self.sleeptime())])
            try:
                check_call(['hwclock','--hctosys'])
            except CalledProcessError as err:
                self.logger.warning("An error occured while resetting the clock: {}".format(err))
        except:
            self.logger.exception("An error occurred while trying to sleep: {}".format(sys.exc_info()[0]))
        t = time.time() - t
        self.notify_signals([Signal({'sleeptime':t})])
