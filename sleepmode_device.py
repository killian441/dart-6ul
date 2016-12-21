from enum import Enum
from threading import Lock
from nio.block.base import Block
from nio.util.discovery import discoverable
from nio.signal.base import Signal
from nio.properties import Property, PropertyHolder, StringProperty, \
    IntProperty, BoolProperty, ListProperty, SelectProperty, ObjectProperty \
    VersionProperty
from nio.util.threading.spawn import spawn
from time import sleep
from subprocess import call


class LowPowerSleepMode(Block):

    rtcdevice = StringProperty(title='RTC Device', default='1')
    sleeptime = IntProperty(title='Sleep Time (in seconds)', default=10)
    version = VersionProperty('0.0.1')

    """Upon receipt of a signal, sleep"""

    def process_signals(self, signals):
        try:
            call(['rtcwake','-m mem','-d ',self.rtcdevice(),'-s',self.sleeptime()])
        except:
            pass