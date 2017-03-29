from enum import Enum
from threading import Lock
from nio.block.base import Block
from nio.signal.base import Signal
from nio.util.discovery import discoverable
from nio.properties import IntProperty, VersionProperty, SelectProperty, \
    ObjectProperty, PropertyHolder
from .gpio_device import GPIODevice


class TriggerOptions(Enum):
    T_RISING = 'rising'
    T_FALLING = 'falling'
    T_BOTH = 'both'


class Trigger(PropertyHolder):
    default = SelectProperty(TriggerOptions,
                             title="Interrupt Trigger",
                             default=TriggerOptions.T_BOTH)
    # TODO: add ability to select base on pin number


@discoverable
class GPIOInterrupts(Block):

    pin = IntProperty(default=0, title="Pin Number")
    version = VersionProperty('0.1.0')
    interrupt_trigger = ObjectProperty(Trigger,
                                  title="Trigger on which edge:",
                                  default=Trigger())

    def __init__(self):
        super().__init__()
        self._gpio = None

    def configure(self, context):
        super().configure(context)
        self._gpio = GPIODevice(self.logger)
        # TODO: allow more than one pin to be configured per block
        self._gpio.interrupt(
            self._callback, self.pin(), self.interrupt_trigger().default().value)

    def stop(self):
        self._gpio.close()
        super().stop()

    def process_signals(self, signals):
        pass

    def _callback(self, channel):
        self.logger.debug(
            "Interrupt callback invoked by pin: {}".format(channel))
        self.notify_signals(Signal({"pin": channel}))
