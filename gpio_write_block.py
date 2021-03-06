from enum import Enum
from threading import Lock
from nio.block.base import Block
from nio.util.discovery import discoverable
from nio.properties import IntProperty, VersionProperty, Property
from .gpio_device import GPIODevice


@discoverable
class GPIOWrite(Block):

    pin = IntProperty(default=0, title="Pin Number")
    value = Property(title='Write Value', default="{{ False }}")
    version = VersionProperty('0.1.0')

    def __init__(self):
        super().__init__()
        self._gpio = None

    def configure(self, context):
        super().configure(context)
        self._gpio = GPIODevice(self.logger)

    def stop(self):
        self._gpio.close()
        super().stop()

    def process_signals(self, signals):
        for signal in signals:
            self._write_gpio_pin(self.pin(signal), self.value(signal))
        self.notify_signals(signals)

    def _write_gpio_pin(self, pin, value):
        try:
            return self._gpio.write(pin, value)
        except:
            self.logger.warning(
                "Failed to write value {} to gpio pin: {}".format(value, pin),
                exc_info=True)
