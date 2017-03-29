from enum import Enum
from threading import Lock
from nio.block.base import Block
from nio.util.discovery import discoverable
from nio.properties import IntProperty, VersionProperty, SelectProperty, \
    ObjectProperty, PropertyHolder


try:
    from periphery import GPIO
except:
    # Let the block code load anyway so that som unit tests can run.
    pass

class GPIODevice():

    """Communicate with a device over GPIO."""

    def __init__(self, logger):
        self.logger = logger
        self._gpio_lock = Lock()

    def read(self, pin):
        """Read bool value from a pin.

        Args:
            pin (int): the pin to read from

        Return:
            bool: value of digital pin reading

        """
        with self._gpio_lock:
            gpio_pin = GPIO(pin, "in")
            value = gpio_pin.read()
            gpio_pin.close()
            self.logger.debug(
                "Read value from GPIO pin {}: {}".format(pin, value))
        return bool(value)

    def write(self, pin, value):
        """Write bool value to a pin.

        Args:
            pin (int): the pin to write to
            value (bool): boolean value to write to pin

        """
        with self._gpio_lock:
            gpio_pin = GPIO(pin, "out")
            gpio_pin.write(value)
            gpio_pin.close()
            self.logger.debug(
                "Wrote value to GPIO pin {}: {}".format(pin, value))

    def interrupt(self, callback, pin, interrupt_trigger=None, timeout=0):
        """Init interrupt callback function for pin.

        Args:
            callback (function): function to call on interrupt
            pin (int): the pin to monitor for interrupts

        """
        with self._gpio_lock:
            gpio_pin = GPIO(pin, "preserve")
            if gpio_pin.supports_interrupts:
                gpio_pin.direction = "in"
                gpio_pin.edge = interrupt_trigger
                if gpio_pin.poll(timeout):
                    # Enter callback function
                    pass
                gpio_pin.edge = "none" #need to set to none inorder to reuse pin for anything else
            else:
                self.logger.error("GPIO pin {} does not support interrupts".format(pin))
            
            self.logger.debug(
                "Set interrupt callback of GPIO pin {}".format(pin))


    def close(self):
        try:
            pass
        except:
            self.logger.warning("Failed to close GPIO", exc_info=True)
