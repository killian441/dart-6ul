from nio.block.base import Block
from nio.signal.base import Signal
from nio.util.discovery import discoverable
from nio.properties import StringProperty, BoolProperty, IntProperty
from nio.properties import VersionProperty


@discoverable
class Example(Block):

    sProp = StringProperty(title='String', default='')
    bProp = BoolProperty(title='Boolean', default=True)
    iProp = IntProperty(title='Integer', default=1)
    version = VersionProperty('0.0.1')

    def __init__(self):
        """ Create a new block instance.
        Take care of setting up instance variables in your block's constructor.
        Note that the properties of the block are not available when the block
        is created. Those will be available when the block is configured.
        It is normally meaningless to pass variables to the constructor of the
        block. Any data the block requires will be passed through the
        BlockContext when the block is configured.
        """
        super().__init__()

    def configure(self, context):
        """Overrideable method to be called when the block configures.
        The block creator should call the configure method on the parent,
        after which it can assume that any parent configuration options present
        on the block are loaded in as class variables. They can also assume
        that all functional modules in the service process are loaded and
        started.
        Args:
            context (BlockContext): The context to use to configure the block.
        Raises:
            TypeError: If the specified router is not a BlockRouter
        """
        super().configure(context)

    def start(self):
        """Overrideable method to be called when the block starts.
        The block creator can assume at this point that the block's
        initialization is complete and that the service and block router
        are in "starting" state.
        """
        pass  # pragma: no cover

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
            pass
        self.notify_signals(signals)

    def notify_signals(self, signals, output_id=None):
        """Notify signals to router.
        This is the method the block should call whenever it would like
        to "output" signals for the router to send downstream.
        Args:
            signals (list): A list of signals to notify to the router
            output_id: The identifier of the output terminal to notify the
                signals on
        The signals argument is handled as follows:
            - a dictionary is not allowed
            - if a single signal is notified not as an iterable, it will get
            wrapped inside a list before forwarding to block router.
        Raises:
            TypeError: when signals are not instances of class Signal
        """
        pass

    def stop(self):
        """Overrideable method to be called when the block stops.
        The block creator can assume at this point that the service and block
        router are in "stopping" state. All modules are still available for use
        in the service process.
        """
        super().stop()

