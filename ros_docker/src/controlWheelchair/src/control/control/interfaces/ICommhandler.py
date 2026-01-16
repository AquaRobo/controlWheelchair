from zope.interface import Interface

class ICommhandler(Interface):
    def __init__(self, config: dict):
        """Initialize the communication handler with the given configuration."""

    def sendData(self, data: list) -> None:
        """Send data using the communication protocol."""
        pass

    def receiveData(self) -> list:
        """Receive data using the communication protocol."""
        pass