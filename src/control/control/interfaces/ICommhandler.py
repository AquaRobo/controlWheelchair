from zope.interface import Interface

class ICommhandler(Interface):
    def __init__(self, config: dict):
        """Initialize the communication handler with the given configuration."""

    def sendData(self, data: str) -> None:
        """Send data using the communication protocol."""
        pass

    def receiveData(self) -> str:
        """Receive data using the communication protocol."""
        pass