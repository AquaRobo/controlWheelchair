from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler

@implementer(ICommhandler)
class SPIHandler:
    def __init__(self, spi_config: dict):
        pass

    def sendData(self, data: str) -> None:
        # Implementation for sending data via SPI
        pass

    def receiveData(self) -> str:
        # Implementation for receiving data via SPI
        pass