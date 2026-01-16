from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler

@implementer(ICommhandler)
class SPIHandler:
    def __init__(self, spi_config: dict):
        pass

    def sendData(self, data: list) -> None:
        # Implementation for sending data via SPI
        pass

    def receiveData(self) -> list:
        # Implementation for receiving data via SPI
        pass