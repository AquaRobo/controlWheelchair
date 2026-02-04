import spidev
from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler
from control.exceptions.CommInitException import CommInitError
from control.exceptions.CommCloseException import CommCloseError
from control.exceptions.CommReadException import CommReadError
from control.exceptions.CommWriteException import CommWriteError

@implementer(ICommhandler)
class SPIHandler:
    """Wrapper around spidev python library."""
    def __init__(self, spi_config: dict):
        self.spi_details = spi_config
        self.device_address = self.spi_details['address']
        self.device_register = self.spi_details['register']
        self.frequency = self.spi_details['frequency']
        self.data_size = self.spi_details['data_size']
        self.bus = self.spi_details['bus']
        self.device = self.spi_details['device']
        self.spi = None
        self.__initialize()

    def __initialize(self) -> None:
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.bus, self.device)  
            self.spi.max_speed_hz = self.frequency
        except Exception as e:
            raise CommInitError(f"Error initializing SPI device: {e}")

    def sendData(self, data: list) -> None:
        try:
            self.spi.writebytes(data)
        except Exception as e:
            raise CommWriteError(f"Error writing to SPI device: {e}")

    def receiveData(self) -> list:
        try:
            result = self.spi.readbytes(self.data_size)
            return result
        except Exception as e:
            raise CommReadError(f"Error reading from SPI device: {e}")
        
    def transfer(self, data: list):
        try:
            result = self.spi.xfer2(data.copy())
            return result
        except Exception as e:
            raise CommReadError(f"Error transferring data via SPI: {e}")

    def close(self) -> None:
        try:
            self.spi.close()
        except Exception as e:
            raise CommCloseError(f"Error closing SPI device: {e}")