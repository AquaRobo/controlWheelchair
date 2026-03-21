import serial
from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler
from control.helpers.DataStructer import DataStructer
from control.exceptions.CommInitException import CommInitError
from control.exceptions.CommCloseException import CommCloseError
from control.exceptions.CommReadException import CommReadError
from control.exceptions.CommWriteException import CommWriteError

@implementer(ICommhandler)
class UARTHandler:
    """Wrapper around UARTdev python library."""
    def __init__(self, UART_config: dict):
        self.UART_details = UART_config
        self.port = UART_config['port']
        self.baudrate = UART_config['baudrate']
        self.timeout = UART_config['timeout']
        self.UART = None
        self.__initialize()

    def __initialize(self) -> None:
        try:
            self.UART = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
        except Exception as e:
            raise CommInitError(f"Error initializing UART device: {e}")

    def sendData(self, data: list) -> None:
        try:
            packet = DataStructer.to_bytes(data)
            self.UART.write(packet)
            self.UART.flush()
        except Exception as e:
            raise CommWriteError(f"Error writing to UART device: {e}")

    def receiveData(self) -> list:
        try:
            if self.UART.in_waiting == 0:
                return []
            data = self.UART.read(self.UART.in_waiting)
            return list(data)
        except Exception as e:
            raise CommReadError(f"Error reading from UART device: {e}")
        
    def close(self) -> None:
        try:
            if self.UART and self.UART.is_open:
                self.UART.close()
        except Exception as e:
            raise CommCloseError(f"Error closing UART device: {e}")