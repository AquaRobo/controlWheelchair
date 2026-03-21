from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler
from control.exceptions.CommInitException import CommInitError
from control.exceptions.CommCloseException import CommCloseError
from control.exceptions.CommReadException import CommReadError
from control.exceptions.CommWriteException import CommWriteError

@implementer(ICommhandler)
class CommMock:
    def __init__(self, comm_config: dict):
        self.spi_details = comm_config
        self.__initialize()

    def __initialize(self) -> None:
        try:
            print("Comm is initialized.")
        except Exception as e:
            raise CommInitError(f"Error initializing comm: {e}")

    def sendData(self, data: list) -> None:
        try:
            print(f"Data: {data} has been sent.")
        except Exception as e:
            raise CommWriteError(f"Error writing: {e}")

    def receiveData(self) -> list:
        try:
            result = [0,0,0]
            return result
        except Exception as e:
            raise CommReadError(f"Error reading: {e}")
        
    def transfer(self, data: list):
        try:
            result = [0,0,0]
            return result
        except Exception as e:
            raise CommReadError(f"Error transferring: {e}")

    def close(self) -> None:
        try:
            print("Comm is closed.")
        except Exception as e:
            raise CommCloseError(f"Error closing comm: {e}")