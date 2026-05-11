from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler

@implementer(ICommhandler)
class CANHandler:
    def __init__(self, can_config: dict):
        pass

    def sendData(self, data: str) -> None:
        # Implementation for sending data via CAN
        pass

    def receiveData(self) -> str:
        # Implementation for receiving data via CAN
        pass