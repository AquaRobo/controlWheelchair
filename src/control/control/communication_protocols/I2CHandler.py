from zope.interface import implementer
from control.interfaces.ICommhandler import ICommhandler
from control.helpers.DataStructer import DataStructer
from smbus2 import SMBus

@implementer(ICommhandler)
class I2CHandler:
    def __init__(self, i2c_config: dict):
        self.__bus = SMBus(bus=i2c_config["bus"])
        self.__ESP_address = i2c_config["address"]

    def sendData(self, data: list, register: int = 0x01) -> None:
        byte_data = DataStructer.to_byte_list(data)
        self.__bus.write_block_data(self.__ESP_address, register, byte_data)
       
    def receiveData(self) -> str:
        # Implementation for receiving data via I2C
        pass

    def stopAll(self) -> None:
        '''
        Sends the Stop register (0x00) to stop all motors
        '''
        register = 0x00 
        if self.__bus is not None:
            self.sendData(data=["0"], register=register)
        else:
            print("ESP communication hasn't been initialized")


    def close (self) -> None:
        self.stopAll()
        self.__bus.close()