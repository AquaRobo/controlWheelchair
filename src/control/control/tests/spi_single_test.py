from utils.Dispatcher import Dispatcher
from random import random
import struct
import time 

class SingleSPITest:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("ACTUATOR")

    def testSend(self):
        data = ["w", 100.0, -100.0]
        data_bytes = struct.pack('<cff', data[0].encode(), data[1], data[2])
        print(f"Data: {data} -> Bytes: {data_bytes}")
        while True:
            self.commHandler.sendData(data_bytes)

    def testRandomizeSendingData(self):
        left_speed = (random() - 0.5) * 200  
        right_speed = (random() - 0.5) * 200  
        
        data = ["w", left_speed, right_speed]
        
        while True:
            # Update random values each iteration
            data[1] = (random() - 0.5) * 200
            data[2] = (random() - 0.5) * 200
            data_bytes = struct.pack('<cff', data[0].encode(), data[1], data[2])
            print(f"Data: {data} -> Bytes: {data_bytes}")
            self.commHandler.sendData(data_bytes)
            time.sleep(3)
    
    def testRecieveData(self):
        while True:
            fmt = '<fff'
            data = self.commHandler.receiveData()
            unpacked_data = struct.unpack(fmt, bytes(data))
            print(f"Recieved Bytes: {unpacked_data}")
            time.sleep(3)
            
    def testTransfer(self):
        data = ["w", 100.0, -100.0]
        data_bytes = struct.pack('<cff', data[0].encode(), data[1], data[2])
        while True:
            recieved_data = self.commHandler.transfer(data_bytes)
            print(f"Data: {data} -> Bytes: {data_bytes}")
            print(f"Data: {recieved_data}")
            time.sleep(3)
    
    def testRandomizeTransferingData(self):
        left_speed = (random() - 0.5) * 200  
        right_speed = (random() - 0.5) * 200  
        fmt = '<cff'
        data = ["w", left_speed, right_speed]
        while True:
            # Update random values each iteration
            data[1] = (random() - 0.5) * 200
            data[2] = (random() - 0.5) * 200
            data_bytes = struct.pack(fmt, data[0].encode(), data[1], data[2])
            print(f"Data: {data} -> Bytes: {data_bytes}")
            recieved_data = self.commHandler.transfer(data_bytes)
            unpacked_data = struct.unpack(fmt, bytes(recieved_data))
            print(f"Recieved Bytes: {unpacked_data}")

            time.sleep(3)

if __name__ == "__main__":

    try:
        test = SingleSPITest()
        # test.testSend()
        # test.testRandomizeSendingData()
        test.testRecieveData()
        # test.testTransfer()
        # test.testRandomizeTransferingData()
    except KeyboardInterrupt:
        print("Comm Closed")
        test.commHandler.close()
    except Exception as e:
        print(f"Error occured: {e}")


    