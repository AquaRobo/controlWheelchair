from utils.Dispatcher import Dispatcher
from random import random

class SingleSPITest:
    def __init__(self):
        self.commHandler = Dispatcher().get_communication_handler("ACTUATOR")

    def testSend(self):
        data = ["w", 100.0, -100.0]
        while True:
            self.commHandler.sendData(data)

    def testRandomizeSendingData(self):
        left_speed = (random() - 0.5) * 200  
        right_speed = (random() - 0.5) * 200  
        
        data = ["w", left_speed, right_speed]
        
        while True:
            # Update random values each iteration
            data[1] = (random() - 0.5) * 200
            data[2] = (random() - 0.5) * 200
            print(f"Data: {data}")
            self.commHandler.sendData(data)
    
    def testRecieveData(self):
        while True:
            data = self.commHandler.receiveData()
            print(f"Data: {data}")

    def testTransfer(self):
        data = ["w", 100.0, -100.0]
        while True:
            recieved_data = self.commHandler.transfer(data)
            print(f"Data: {recieved_data}")
    
    def testRandomizeTransferingData(self):
        left_speed = (random() - 0.5) * 200  
        right_speed = (random() - 0.5) * 200  
        
        data = ["w", left_speed, right_speed]
        
        while True:
            # Update random values each iteration
            data[1] = (random() - 0.5) * 200
            data[2] = (random() - 0.5) * 200
            print(f"Sending Data: {data}")
            recieved_data = self.commHandler.transfer(data)
            print(f"Recieved Data: {recieved_data}")

if __name__ == "__main__":
    try:
        test = SingleSPITest()
        test.testSend()
        # test.testRandomizeSendingData()
        # test.testRecieveData()
        # test.testTransfer()
        # test.testRandomizeTransferingData()
    except KeyboardInterrupt:
        print("Comm Closed")
        test.commHandler.close()
    except Exception as e:
        print(f"Error occured: {e}")


    