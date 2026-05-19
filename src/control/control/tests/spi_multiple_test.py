from utils.Dispatcher import Dispatcher
from random import random

class MultipleSPITest:
    def __init__(self):
        self.actuators_commHandler = Dispatcher().get_communication_handler("ESP")
        self.sensors_commHandler = Dispatcher().get_communication_handler("STM")

    def testSend(self):
        data = ["w", 100.0, -100.0]
        while True:
            self.actuators_commHandler.sendData(data)
            self.sensors_commHandler.sendData(data)
            
    def testRandomizeSendingData(self):
        left_speed = (random() - 0.5) * 200  
        right_speed = (random() - 0.5) * 200  
        
        data = ["w", left_speed, right_speed]
        
        while True:
            # Update random values each iteration
            data[1] = (random() - 0.5) * 200
            data[2] = (random() - 0.5) * 200
            print(f"Data: {data}")
            self.actuators_commHandler.sendData(data)
            self.sensors_commHandler.sendData(data)
    
    def testRecieveData(self):
        while True:
            actuators_data = self.actuators_commHandler.receiveData()
            sensors_data = self.sensors_commHandler.receiveData()
            print(f"Actuators Data: {actuators_data} \nSensors Data: {sensors_data}")

    def testTransfer(self):
        data = ["w", 100.0, -100.0]
        while True:
            actuators_data = self.actuators_commHandler.transfer(data)
            sensors_data = self.sensors_commHandler.transfer(data)
            print(f"Actuators Data: {actuators_data} \nSensors Data: {sensors_data}")
    
    def testRandomizeTransferingData(self):
        left_speed = (random() - 0.5) * 200  
        right_speed = (random() - 0.5) * 200  
        
        data = ["w", left_speed, right_speed]
        
        while True:
            # Update random values each iteration
            data[1] = (random() - 0.5) * 200
            data[2] = (random() - 0.5) * 200
            print(f"Sending Data: {data}")
            actuators_data = self.actuators_commHandler.transfer(data)
            sensors_data = self.sensors_commHandler.transfer(data)
            print(f"Actuators Data: {actuators_data} \nSensors Data: {sensors_data}")

    def testSendRecieve(self):
        data = ["w", 100.0, -100.0]
        while True:
            self.actuators_commHandler.sendData(data)
            sensors_data = self.sensors_commHandler.receiveData()
            print(f"Sensors Data: {sensors_data}")

if __name__ == "__main__":
    try:
        test = MultipleSPITest()
        test.testSend()
        # test.testRandomizeSendingData()
        # test.testRecieveData()
        # test.testTransfer()
        # test.testRandomizeTransferingData()
        # test.testSendRecieve()
    except KeyboardInterrupt:
        print("Comm Closed")
        test.actuators_commHandler.close()
    except Exception as e:
        print(f"Error occured: {e}")


    