from control.services.Navigation import Navigation
from control.services.MotorDriver import MotorDriver
import time

class NavigationTest:
    def __init__(self):
        self.motor_driver = MotorDriver()
        self.navigation = Navigation(self.motor_driver)

    def testForwardMovement(self):
        x_axis = 0
        y_axis = 0
        while y_axis <= 1:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            y_axis += 0.1
        y_axis = 1
        while y_axis >= 0:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            y_axis -= 0.1

    def testBackwardMovement(self):
        x_axis = 0
        y_axis = 0
        while y_axis >= -1:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            y_axis -= 0.1
        y_axis = -1
        while y_axis <= 0:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            y_axis += 0.1

    def testRightMovement(self):
        x_axis = 0
        y_axis = 0
        while x_axis <= 1:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            x_axis += 0.1
        x_axis = 1
        while x_axis >= 0:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            x_axis -= 0.1

    def testLeftMovement(self):
        x_axis = 0
        y_axis = 0
        while x_axis >= -1:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            x_axis -= 0.1
        x_axis = -1
        while x_axis <= 0:
            self.navigation.navigate(x_axis, y_axis)
            time.sleep(1)
            x_axis += 0.1

    def run(self):
        print("Testing Forward Movement")
        self.testForwardMovement()
        print("Testing Backward Movement")
        self.testBackwardMovement()
        print("Testing Right Movement")
        self.testRightMovement()
        print("Testing Left Movement")
        self.testLeftMovement()
        print("All tests completed.")
        

if __name__ == "__main__":
    try:
        test = NavigationTest()
        test.run()
    except Exception as e:
        print(f"Error occurred: {e}")
    except KeyboardInterrupt:
        print("Test interrupted by user.")