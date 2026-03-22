import math
import struct
from zope.interface import implementer
from utils.Dispatcher import Dispatcher
from control.interfaces.IIMU import IIMU

@implementer(IIMU)
class IMU:
    def __init__(self):
        self.__commHandler = Dispatcher().get_communication_handler("ESP")
        self._fmt = '<' + 'f' * 10 # 4 floats for orientation (quaternion), 3 for angular velocity, and 3 for linear acceleration
        self.imu_data = None

    def update(self) -> None:
        """Fetch the latest IMU data from the communication handler and store it locally."""
        recieved_data = self.__commHandler.receiveData()
        self.imu_data = struct.unpack(self._fmt, bytes(recieved_data))

    def getOrientation(self) -> list[float]:
        """Get the current orientation data in quaternion format [x, y, z, w]."""
        if self.imu_data is None:
            return [0.0, 0.0, 0.0, 1.0]  
        return list(self.imu_data[0:4])
    
    def getLinearAcceleration(self) -> list[float]:
        """Get the current linear acceleration data in meters per second squared [x, y, z]."""
        if self.imu_data is None:
            return [0.0, 0.0, 0.0]
        return list(self.imu_data[4:7])
    
    def getAngularVelocity(self) -> list[float]:
        """Get the current angular velocity data in radians per second [x, y, z]."""
        if self.imu_data is None:
            return [0.0, 0.0, 0.0]
        return list(self.imu_data[7:10])
    
    def getEulerAngles(self) -> list[float]:
        """Get the current orientation data in Euler angles format [roll, pitch, yaw] in degrees."""
        orientation = self.getOrientation()
        x, y, z, w = orientation
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(2.0 * (w * y - z * x))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]
