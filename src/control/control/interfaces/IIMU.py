from zope.interface import Interface

class IIMU(Interface):
    def getOrientation(self) -> list[float]:
        """Get the current orientation data in quaternion format [x, y, z, w]."""

    def getAngularVelocity(self) -> list[float]:
        """Get the current angular velocity data in radians per second [x, y, z]."""

    def getLinearAcceleration(self) -> list[float]:
        """Get the current linear acceleration data in meters per second squared [x, y, z]."""
    
    def getEulerAngles(self) -> list[float]:
        """Get the current orientation data in Euler angles format [roll, pitch, yaw] in degrees."""