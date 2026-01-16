from zope.interface import Interface
from control.DTOs.motors import Motors

class IMotorDriver(Interface):
    def drive(self, motors_dict: dict[str, Motors]) -> None:
        """Drive the motors based on the provided motors dictionary."""

    def __buildMotorsArray(self, motors_dict: dict[str, Motors]) -> list[Motors]:
        """Build an array of Motors from the provided motors dictionary to pass it to the commHandler."""