from zope.interface import Interface, Attribute

class ISmoother(Interface):

    def smooth(current_value:float, target_value:float) -> float:
        """Smooth the input value.

        Args:
            current_value (float): The current value to be smoothed.
            target_value (float): The target value to be reached.

        Returns:
            float: The smoothed output value.
        """
        pass