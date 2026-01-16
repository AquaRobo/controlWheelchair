#!/usr/bin/env python3
from zope.interface import implementer
from control.interfaces.ISmoother import ISmoother

@implementer(ISmoother)
class LinearSmoothing:

    def __init__(self, smoothing_factor: float = 0.1):
        self.smoothing_factor = smoothing_factor

    def smooth(self, current_value: float, target_value: float) -> float:
        """Smooth the input value using a linear approach.
        Args:
            current_value (float): The current value to be smoothed.
            target_value (float): The target value to be reached.

        Returns:
            float: The smoothed output value.
        """
        if abs(target_value - current_value) > self.smoothing_factor:
            if target_value > current_value:
                current_value += self.smoothing_factor
            else:
                current_value -= self.smoothing_factor
        else:
            current_value = target_value
        return current_value