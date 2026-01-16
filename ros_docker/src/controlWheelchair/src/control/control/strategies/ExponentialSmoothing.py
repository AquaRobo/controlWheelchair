#!/usr/bin/env python3
from zope.interface import implementer
from control.interfaces.ISmoother import ISmoother

@implementer(ISmoother)
class ExponentialSmoothing:

    def __init__(self, alpha: float = 0.5):
        """Initialize the ExponentialSmoothing with a smoothing factor alpha.
        Args:
            alpha (float): Smoothing factor between 0 and 1. Default is 0.5.
        """
        if not (0 < alpha < 1):
            raise ValueError("Alpha must be between 0 and 1.")
        self.alpha = alpha

    def smooth(self, current_value: float, target_value: float, tolerance: float = 0.1) -> float:
        """Smooth the input value using an exponential smoothing approach.
        Args:
            current_value (float): The current value to be smoothed.
            target_value (float): The target value to be reached.
            tolerance (float): The tolerance within which to snap to the target value. Default is 0.1.

        Returns:
            float: The smoothed output value.
        """
        smoothed_value = (self.alpha * target_value) + ((1 - self.alpha) * current_value)
        if abs(smoothed_value - target_value) <= tolerance:
            return target_value
        smoothed_value = float(f"{smoothed_value:.2f}")
        return smoothed_value
