#!/usr/bin/env python3
from utils.Dispatcher import Dispatcher

class SmoothingTest:
    def __init__(self):
        self.smoother = Dispatcher.get_smoothing_strategy()

    def run(self):
        current_value = 0.0
        target_value = 1.0
        print(f"Initial current value: {current_value}, target value: {target_value}")
        for _ in range(15):
            current_value = self.smoother.smooth(current_value, target_value)
            print(f"Smoothed value: {current_value}")


if __name__ == "__main__":
    test = SmoothingTest()
    test.run()
