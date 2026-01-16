#!/usr/bin/env python3
from utils.Dispatcher import Dispatcher

class DispatcherTest:
    @staticmethod
    def test():
        print("Testing Dispatcher...")
        print(Dispatcher.get_smoothing_strategy())

if __name__ == "__main__":
    DispatcherTest.test()