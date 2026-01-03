#!/usr/bin/env python3
from utils.Configurator import Configurator

class ConfiguratorTest:
    def __init__(self, pkg_name="control"):
        self.configurator = Configurator(pkg_name=pkg_name)

    def smoothing_test(self):
        self.strat = self.configurator.fetchData(Configurator.SMOOTHING_STRAT)
        print(self.strat)

    def wheelchair_controllers_test(self):
        self.controllers = self.configurator.fetchData(Configurator.WHEELCHAIR_CONTROLLERS)
        print(self.controllers)

    def motors_test(self):
        self.motors = self.configurator.fetchData(Configurator.MOTORS)
        print(self.motors)

if __name__ == "__main__":
    test = ConfiguratorTest(pkg_name="control")
    test.smoothing_test()
    # test.motors_test()
    # test.wheelchair_controllers_test()
