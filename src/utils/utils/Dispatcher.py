#!/usr/bin/env python3
from utils.Configurator import Configurator
from utils.EnvParams import EnvParams

class Dispatcher:

    @staticmethod
    def get_smoothing_strategy() -> object:
        """Dynamically fetch and instantiate the smoothing strategy based on configuration.
        Returns:
            object: An instance of the selected smoothing strategy."""
        smoothing_dict = Configurator("control").fetchData(Configurator.SMOOTHING_STRAT)
        smoothing_strat = smoothing_dict["smoothing_strategy"]
        if smoothing_strat == "LinearSmoothing":
            from control.strategies.LinearSmoothing import LinearSmoothing
            factor = smoothing_dict["linear_factor"]
            return LinearSmoothing(factor)
        elif smoothing_strat == "ExponentialSmoothing":
            from control.strategies.ExponentialSmoothing import ExponentialSmoothing
            factor = smoothing_dict["exponential_factor"]
            return ExponentialSmoothing(factor)
        else:
            raise ValueError(f"Unknown smoothing strategy: {smoothing_strat}")
        
    @staticmethod
    def get_steering_strategy() -> object:
        steering_dict = Configurator("control").fetchData(Configurator.STEERING_STRAT)["steering_strategy"]
        if steering_dict == "DifferentialDrive":
            from control.strategies.DifferentialDriveSteering import DifferentialDriveSteering
            return DifferentialDriveSteering()
        else:
            raise ValueError(f"Unknown steering strategy: {steering_dict}")

    @staticmethod
    def get_communication_handler(module_type: str) -> object:
        """Dynamically fetch and instantiate the communication handler based on configuration.
        Args:
            module_type (str): The type of module requesting the communication handler.
        Returns:
            object: An instance of the selected communication handler."""
        comm_dict = Configurator("control").fetchData(Configurator.COMM_HANDLER)
        esp_protocol = Configurator("control").fetchData(Configurator.COMM_HANDLER)['esp_protocol']
        stm_protocol = Configurator("control").fetchData(Configurator.COMM_HANDLER)['stm_protocol']
        if EnvParams().ENV == "SIMULATION":
            from control.simulation_services.CommMock import CommMock
            return CommMock(comm_dict)
        else:
            if module_type == "ESP":
                if esp_protocol == "I2C":
                    comm_handler_config = comm_dict["i2c_esp_config"]
                    from control.communication_protocols.I2CHandler import I2CHandler
                    return I2CHandler(comm_handler_config)
                elif esp_protocol == "SPI":
                    comm_handler_config = comm_dict["spi_esp_config"]
                    from control.communication_protocols.SPIHandler import SPIHandler
                    return SPIHandler(comm_handler_config)
                elif esp_protocol == "UART":
                    comm_handler_config = comm_dict["uart_esp_config"]
                    from control.communication_protocols.UARTHandler import UARTHandler
                    return UARTHandler(comm_handler_config)
                else:
                    raise ValueError(f"Unknown esp protocol: {esp_protocol}")
            elif module_type == "STM":
                if stm_protocol == "I2C":
                    comm_handler_config = comm_dict["i2c_stm_config"]
                    from control.communication_protocols.I2CHandler import I2CHandler
                    return I2CHandler(comm_handler_config)
                elif stm_protocol == "SPI":
                    comm_handler_config = comm_dict["spi_stm_config"]
                    from control.communication_protocols.SPIHandler import SPIHandler
                    return SPIHandler(comm_handler_config)
                elif stm_protocol == "UART":
                    comm_handler_config = comm_dict["uart_stm_config"]
                    from control.communication_protocols.UARTHandler import UARTHandler
                    return UARTHandler(comm_handler_config)
                else:
                    raise ValueError(f"Unknown stm protocol: {stm_protocol}")
            else:
                raise ValueError(f"Unknown module type: {module_type}")
        
    
        