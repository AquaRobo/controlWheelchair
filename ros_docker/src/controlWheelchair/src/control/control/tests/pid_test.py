from control.services.PIDController import PIDController
from utils.Configurator import Configurator


class PIDControllerTest:
    def __init__(self):
        self.pid_params = Configurator("control").fetchData(Configurator.PID_PARAMS)
        self.pid_controller = PIDController(
            kp=self.pid_params["yaw_KP"],
            ki=self.pid_params["yaw_KI"],
            kd=self.pid_params["yaw_KD"],
            setpoint=10.0
        )

    def test_update_setpoint(self):
        new_setpoint = 20.0
        self.pid_controller.updateSetpoint(new_setpoint)
        assert self.pid_controller.setpoint == new_setpoint, "Setpoint update failed."

    def test_update_constants(self):
        new_kp = 0.02
        new_ki = 0.0
        new_kd = 0.001
        self.pid_controller.updateConstants(new_kp, new_ki, new_kd)
        assert (self.pid_controller.kp, self.pid_controller.ki, self.pid_controller.kd) == (new_kp, new_ki, new_kd), "Constants update failed."

    def test_stablize(self):
        measured_value = 30
        control_output = self.pid_controller.stablize(measured_value)
        print (f"Control Output: {control_output} for Measured Value: {measured_value} with Setpoint: {self.pid_controller.setpoint}")
        assert control_output is not None, "Stablize method failed to return control output."

if __name__ == "__main__":
    test_suite = PIDControllerTest()
    test_suite.test_update_setpoint()
    test_suite.test_update_constants()
    test_suite.test_stablize()
    print("All PIDController tests passed.")