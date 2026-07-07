import math
import board
import busio
import time

from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

class IMU:
    """Driver for a BNO08x IMU wired directly to the Raspberry Pi over I2C.

    Enables the accelerometer, gyroscope, magnetometer and rotation-vector
    reports on the sensor and exposes them through simple getters. Also
    provides interactive calibration helpers. Used by PiIMUNode.

    Input:  BNO08x sensor at the given I2C address (default 0x4b) via
            board SCL/SDA at 400 kHz.
    Output: get_rotation() quaternion (i, j, k, real), get_gyro() [rad/s],
            get_acceleration() [m/s²], get_magnetometer() [µT],
            get_euler_angles() (roll, pitch, yaw).
    """

    def __init__ (self, address = 0x4b):
        self.i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        self.bno = BNO08X_I2C(self.i2c, address=address)
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        print("IMU initialized successfully")
        
    def calibrate(self, duration=30):
        """calibration"""
        print(" Calibration mode: ACTIVE")
        self.bno.begin_calibration()
        
        print("\n Performing calibration motions...")
        
        for i in range(duration, 0, -1):
            print(f"Time remaining: {i} seconds", end='\r')
            time.sleep(1)
        
        try:
            self.bno.save_calibration_data()
            print("\nCALIBRATION COMPLETE AND SAVED!")
            return True
        except Exception as e:
            print(f"\n Failed to save calibration: {e}")
            return False
        
    def check_calibration_status(self):
        """calibration status check"""
        mag_readings = []
        for i in range(10):
            mag_x, mag_y, mag_z = self.bno.magnetic
            mag_readings.append((mag_x, mag_y, mag_z))
            time.sleep(0.1)
        
        magnitudes = [math.sqrt(x**2 + y**2 + z**2) for x, y, z in mag_readings]
        avg_mag = sum(magnitudes) / len(magnitudes)
        mag_variance = max(magnitudes) - min(magnitudes)
        print(f"Magnetometer - Avg Magnitude: {avg_mag:.1f} μT, Variance: {mag_variance:.1f} μT")

        status = self.bno.calibration_status
        status_levels = ["Unreliable", "Low", "Medium", "High"]
        print(f"Calibration status {status_levels[status]}")
        
        if 20 <= avg_mag <= 70:
            print("✓ Magnetometer magnitude looks good")
        else:
            print("⚠ Magnetometer may need calibration")
        
    def get_acceleration(self) -> tuple:
        return self.bno.acceleration

    def get_gyro(self) -> tuple:
        return self.bno.gyro
    
    def get_magnetometer(self) -> tuple:
        return self.bno.magnetic

    def get_rotation(self) -> tuple:
        return self.bno.quaternion
    
    def get_euler_angles(self, degrees=True) -> tuple:
        """Get orientation as Euler angles"""
        quat_i, quat_j, quat_k, quat_real = self.get_rotation()
        roll, pitch, yaw = self.quaternion_to_euler(quat_i, quat_j, quat_k, quat_real)
        
        if degrees:
            roll = math.degrees(roll)
            pitch = math.degrees(pitch)
            yaw = math.degrees(yaw)

        angles = (roll,pitch,yaw)
        return angles
    
    def quaternion_to_euler(self, q_i, q_j, q_k, q_real):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q_real * q_i + q_j * q_k)
        cosr_cosp = 1 - 2 * (q_i * q_i + q_j * q_j)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q_real * q_j - q_k * q_i)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q_real * q_k + q_i * q_j)
        cosy_cosp = 1 - 2 * (q_j * q_j + q_k * q_k)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw


        