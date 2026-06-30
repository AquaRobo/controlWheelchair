#include "/home/mahmoud/ESP/headers/SPIHandler.h"
#include "/home/mahmoud/ESP/services/SPIHandler.cpp"
#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <stdlib.h>

// -------------------------------------------------------
// SPI
// -------------------------------------------------------
SPIHandler spi_handler(40, 4);

// -------------------------------------------------------
// Pin Definitions
// -------------------------------------------------------
#define STEP_SHOULDER_R   14    // stepper1 - Shoulder Right
#define DIR_SHOULDER_R    27

#define STEP_SHOULDER_L   12    // stepper2 - Shoulder Left
#define DIR_SHOULDER_L    15

#define STEP_GEARED       33    // stepper3 - Geared shoulder (5.18:1)
#define DIR_GEARED         4

#define STEP_FOREARM      32    // stepper4 - Forearm
#define DIR_FOREARM       16

#define STEP_WRIST        26    // stepper5 - Wrist
#define DIR_WRIST         25

#define STEP_ROTATION     36    // stepper6 - Base Rotation
#define DIR_ROTATION       2

#define SERVO_PIN          13    // Gripper servo GPIO

// ------------------------------------------------------------
// Motor Speed & Acceleration
// ------------------------------------------------------------
// Microstepping = 16  →  3200 steps/rev (motor shaft)
// Target ≈ 150 RPM  →  150 × 3200 / 60 = 8000 steps/s
// Ramp to max in ~0.5s → 16000 steps/s²
// ------------------------------------------------------------
#define MICROSTEPS          16
#define STEPS_PER_REV       (200 * MICROSTEPS)   // 3200
#define SPEED_STANDARD   4000
#define ACCEL_STANDARD   6000

#define SPEED_WRIST          300
#define SPEED_ROTATION      6000

#define ACCEL_WRIST          600
#define ACCEL_ROTATION     12000

// -------------------------------------------------------
// Stepper Motors
// -------------------------------------------------------
AccelStepper motorShoulderR (AccelStepper::DRIVER, STEP_SHOULDER_R, DIR_SHOULDER_R);
AccelStepper motorShoulderL (AccelStepper::DRIVER, STEP_SHOULDER_L, DIR_SHOULDER_L);
AccelStepper motorGeared    (AccelStepper::DRIVER, STEP_GEARED,     DIR_GEARED);
AccelStepper motorForearm   (AccelStepper::DRIVER, STEP_FOREARM,    DIR_FOREARM);
AccelStepper motorWrist     (AccelStepper::DRIVER, STEP_WRIST,      DIR_WRIST);
AccelStepper motorRotation  (AccelStepper::DRIVER, STEP_ROTATION,   DIR_ROTATION);

// -------------------------------------------------------
// Gripper Servo
// -------------------------------------------------------
Servo gripperServo;
char lastGripperCmd = 0;

// -------------------------------------------------------
// Globals
// -------------------------------------------------------
char cmd;
float v1, v2;

// -------------------------------------------------------
// Sensor data (IMU placeholders)
// -------------------------------------------------------
float orientation_x, orientation_y, orientation_z, orientation_w;
float linear_acc_x, linear_acc_y, linear_acc_z;
float angular_vel_x, angular_vel_y, angular_vel_z;

// -------------------------------------------------------
// sensorTask — fills TX buffer with IMU data for Pi
// Runs on core 1
// -------------------------------------------------------
void sensorTask(void* arg) {
    while (true) {
        orientation_x = 0.707f;
        orientation_y = 0.0f;
        orientation_z = 0.707f;
        orientation_w = 0.0f;
        linear_acc_x  = 0.0f;
        linear_acc_y  = 0.0f;
        linear_acc_z  = 9.81f;
        angular_vel_x = 1.0f;
        angular_vel_y = 0.0f;
        angular_vel_z = 0.0f;

        float imu_values[10] = {
            orientation_x, orientation_y, orientation_z, orientation_w,
            linear_acc_x,  linear_acc_y,  linear_acc_z,
            angular_vel_x, angular_vel_y, angular_vel_z
        };
        uint8_t imu_payload[sizeof(imu_values)];
        memcpy(imu_payload, imu_values, sizeof(imu_values));
        spi_handler.fillTXBuffer(imu_payload, sizeof(imu_payload));

        vTaskDelay(20);  // ~50 Hz
    }
}

// -------------------------------------------------------
// actuatorTask — receives SPI packets, updates motor
// targets via moveTo(). NEVER blocks on runToPosition().
// stepperTask handles the actual stepping.
// Runs on core 0.
// -------------------------------------------------------
void actuatorTask(void* arg) {
    int32_t prevSteps[6] = {0, 0, 0, 0, 0, 0};

    while (true) {
        spi_handler.transferBuffers();
        auto actuators = spi_handler.getRXData();

        if (actuators.size() >= 4) {
            cmd = actuators[0];

            // 'w' command — wheel speeds (2 floats)
            if (cmd == 'w' && actuators.size() >= 9) {
                memcpy(&v1, &actuators[4], 4);
                memcpy(&v2, &actuators[8], 4);
                Serial.printf("Received: %c, %.2f, %.2f\n", cmd, v1, v2);
            }

            // 's' command — absolute step targets for 6 joints
            else if (cmd == 's' && actuators.size() >= 28) {
                int32_t newSteps[6];
                memcpy(&newSteps[0], &actuators[4],  4);
                memcpy(&newSteps[1], &actuators[8],  4);
                memcpy(&newSteps[2], &actuators[12], 4);
                memcpy(&newSteps[3], &actuators[16], 4);
                memcpy(&newSteps[4], &actuators[20], 4);
                memcpy(&newSteps[5], &actuators[24], 4);

                // Joint 1 → Base rotation
                if (newSteps[0] != prevSteps[0]) {
                    motorRotation.moveTo(newSteps[0]);
                    prevSteps[0] = newSteps[0];
                }

                // Joint 2 → Shoulder (both motors, mirrored)
                if (newSteps[1] != prevSteps[1]) {
                    motorShoulderR.moveTo( 2*newSteps[1]);
                    motorShoulderL.moveTo(-2*newSteps[1]);
                    prevSteps[1] = newSteps[1];
                }

                // Joint 3 → Geared shoulder
                if (newSteps[2] != prevSteps[2]) {
                    motorGeared.moveTo(-15*newSteps[2]);
                    prevSteps[2] = newSteps[2];
                }

                // Joint 4 → Forearm
                if (newSteps[3] != prevSteps[3]) {
                    motorForearm.moveTo(-newSteps[3]);
                    prevSteps[3] = newSteps[3];
                }

                // Joint 5 → Wrist
                if (newSteps[4] != prevSteps[4]) {
                    motorWrist.moveTo(newSteps[4]);
                    prevSteps[4] = newSteps[4];
                }

                // Throttled debug output
                static unsigned long lastPrint = 0;
                if (millis() - lastPrint > 500) {
                    lastPrint = millis();
                    Serial.printf("Targets → Rot:%d  Sh:%d  Gear:%d  Fore:%d  Wrist:%d\n",
                                  newSteps[0], newSteps[1], newSteps[2],
                                  newSteps[3], newSteps[4]);
                }
            }

            // 'g' command — gripper servo (1 byte: 0=open, 1=close)
            else if (cmd == 'g' && actuators.size() >= 5) {
                uint8_t grip = actuators[4];
                if (grip == 0 && lastGripperCmd != 0) {
                    gripperServo.write(20);
                    lastGripperCmd = 0;
                    Serial.println("GRIPPER → Open");
                } else if (grip == 1 && lastGripperCmd != 1) {
                    gripperServo.write(90);
                    lastGripperCmd = 1;
                    Serial.println("GRIPPER → Close");
                }
            }
        }

        vTaskDelay(1);
    }
}

// -------------------------------------------------------
// stepperTask — continuously calls run() on all motors.
// This is the ONLY place motors actually step.
// Runs on core 1.
// -------------------------------------------------------
void stepperTask(void* arg) {
    while (true) {
        motorRotation.run();
        motorShoulderR.run();
        motorShoulderL.run();
        motorGeared.run();
        motorForearm.run();
        motorWrist.run();

        // Minimal yield — run() must be called as fast as possible
        // for smooth acceleration. taskYIELD is cheaper than vTaskDelay.
        taskYIELD();
    }
}

// -------------------------------------------------------
// Setup
// -------------------------------------------------------
void setup() {
    Serial.begin(115200);

    // Gripper servo
    gripperServo.attach(SERVO_PIN);
    gripperServo.write(20);  // Start open

    // Shoulder motors
    motorShoulderR.setMaxSpeed(SPEED_STANDARD);
    motorShoulderR.setAcceleration(ACCEL_STANDARD);

    motorShoulderL.setMaxSpeed(SPEED_STANDARD);
    motorShoulderL.setAcceleration(ACCEL_STANDARD);

    // Geared shoulder
    motorGeared.setMaxSpeed(7*SPEED_STANDARD);
    motorGeared.setAcceleration(7*ACCEL_STANDARD);

    // Forearm
    motorForearm.setMaxSpeed(SPEED_STANDARD);
    motorForearm.setAcceleration(ACCEL_STANDARD);

    // Wrist — finer, slower
    motorWrist.setMaxSpeed(SPEED_WRIST);
    motorWrist.setAcceleration(ACCEL_WRIST);

    // Base rotation
    motorRotation.setMaxSpeed(SPEED_ROTATION);
    motorRotation.setAcceleration(ACCEL_ROTATION);

    // Initialize SPI
    spi_handler.initialize();

    // Core 0: actuatorTask (SPI receive + target updates)
    // Core 1: stepperTask (continuous run()) + sensorTask (IMU TX, sleeps 20ms)
    xTaskCreatePinnedToCore(actuatorTask, "actuatorTask", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(stepperTask,  "stepperTask",  4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(sensorTask,   "sensorTask",   4096, NULL, 1, NULL, 1);

    Serial.println("ESP32 Arm Controller Ready.");
}

// -------------------------------------------------------
// Loop — empty, everything runs in FreeRTOS tasks
// -------------------------------------------------------
void loop() {}