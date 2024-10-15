#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class MotorController {
public:
    MotorController(uint8_t addr = 0x40); // Default I2C address for PCA9685 is 0x40
    void begin();
    
    // Function to set motor speed and direction
    void setMotorSpeed(uint8_t motor, int speed); // speed (-255 to 255)
    void stopMotor(uint8_t motor); // Stop a specific motor

private:
    Adafruit_PWMServoDriver pwm;
    
    // Private helper functions to set IN1 and IN2
    void setMotorPwm(uint8_t motor, uint16_t pwmValue);
    void setMotorDirection(uint8_t motor, bool forward);

    // Motor pins (NET1-NET8 in your schematics)
    const uint8_t motorPins[4][2] = {
        {0, 1}, // Motor 1 IN1 and IN2
        {2, 3}, // Motor 2 IN1 and IN2
        {4, 5}, // Motor 3 IN1 and IN2
        {6, 7}  // Motor 4 IN1 and IN2
    };
};

#endif
