#include "MotorController.h"

MotorController::MotorController(uint8_t addr) : pwm(addr) {
}

void MotorController::begin() {
    pwm.begin();
    pwm.setPWMFreq(1600); // Set frequency to 1.6kHz
}

// Set motor speed: negative for reverse, positive for forward, 0 to stop
void MotorController::setMotorSpeed(uint8_t motor, int speed) {
    if (motor >= 4) return; // Invalid motor number
    
    if (speed == 0) {
        stopMotor(motor);
        return;
    }

    bool forward = (speed > 0);
    uint16_t pwmValue = abs(speed) * 16; // Scale 0-255 to PCA9685 resolution (0-4096)
    
    setMotorDirection(motor, forward);
    setMotorPwm(motor, pwmValue);
}

void MotorController::stopMotor(uint8_t motor) {
    setMotorPwm(motor, 0); // Set both IN1 and IN2 to low to stop the motor
}

// Helper function to set the PWM signal for a specific motor
void MotorController::setMotorPwm(uint8_t motor, uint16_t pwmValue) {
    pwm.setPWM(motorPins[motor][0], 0, pwmValue); // Set PWM for IN1
    pwm.setPWM(motorPins[motor][1], 0, pwmValue); // Set PWM for IN2
}

// Helper function to set motor direction
void MotorController::setMotorDirection(uint8_t motor, bool forward) {
    if (forward) {
        pwm.setPWM(motorPins[motor][0], 0, 4096); // IN1 high
        pwm.setPWM(motorPins[motor][1], 0, 0);    // IN2 low
    } else {
        pwm.setPWM(motorPins[motor][0], 0, 0);    // IN1 low
        pwm.setPWM(motorPins[motor][1], 0, 4096); // IN2 high
    }
}
