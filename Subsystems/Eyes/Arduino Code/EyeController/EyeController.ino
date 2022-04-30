// Copyright (c) 2022 Mario "pinsrq" Sieg <mt3000@gmx.de>

//  X-axis joystick pin: A1
//  Y-axis joystick pin: A0
//  Trim potentiometer pin: A2
//  Button pin: 2

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

static constexpr int32_t ANALOG_INPUT_PIN { A0 };
static constexpr uint16_t SERVO_MIN { 140 };
static constexpr uint16_t SERVO_MAX { 520 };

static struct IVec2 final {
    int32_t x, y;
} accumulator, leftPulse, rightPulse, lidPulse;

static int32_t trimVal, sensorVal, outputVal, switchVal;

static Adafruit_PWMServoDriver servoDriver { };

static inline auto initSerialSystem() -> void {
    Serial.begin(9600);
    Serial.println("8 channel Servo test!");
}

static inline auto initServoSystem(const uint8_t freq = 60) -> void {
    pinMode(ANALOG_INPUT_PIN, INPUT);
    pinMode(2, INPUT);
    servoDriver.begin();
    servoDriver.setPWMFreq(freq);  // Analog servos run at +-~60 Hz updates
}

// Used to set the pulse length in seconds
// e.g. setServoPulseFrequency(0, 0.001) is a ~1 millisecond pulse width. Precision is limited!
static inline auto setServoPulseFrequency(const uint8_t n, double pulse, double* const outPulse = nullptr) -> void {
    static constexpr double pulseLen { (1000000.0 / 60.0) / static_cast<double>(1 << 12) }; // 1,000,000 us per second / 60 Hz / 12 bits of resolution (1<<12)=(4096)
    pulse *= 1000000.0;
    pulse /= pulseLen;
    servoDriver.setPWM(n, 0, pulse);
    if (outPulse) *outPulse = pulse;
    Serial.println(pulse);
}

static inline auto updateXAxis() -> void {
    accumulator.x = analogRead(A1);
    leftPulse.x = map(accumulator.x, 0, 1023, 270, 390);
    rightPulse.x = leftPulse.x;
}

static inline auto updateYAxis() -> void {
    accumulator.y = analogRead(A0);
    leftPulse.y = map(accumulator.y, 0,1023, 280, 400);
    rightPulse.y = map(accumulator.y, 0,1023, 400, 280);
}

static inline auto trim() -> void {
    trimVal = analogRead(A2);
    trimVal = map(trimVal, 320, 580, -40, 40);
    lidPulse.x = map(accumulator.y, 0, 1023, 280, 420);
    lidPulse.x += (trimVal - 40);
    lidPulse.x = constrain(lidPulse.x, 280, 400);
    lidPulse.y = map(accumulator.y, 0, 1023, 410, 280);
    lidPulse.y += trimVal >> 1;
    lidPulse.y = constrain(lidPulse.y, 280, 400);  
}

static inline auto updateDriverPWM() -> void {
    auto& pwm { servoDriver };

    pwm.setPWM(0, 0, leftPulse.x);
    pwm.setPWM(1, 0, leftPulse.y);
    pwm.setPWM(2, 0, rightPulse.x);
    pwm.setPWM(3, 0, rightPulse.y);

    switch (switchVal) {
        case HIGH:
            pwm.setPWM(4, 0, 240);
            pwm.setPWM(5, 0, 240);
        break;
        case LOW:
            pwm.setPWM(4, 0, lidPulse.x);
            pwm.setPWM(5, 0, lidPulse.y);
        break;
        default: ;
    }
}

auto setup() -> void {
    initSerialSystem();
    initServoSystem();
    delay(10);
}

auto loop() -> void {
    updateXAxis();
    switchVal = digitalRead(2);
    updateYAxis();

    trim();
    updateDriverPWM();

    Serial.println(leftPulse.x);

    delay(5);
}
