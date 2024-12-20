#include <Wire.h>
#include "Adafruit_TCS34725.h"

#define PWMA 4  // PWM control pin for Motor A
#define AIN1 6  // Direction control pin 1 for Motor A
#define AIN2 5  // Direction control pin 2 for Motor A
#define PWMB 10 // PWM control pin for Motor B
#define BIN1 8  // Direction control pin 1 for Motor B
#define BIN2 9  // Direction control pin 2 for Motor B
#define STBY 7  // Standby pin for the motor driver
#define SWITCH_PIN 13 // Define the switch pin
#define ENCODER_E1A_PIN 2
#define ENCODER_E1B_PIN 3
#define ENCODER_E2A_PIN 18
#define ENCODER_E2B_PIN 19

volatile int encoderCount1 = 0; // Encoder count for Motor 1
volatile int encoderCount2 = 0; // Encoder count for Motor 2

int pos = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int targetSpeed = 50;
int LUX_THRESHOLD = 30; //Black color on paper
int extraEncoderCount = 0;
int estimatedTargetEncoderCount = 10000;

// Enum to represent the motor states
enum MotorState {
  STOPPED,
  FORWARD,
  REVERSE
};

MotorState motorState = STOPPED; // Initial state of the motors
unsigned long switchPressedTime = 0; // Time when the switch was pressed
bool switchWasPressed = false; // Flag to keep track of switch press
unsigned long stopTime = 0; // Time when the motors stopped

/* Initialise with specific int time and gain values */
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_1X);

void setup() {
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(ENCODER_E1A_PIN, INPUT);
  pinMode(ENCODER_E1B_PIN, INPUT);
  pinMode(ENCODER_E2A_PIN, INPUT);
  pinMode(ENCODER_E2B_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_E1A_PIN), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_E2A_PIN), readEncoder2, RISING);

  digitalWrite(AIN1, 1);
  digitalWrite(AIN2, 0);
  digitalWrite(BIN1, 1);
  digitalWrite(BIN2, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(SWITCH_PIN, INPUT_PULLUP); // Set the switch pin as input with internal pull-up

}

void SetPWM(int motor, int pwm) {
  if (motor == 1 && pwm >= 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, pwm);
  } else if (motor == 1 && pwm < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, -pwm);
  } else if (motor == 2 && pwm >= 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, pwm);
  } else if (motor == 2 && pwm < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, -pwm);
  }
}

void loop() {
  static bool lastSwitchState = HIGH; // Store the last switch state
  bool currentSwitchState = digitalRead(SWITCH_PIN);

  uint16_t r, g, b, c, colorTemp, lux;

  tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  // colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
  lux = tcs.calculateLux(r, g, b);

  // PID constants
  float kp = 1.095;//balance=1.095;
  float kd = 0.0;//0.0;
  float ki = 0.0;//0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / ( 1.0e6 );
  prevT = currT;

  // error
  int error = pos - targetSpeed;
  // int error = encoderCount2 - encoderCount1;

  // derivative
  float dedt = (error - eprev) / (deltaT);

  // integral
  eintegral = eintegral + error * deltaT;

  // control signal
  float u = kp * error + kd * dedt + ki * eintegral;

  // Motor power
  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  // Motor direction
  int dir = 1;
  if (u < 0) {
    dir = -1;
  }

  eprev = error;

  // Check for switch state change from HIGH to LOW (button press)
  if (lastSwitchState == HIGH && currentSwitchState == LOW) {
    // Change motor state based on current state
    if (motorState == STOPPED) {
      motorState = FORWARD;
      SetPWM(1, targetSpeed); // Set motors to move forward
      SetPWM(2, pwr);
    } else if (motorState == FORWARD) {
      motorState = REVERSE;
      SetPWM(1, -1 * targetSpeed); // Set motors to move in reverse
      SetPWM(2, -pwr);
    } else if (motorState == REVERSE) {
      motorState = STOPPED; // Stop the motors indefinitely
      SetPWM(1, 0);
      SetPWM(2, 0);
      stopTime = millis(); // Reset the stop time when the switch is pressed in the REVERSE state
    }
  }
  lastSwitchState = currentSwitchState; // Update the last switch state

  if (motorState == STOPPED && encoderCount2 == 0) {
    SetPWM(1, 0);
    SetPWM(2, 0);
    motorState = STOPPED;
    stopTime = millis(); // Reset the stop time when the encoder count reaches the threshold
  }

  if (motorState == REVERSE && encoderCount2 <= 133) {
    SetPWM(1, 0);
    SetPWM(2, 0);
    motorState = STOPPED;
    stopTime = millis(); // Reset the stop time when the encoder count reaches the threshold
  }

  // Check if the motors have been stopped for 10 seconds
  if (motorState == STOPPED && millis() - stopTime >= 9000) {
    motorState = FORWARD; // Restart the motors in the forward direction
    SetPWM(1, targetSpeed / 3);
    SetPWM(2, pwr / 3);
  }

  if (motorState == FORWARD && millis() - stopTime >= 10000 && lux < LUX_THRESHOLD)  {
    estimatedTargetEncoderCount = encoderCount2 + 39;
    motorState = FORWARD;
  }

  // Check if the motors have reached the target encoder count plus the extra encoder count
  if (motorState == FORWARD && encoderCount2 > estimatedTargetEncoderCount) {
    SetPWM(1, 0);
    SetPWM(2, 0);
    motorState = STOPPED;
  }
  Serial.print("Motor 1 ");
  Serial.print(encoderCount1);
  Serial.print(",");
  Serial.print("Motor 2 ");
  Serial.print(encoderCount2);
  Serial.print(",");
  Serial.print("Estimated count ");
  Serial.print(estimatedTargetEncoderCount);
  Serial.print(",");
  Serial.print("Lux: ");
  Serial.println(lux, DEC);
  delay(10);

}
void readEncoder1() {
  int b1 = digitalRead(ENCODER_E1B_PIN);
  if (b1 > 0) {
    encoderCount1++;
  }
  else {
    encoderCount1--;
  }
}
void readEncoder2() {
  int b2 = digitalRead(ENCODER_E2B_PIN);
  if (b2 > 0) {
    encoderCount2--;
  }
  else {
    encoderCount2++;
  }
}
