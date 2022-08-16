#include "encoders.h"
#include "kinematics.h"
#include <Wire.h>
//#include <Zumo32U4.h>

Kinematics_c kinematics;

#define I2C_ADDR 8

// Defines motor directions
#define FWD LOW
#define REV HIGH

// Defines motor pins
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define POSE_PACKET 0
#define FORCE_PACKET 1
#define UNCERTAINTY_PACKET 2

#define SPEED_SCALE (255.0 / 1.5);
#define TURNING_MULTIPLIER 2;

#define MAX_SPEED 0.25

float force_x = 0;
float force_y = 0;
float goal_angle = 0;
float position_uncertainty = 0.7;
float rotation_uncertainty = 0.9;
float leftVel = 30;
float rightVel = 30;

// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device
// Note that the Arduino is limited to
// a buffer of 32 bytes!
#pragma pack(1)
typedef struct i2c_status {
  float x;             // 4 bytes
  float y;             // 4 bytes
  float theta;         // 4 bytes
  int8_t status;       // 1 byte
  int8_t packet_type;  // 1 byte
} i2c_status_t;
#pragma pack()

// Data sent to and from the M5Stack
i2c_status_t i2c_status_tx;
volatile i2c_status_t i2c_status_rx;

// Drives the left motor
// Positive velocity is forward, negative reverse
void setLeftMotor(int velocity) {
  if (velocity >= 0) {
    digitalWrite(L_DIR_PIN, FWD);
  }
  else {
    digitalWrite(L_DIR_PIN, REV);
  }

  analogWrite(L_PWM_PIN, abs(velocity));
}

// Drives the right motor
// Positive velocity is forward, negative reverse
void setRightMotor(int velocity) {
  if (velocity >= 0) {
    digitalWrite(R_DIR_PIN, FWD);
  }
  else {
    digitalWrite(R_DIR_PIN, REV);
  }

  analogWrite(R_PWM_PIN, abs(velocity));
}

// When the Core2 calls an i2c request, this function
// is executed.  Sends robot status to Core2.
void i2c_sendStatus() {
  // Populate our current status
  i2c_status_tx.x = kinematics.x_global;
  i2c_status_tx.y = kinematics.y_global;
  i2c_status_tx.theta = kinematics.currentRotationCutoff;

  // Send up
  Wire.write((byte*)&i2c_status_tx, sizeof(i2c_status_tx));
}

float circular_uncertainty_mean(float cam_angle, float odo_angle) {
  float sin_sum = sin(cam_angle) * (1 - rotation_uncertainty) +
                  sin(kinematics.currentRotationCutoff) * rotation_uncertainty;
  float cos_sum = cos(cam_angle) * (1 - rotation_uncertainty) +
                  cos(kinematics.currentRotationCutoff) * rotation_uncertainty;
  return atan2(sin_sum, cos_sum);
}

// When the Core2 calls and i2c write, the robot
// will call this function to receive the data down.
void i2c_recvStatus(int len) {
  // Read the i2c status sent by the Core2
  Wire.readBytes((byte*)&i2c_status_rx, sizeof(i2c_status_rx));

  if (i2c_status_rx.packet_type == FORCE_PACKET) {
    // Recieves a force packet and sets the robots current force vector
    force_x = i2c_status_rx.x;
    force_y = i2c_status_rx.y;

    float angle = atan2(force_y, force_x);
    goal_angle = angle;
  }
  else if (i2c_status_rx.packet_type == POSE_PACKET) {
    // Recieves a pose packet and updates the robots kinematics
    // The greater the uncertainty the less the kinematics are changed
    kinematics.x_global = i2c_status_rx.x * (1 - position_uncertainty) +
                          kinematics.x_global * position_uncertainty;
    kinematics.y_global = i2c_status_rx.y * (1 - position_uncertainty) +
                          kinematics.y_global * position_uncertainty;
    kinematics.currentRotationCutoff = circular_uncertainty_mean(
        i2c_status_rx.theta, kinematics.currentRotationCutoff);
  }
  else if (i2c_status_rx.packet_type == UNCERTAINTY_PACKET) {
    // Recieves an uncertanty packet and sets the robot's uncertainties
    position_uncertainty = i2c_status_rx.x;
    rotation_uncertainty = i2c_status_rx.theta;
  }
}

// Reads the battery voltage
inline uint16_t readBatteryMillivolts() {
  const uint8_t sampleCount = 8;
  uint16_t sum = 0;
  for (uint8_t i = 0; i < sampleCount; i++) {
    sum += analogRead(A1);
  }

  // VBAT = 2 * millivolt reading = 2 * raw * 5000/1024
  //      = raw * 625 / 64
  // The correction term below makes it so that we round to the
  // nearest whole number instead of always rounding down.
  const uint32_t correction = 32 * sampleCount - 1;
  return ((uint32_t)sum * 625 + correction) / (64 * sampleCount);
}

int battery_millivolts;

void warning_beep() {
  tone(6, 1000);
  delay(200);
  tone(6, 1500);
  delay(200);
  noTone(6);
  delay(300);
}

// Checks for low battery
void check_battery() {
  battery_millivolts = readBatteryMillivolts();
  if (battery_millivolts / 4 < 700 && battery_millivolts > 20) {
    warning_beep();
  }
}

void setup() {
  check_battery();

  // Sets up motor output pins
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  pinMode(A1, INPUT);

  // Stops both motors
  setLeftMotor(0);
  setRightMotor(0);

  setupEncoder0();
  setupEncoder1();

  // Serial for debugging.
  Serial.begin(9600);
  Serial.println("***RESTART***");
  delay(1000);

  // Clear out i2c data structs
  memset((void*)&i2c_status_tx, 0, sizeof(i2c_status_tx));
  memset((void*)&i2c_status_rx, 0, sizeof(i2c_status_rx));

  // Begin I2C as a slave device.
  Wire.begin(I2C_ADDR);
  Wire.onRequest(i2c_sendStatus);
  Wire.onReceive(i2c_recvStatus);

  goal_angle = 0.0;
}

void go_forward(float vel) {
  setLeftMotor(vel);
  setRightMotor(vel);
  leftVel = vel;
  rightVel = vel;
}

// Sets value to be between -pi and pi
float between_pi(float angle) {
  while (abs(angle) > PI) {
    if (angle > 0) {
      angle -= 2 * PI;
    }
    else {
      angle += 2 * PI;
    }
  }

  return angle;
}

int last_check_time = 0;
int current_time = 0;

// Prevents the 3pi motors from stalling (they don't like pwm values of less
// than 20)
int min_pwm(int pwm) {
  if (abs(pwm) < 20) {
    return 0;
  }

  if (abs(pwm) < 25) {
    if (pwm > 0) {
      pwm = 25;
    }
    else {
      pwm = -25;
    }
  }

  return pwm;
}

void loop() {
  float current_angle = kinematics.currentRotationCutoff;
  float newGoal = goal_angle;
  float max_speed = MAX_SPEED;
  float speed = 0;

  // Error is difference between current and goal angles
  float turn_error = (goal_angle - current_angle);

  // Turn direction is not inverted by default
  int turnDirection = 1;

  // Check battery level every 20s
  current_time = millis();
  if (current_time - last_check_time > 5000) {
    check_battery();
    last_check_time = millis();
  }

  current_angle = between_pi(current_angle);
  turn_error = between_pi(turn_error);

  // If error is more than a quarter turn put robot into reverse
  if (abs(turn_error) > PI / 2) {
    // goal angle is a half rotation away from the actual goal when reversing
    if (goal_angle > 0) {
      newGoal = goal_angle - PI;
    }
    else {
      newGoal = goal_angle + PI;
    }

    // Speed is negative when reversing
    max_speed = -MAX_SPEED;
    speed = -sqrt(force_x * force_x + force_y * force_y);

    // Turn direction is inverted when reversing
    turnDirection = -1;
  }
  else {
    // Set speed to the magnitude of the force vector
    speed = sqrt(force_x * force_x + force_y * force_y);
  }

  turn_error = (newGoal - current_angle) * turnDirection;  // recheck turn_error
  turn_error = between_pi(turn_error);

  // Only move if force is not close to 0
  if (force_x * force_x + force_y * force_y > 0.001) {
    // Dont move at more than max speed
    if (abs(speed) > abs(max_speed)) {
      speed = max_speed;
    }

    // Convert speed from metres per second to pwm values
    speed *= SPEED_SCALE;

    // Turn if error is too large
    if (abs(turn_error) > 0.2) {
      // Increase error by turning multiplier, for sharper turns
      float scaled_error = turn_error * TURNING_MULTIPLIER;
      if (scaled_error > PI) {
        scaled_error = PI;
      }
      else if (scaled_error < -PI) {
        scaled_error = -PI;
      }

      if (turn_error > 0) {
        // Slow down left wheel when turning left
        leftVel = speed * cos(scaled_error);
        rightVel = speed;
      }
      else {
        // Slow down right wheel when turning right
        leftVel = speed;
        rightVel = speed * cos(scaled_error);
      }

      setLeftMotor(min_pwm(leftVel));
      setRightMotor(min_pwm(rightVel));
    }
    else {
      go_forward(min_pwm(speed));
    }
  }
  else {
    go_forward(0);
  }

  // Check encoders to get new x, y and angle
  kinematics.updateLoop();
}

void printRXStatus() {
  Serial.println(i2c_status_rx.x);
  Serial.println(i2c_status_rx.y);
  Serial.println(i2c_status_rx.theta);
  Serial.println(i2c_status_rx.status);
}
