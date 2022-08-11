#include <Wire.h>
#include "encoders.h"
#include "kinematics.h"
//#include <Zumo32U4.h>

Kinematics_c kinematics;

#define I2C_ADDR 8

//  Defines motor directions
#define FWD LOW
#define REV HIGH

//  Defines motor pins
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
int turnDirection = 1;

float speed;

float force_x = 0;
float force_y = 0;
float goal = 0;
float position_uncertainty = 0.3;
float rotation_uncertainty = 0.7;
float leftVel = 30;
float rightVel = 30;

// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device
// Note that the Arduino is limited to
// a buffer of 32 bytes!
#pragma pack(1)
typedef struct i2c_status
{
  float x;       // 4 bytes
  float y;       // 4 bytes
  float theta;   // 4 bytes
  int8_t status; // 1 byte
  int8_t packet_type; // 1 byte
} i2c_status_t;
#pragma pack()

//  Data sent to and from the M5Stack
i2c_status_t i2c_status_tx;
volatile i2c_status_t i2c_status_rx;

//  Drives the left motor
//  Positive velocity is forward, negative reverse
void setLeftMotor(int velocity)
{
  if (velocity >= 0)
  {
    digitalWrite(L_DIR_PIN, FWD);
  }
  else
  {
    digitalWrite(L_DIR_PIN, REV);
  }

  analogWrite(L_PWM_PIN, abs(velocity));
}

//  Drives the right motor
//  Positive velocity is forward, negative reverse
void setRightMotor(int velocity)
{
  if (velocity >= 0)
  {
    digitalWrite(R_DIR_PIN, FWD);
  }
  else
  {
    digitalWrite(R_DIR_PIN, REV);
  }

  analogWrite(R_PWM_PIN, abs(velocity));
}

// When the Core2 calls an i2c request, this function
// is executed.  Sends robot status to Core2.
void i2c_sendStatus()
{

  // Populate our current status
  i2c_status_tx.x = kinematics.x_global;
  i2c_status_tx.y = kinematics.y_global;
  i2c_status_tx.theta = kinematics.currentRotationCutoff;

  // Send up
  Wire.write((byte *)&i2c_status_tx, sizeof(i2c_status_tx));
}

// When the Core2 calls and i2c write, the robot
// will call this function to receive the data down.
void i2c_recvStatus(int len)
{
  //  Read the i2c status sent by the Core2
  Wire.readBytes((byte *)&i2c_status_rx, sizeof(i2c_status_rx));

 // Serial.println((String) "Message recieved");

  //  Set both motors to run at the speed of the status x value
  // setLeftMotor(i2c_status_rx.x);
  // setRightMotor(i2c_status_rx.y);

  if (i2c_status_rx.packet_type == FORCE_PACKET) {
    force_x = i2c_status_rx.x;
    force_y = i2c_status_rx.y;

    float angle = atan2(force_y, force_x);
    //Serial.println((String) "Angle" + angle);

    goal = angle;
  }
  else if (i2c_status_rx.packet_type == POSE_PACKET) {
    kinematics.x_global = i2c_status_rx.x * (1 - position_uncertainty) + kinematics.x_global * position_uncertainty;
    kinematics.y_global = i2c_status_rx.y * (1 - position_uncertainty) + kinematics.y_global * position_uncertainty;
    kinematics.currentRotationCutoff = i2c_status_rx.theta * (1 - rotation_uncertainty) + kinematics.currentRotationCutoff * rotation_uncertainty;
  }
  else if (i2c_status_rx.packet_type == UNCERTAINTY_PACKET) {
    position_uncertainty = i2c_status_rx.x;
    rotation_uncertainty = i2c_status_rx.theta;
  }

}

inline uint16_t readBatteryMillivolts()
 {
     const uint8_t sampleCount = 8;
     uint16_t sum = 0;
     for (uint8_t i = 0; i < sampleCount; i++)
     {
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

void setup()
{
  battery_millivolts = readBatteryMillivolts();
  if (battery_millivolts / 4 < 700 && battery_millivolts > 20) {
    tone(6, 1000);
    delay(200);
    tone(6, 1500);
    delay(200);
    noTone(6);
    delay(300);
  }
  
  //  Sets up motor output pins
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

  pinMode(A1, INPUT);

  //  Stops both motors
  setLeftMotor(0);
  setRightMotor(0);

  setupEncoder0();
  setupEncoder1();

  // Serial for debugging.
  Serial.begin(9600);
  Serial.println("***RESTART***");
  delay(1000);

  // Clear out i2c data structs
  memset((void *)&i2c_status_tx, 0, sizeof(i2c_status_tx));
  memset((void *)&i2c_status_rx, 0, sizeof(i2c_status_rx));

  // Begin I2C as a slave device.
  Wire.begin(I2C_ADDR);
  Wire.onRequest(i2c_sendStatus);
  Wire.onReceive(i2c_recvStatus);

  goal = 0.0;
}

void set_z_rotation(float vel)
{
  if (vel == 0) {
    setLeftMotor(0);
    setRightMotor(0);
  }
  else if (vel * 30 < 22 && vel > 0) {
    setLeftMotor(-22.0);
    setRightMotor(22.0);
  }
  else if (vel * 30 > -22 && vel < 0) {
    setLeftMotor(22.0);
    setRightMotor(-22.0);
  }
  else {
    setLeftMotor(-vel * 30);
    setRightMotor(vel * 30);
  }
}

void go_forward(float vel)
{
  setLeftMotor(vel);
  setRightMotor(vel);
  leftVel = vel ;
  rightVel = vel ;
}

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

int batteryTS = 0;
int currentTS = 0;

void loop() {
  float theta = kinematics.currentRotationCutoff; 
  float error = (goal - theta);
  float newGoal = goal;
  float max_speed = MAX_SPEED;

  currentTS = millis();
  if (currentTS - batteryTS > 20000) {
    battery_millivolts = readBatteryMillivolts();
    if (battery_millivolts / 4 < 700 && battery_millivolts > 20) {
      tone(6, 1000);
      delay(200);
      tone(6, 1500);
      delay(200);
      noTone(6);
      delay(300);
    }
    batteryTS = millis();
  }

  theta = between_pi(theta);
  error = between_pi(error);

  if (abs(error) > PI / 2) {
    if (goal > 0) {
      newGoal = goal - PI;
    }
    else {
      newGoal = goal + PI;
    }
    max_speed = -MAX_SPEED;
    speed = -sqrt(force_x * force_x + force_y * force_y);
    //Serial.println((String) "goal set as " + goal);
    turnDirection = -1;
  }
  else {
    speed = sqrt(force_x * force_x + force_y * force_y);
    turnDirection = 1;
  }

  error = (newGoal - theta) * turnDirection; // recheck error
  error = between_pi(error);

  if (force_x * force_x + force_y * force_y > 0.001) {
    if (abs(speed) > abs(max_speed)) {
      speed = max_speed;
    }
    speed *= SPEED_SCALE;
    if (abs(speed) < 25.0) {
      if (speed > 0) {
        speed=25.0;
      }
      else {
        speed=-25.0;
      }
    }

    if (abs(error) > 0.2) {
      float scaled_error = error * TURNING_MULTIPLIER;
      if (scaled_error > PI) {
        scaled_error = PI;
      }
      else if (scaled_error < -PI) {
        scaled_error = -PI;
      }

      if (error > 0) {
        leftVel = speed * cos(scaled_error);
        rightVel = speed;
      }
      else {
        // you need to turn anticlockwise, need to increase right wheel 
        leftVel = speed;
        rightVel = speed * cos(scaled_error);
      }
      setLeftMotor(leftVel);
      setRightMotor(rightVel);
    }
    else {
      go_forward(speed);
    }
  }
  else {
    go_forward(0);
  }

  //Serial.println((String) "Error: " + error);
  //Serial.println((String) "Desired angle: " + goal);
  //Serial.println((String) "Angle of robot:" + theta);

  kinematics.updateLoop();
  // delay(1);
}

void printRXStatus()
{
  Serial.println(i2c_status_rx.x);
  Serial.println(i2c_status_rx.y);
  Serial.println(i2c_status_rx.theta);
  Serial.println(i2c_status_rx.status);
}
