#include <Wire.h>
#include "encoders.h"
#include "kinematics.h"

Kinematics_c kinematics;

#define I2C_ADDR 8

//  Defines motor directions
#define FWD LOW
#define REV HIGH

#define SPEED_SCALE (255.0 / 1.5)
#define MAX_SPEED 0.4

//  Defines motor pins
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define POSE_PACKET 0
#define FORCE_PACKET 1
#define UNCERTAINTY_PACKET 2

float force_x = 0;
float force_y = 0;
float goal = 0;
float position_uncertainty = 0.3;
float rotation_uncertainty = 0.5;

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

  Serial.println((String) "Message recieved");

  //  Set both motors to run at the speed of the status x value
  // setLeftMotor(i2c_status_rx.x);
  // setRightMotor(i2c_status_rx.y);

  if (i2c_status_rx.packet_type == FORCE_PACKET) {
    force_x = i2c_status_rx.x;
    force_y = i2c_status_rx.y;

    float angle = atan2(force_y, force_x);
    Serial.println((String) "Angle" + angle);

    goal = angle;
    Serial.println((String) "goal " + goal);
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

void setup()
{
  //  Sets up motor output pins
  pinMode(L_DIR_PIN, OUTPUT);
  pinMode(L_PWM_PIN, OUTPUT);
  pinMode(R_DIR_PIN, OUTPUT);
  pinMode(R_PWM_PIN, OUTPUT);

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
}

float between_pi(float angle) {
  while (abs(angle) > PI)
  {
    if (angle > 0)
    {
      angle -= 2 * PI;
    }
    else
    {
      angle += 2 * PI;
    }
  }

  return angle;
}

void loop()
{

  float theta = kinematics.currentRotationCutoff; 
  float error = goal - theta;

  theta = between_pi(theta);
  error = between_pi(error);

  if (abs(error) > 0.2)
  {
    float limit = 0.75;
    if (abs(error) < limit)
    {
      if (error > 0)
      {
        error = limit;
      }
      else
      {
        error = -limit;
      }
    }
    set_z_rotation(error);
  }
  else
  {
    if (force_x * force_x + force_y * force_y > 0.001)
    {
      float speed = sqrt(force_x * force_x + force_y * force_y);
      if (speed > MAX_SPEED) {
        go_forward(MAX_SPEED * SPEED_SCALE);
      }
      else {
        speed *= SPEED_SCALE;
        if (speed < 20.0) {
          speed = 20.0;
        }
        go_forward(speed);
      }
    } else {
      go_forward(0);
    }
  }

  Serial.println((String) "Error: " + error);
  Serial.println((String) "Desired angle: " + goal);
  Serial.println((String) "Angle of robot:" + theta);
  Serial.println((String) "x: " + force_x);
  Serial.println((String) "y:" + force_y);

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
