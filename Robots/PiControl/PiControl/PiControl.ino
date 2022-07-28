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

float global_x = 0;
float global_y = 0;
float goal = 0;

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
// Not currently used
void i2c_sendStatus()
{

  // Populate our current status
  i2c_status_tx.x = 123.456;
  i2c_status_tx.y = 789.1011;
  i2c_status_tx.theta = 12.13;
  i2c_status_tx.status--; // debugging

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

  global_x = i2c_status_rx.x;
  global_y = i2c_status_rx.y;
  kinematics.currentRotation = i2c_status_rx.theta;

  float angle = atan2(global_y, global_x);

  Serial.println((String) "Angle" + angle);

  // goal = -kinematics.currentRotation + atan2(global_y,global_x) ;
  goal = angle;
  Serial.println((String) "goal " + goal);
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
  setLeftMotor(-vel * 30);
  setRightMotor(vel * 30);
}

void go_forward(float vel)
{
  setLeftMotor(vel);
  setRightMotor(vel);
}

void loop()
{

  float theta = kinematics.currentRotation; // make minus as this gives angle in clockwise rotation (we're using anticlockwise)
  float error = goal - theta;
  /*if (global_x == 0.0 && global_y == 0.0){
    go_forward(0.0);
  }
  else{*/
  while (abs(theta) > PI)
  {
    if (theta > 0)
    {
      theta -= 2 * PI;
    }
    else
    {
      theta += 2 * PI;
    }
  }

  if (abs(error) > PI)
  {
    if (error > 0)
    {
      error -= 2 * PI;
    }
    else
    {
      error += 2 * PI;
    }
  }
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
    set_z_rotation(0);
    if (global_x * global_x + global_y * global_y > 0.001)
    {
      float speed = sqrt(global_x * global_x + global_y * global_y);
      if (speed > MAX_SPEED) {
        go_forward(50.0);
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
  Serial.println((String) "x: " + global_x);
  Serial.println((String) "y:" + global_y);

  kinematics.updateLoop();
  delay(100);
}

void printRXStatus()
{
  Serial.println(i2c_status_rx.x);
  Serial.println(i2c_status_rx.y);
  Serial.println(i2c_status_rx.theta);
  Serial.println(i2c_status_rx.status);
}
