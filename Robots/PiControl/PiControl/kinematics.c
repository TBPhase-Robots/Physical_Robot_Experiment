  /**
      * Kinematics_c contains the odometry module used by PiControl

      Kinematics_c polls the wheel motor encoders and can determine the robot's local pose (position and orientation).

      This means the 3Pi+ knows how much to turn when given direction vectors.

      This is advantageous when the camera data cannot always be relied upon.

      

      */




// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _KINEMATICS_H
#define _KINEMATICS_H
#include "encoders.h"

#define WHEEL_RADIUS 0.017
#define ROBOT_DIAMETER 0.09
/* document*/
// Class to track robot position.
class Kinematics_c {
  public:
  
    // Constructor, must exist.
    Kinematics_c() {
      /**
      * Constructor for kinematics class
      */
    } 

    float count_wheel_left = 0;  /*!< Cumulative number of wheel encoder counts on the left wheel over lifetime, excluding most recent poll */
    float count_wheel_right = 0; /*!< Cumulative number of wheel encoder counts on the right wheel over lifetime, excluding most recent poll */

    float count_wheel_left_temp = 0; /*!< Cumulative number of wheel encoder counts on the left wheel over lifetime, includiing most recent poll */
    float count_wheel_right_temp = 0; /*!< Cumulative number of wheel encoder counts on the right wheel over lifetime, excluding most recent poll */

    float count_difference_left = 0; /*!< Difference between count_wheel_left_temp and count_wheel_left. Equal to left wheel encoder counts since last poll. */
    float count_difference_right = 0; /*!< Difference between count_wheel_right_temp and count_wheel_tight. Equal to right wheel encoder counts since last poll. */

    float count_difference_left_cum_mean = 0;
    float count_difference_right_cum_mean = 0;
    float n = 0; /*!< Number of encoder polls over lifetime */
    
    float x_global = 0;
    float y_global = 0;
    
    float distance_moved = 0;
    float displacement = 0;
    
    float x_global_debug = 0;
    float y_global_debug = 0;

    float lastTime = 0;
    float currentTime = 0;

    float interval = 0;
    float Xr = 0;
    float rotationalChange = 0;
    // float currentRotation = 0;
    float currentRotationCutoff = 0;
    float movementMultiplier = 3;

    float hallRatio = 358.3;

    // double pi = 3.14159265359;

    bool recordKinematics = true;
    
    // Use this function to update
    // your kinematics
    /* reset kinematics*/
    void resetKinematics(){

      /**
      * Resets all recorded kinematics properties to default values.
      */

      count_wheel_left = 0;
      count_wheel_right = 0;

      count_wheel_left_temp = 0;
      count_wheel_right_temp = 0;

      count_difference_left = 0;
      count_difference_right = 0;

      x_global = 0;
      y_global = 0;

      x_global_debug = 0;
      y_global_debug = 0;

      lastTime = 0;
      currentTime = 0;

      interval = 0;
      Xr = 0;
      rotationalChange = 0;
      // currentRotation = 0;
      currentRotationCutoff = 0;

      count_difference_left_cum_mean = 0;
      count_difference_right_cum_mean = 0;
      n = 0;

      distance_moved = 0;

      displacement = 0;
    }
    void enableKinematics(){
      /**
      * Turns on kinematics utility. Enables local odometry on the 3Pi+
      */
      recordKinematics = true;
    }

    
    void updateLoop() {


      /**
      * Kinematics update loop for the 3Pi+. Called once per update cycle at the end of the PiControl loop.
      Reads volatile wheel encoder values for left and right motor to determine robot rotation and position.
      */


      n ++;
      // get current time
      currentTime =  micros();
      interval = currentTime - lastTime;
      interval /= 1000000;

      // firstly, poll the encoders for the volatile rotation count values

      
      count_wheel_left_temp = count_e0;
      count_wheel_right_temp = count_e1;

      // correct abnormalities
      // no, WRONG abnormalities

      if (count_wheel_right_temp == count_wheel_left_temp + 1.0 || count_wheel_right_temp == count_wheel_left_temp + 2.0) count_wheel_right_temp = count_wheel_left_temp;
      else if (count_wheel_left_temp == count_wheel_right_temp + 1.0 || count_wheel_left_temp == count_wheel_right_temp + 2.0) count_wheel_left_temp = count_wheel_right_temp;

      
      
      // get difference in values compared to last saved ones
      count_difference_left = count_wheel_left_temp - count_wheel_left;
      count_difference_right = count_wheel_right_temp - count_wheel_right;
      count_difference_left *= -1;
      count_difference_right *= -1;
      // save new rotation values
      count_wheel_right = count_wheel_right_temp;
      count_wheel_left = count_wheel_left_temp;

      count_difference_left_cum_mean += count_wheel_left;
      count_difference_left_cum_mean /= n;

      count_difference_right_cum_mean += count_wheel_right;
      count_difference_right_cum_mean /= n;



      double wheelRotationalAmt = -(((double)count_difference_right - (double)count_difference_left) / (hallRatio)) * 2 * PI;
      double wheelForwardAmt = ((double)(count_difference_left + count_difference_right) / (hallRatio));
      // Xr = ((wheel radius * rotation velocity left) /2) + ((wheel radius * rotation velocity right) /2) * interval
      Xr = (wheelForwardAmt * 2 * WHEEL_RADIUS) / 2.0;
      distance_moved += Xr * movementMultiplier;
      
      // Yr = 0
      // L = distance between wheel and midpoint of bot
      // 0r (rotationalChange) = ((wheel radius * rotation velocity left) /2*L) + ((wheel radius * rotation velocity right) /2*L)
      //rotationalChange = ((0.017 * (count_difference_left / interval)) / (2.0* 0.045)) - ((0.017 * (count_difference_right / (interval))) / (2.0* 0.045));
      
      
      float robotRotationalAmt = (WHEEL_RADIUS * wheelRotationalAmt) / ROBOT_DIAMETER; 

      //robotRotationalVelocity /= 3.14159265359;

      rotationalChange = robotRotationalAmt;
     // rotationalChange = (0.017 * ((((count_difference_left - count_difference_right)/(12*30))) / interval)) / (2.0* 0.045);
      
      // define new rotation
      // defined as last rotaion in radians, plus rotational change
      // 0i = 0i + 0r
      // currentRotation += rotationalChange;
      currentRotationCutoff += rotationalChange;
      if(currentRotationCutoff > PI){
        currentRotationCutoff = currentRotationCutoff - 2 * PI;
      }
      else if (currentRotationCutoff < -PI){
        currentRotationCutoff = currentRotationCutoff + 2 * PI;
      }
      // convert local motion to global motion
      
      // Xdelta = Xr * cos(0i)
      // Ydelta = Xr * sin(0i)

      double Xdelta = Xr * cos(currentRotationCutoff) * movementMultiplier;
      double Ydelta = Xr * sin(currentRotationCutoff) * movementMultiplier;
      // update rotation matrix

      // apply new displacement by:
      /*
       * X(t+1) = X(t) + Xdelta
       * Y(t+1) = Y(t) + Ydelta
       */
      x_global += Xdelta;
      y_global += Ydelta;

      displacement = sqrt(x_global*x_global + y_global*y_global); 
      
      x_global_debug += Xdelta;
      y_global_debug += Ydelta;
       
//       Serial.println((String) "displacement is  " + displacement);
//       Serial.println((String) "wheelForwardAmt is  " + wheelForwardAmt);
//       Serial.println(Xr, 8);
//       Serial.println((String) "count_difference_left is " + count_difference_left);
//       Serial.println((String) "count_difference_right is " + count_difference_right);
//       Serial.println((String) "count_difference_left_cum_mean is " + count_difference_left_cum_mean);
//       Serial.println((String) "count_difference_right_cum_mean is " + count_difference_right_cum_mean);
//       Serial.println((String) "rotationalChange is " + rotationalChange);
//       Serial.println((String) "rotation is " + currentRotation + " which is " + currentRotation * 57.2958);
//       
//       Serial.println((String) "deltas are " + Xdelta + "," + Ydelta);
//       Serial.println((String) "coordinates are " + x_global + "," + y_global);
//       Serial.println("=================================================");
       
       lastTime = currentTime;
    }

};



#endif
