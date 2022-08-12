#include <M5Core2.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rclc/node.h>
#include <rclc/subscription.h>
#include <rcutils/strerror.h>
#include <micro_ros_utilities/type_utilities.h>

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/color_rgba.h>
#include <geometry_msgs/msg/vector3.h>
#include <geometry_msgs/msg/pose.h>
#include <lifecycle_msgs/msg/state.h>
#include <lifecycle_msgs/srv/get_state.h>

#include <Wire.h>

#include <arduino-timer.h>

//  ROS error handlers. Calls error_loop if check fails.
#define RCCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){char message[128];sprintf(message, "Error on line %d with status %d. Aborting.\n", __LINE__, (int)temp_rc);M5.lcd.println(message);error_loop();}}
#define RCSOFTCHECK(fn) {rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){/*char message[128];sprintf(message, "Error on line %d with status %d. Continuing.\n", __LINE__, (int)temp_rc);M5.lcd.println(message);*/}}

#define ROBOT_I2C_ADDR  8

//  Stack screen width + height
#define WIDTH 320
#define HEIGHT 240

//  Identifiers for different types of packet for communication with the 3Pi
#define POSE_PACKET 0
#define FORCE_PACKET 1
#define UNCERTAINTY_PACKET 2

//  Time between timer updates, in milliseconds
#define TIMER_RATE 200

//  Stops the robot if no vectors are recieved after VECTOR_STOP_TIME timer updates.
#define VECTOR_STOP_TIME 5


// Data to send(tx) and receive(rx)
// on the i2c bus.
// Needs to match the master device
#pragma pack(1)
typedef struct i2c_status {
  float x;                  // 4 bytes
  float y;                  // 4 bytes
  float theta;              // 4 bytes
  uint8_t status;           // 1 byte
  uint8_t packet_type;           // 1 byte
} i2c_status_t;
#pragma pack()

//  Data sent to and from the 3Pi robot
i2c_status_t i2c_status_tx;
i2c_status_t i2c_status_rx;

//  Data for sending heartbeat messages
rcl_publisher_t heartbeat_publisher;
std_msgs__msg__Bool heartbeat_msg;

//  Data for subscribing to vector
rcl_subscription_t vector_subscriber;
geometry_msgs__msg__Vector3 vector_msg;

//  Data for subscribing to pose messages from the camera
rcl_subscription_t camera_pose_subscriber;
geometry_msgs__msg__Pose camera_pose_msg;

//  Data for subscribing to uncertainty messages
rcl_subscription_t uncertainty_subscriber;
geometry_msgs__msg__Vector3 uncertainty_msg;

//  Data for publishing pose messages
rcl_publisher_t pose_publisher;
geometry_msgs__msg__Pose pose_msg;

//  Data for subscribing to marker messages
rcl_subscription_t marker_subscriber;
std_msgs__msg__Int64 marker_msg;

//  Data for subscribing to colour messages
rcl_subscription_t colour_subscriber;
std_msgs__msg__ColorRGBA colour_msg;

//  Colour for the screen background
u_int32_t colour = WHITE;

//  Data for publishing registration messages
rcl_publisher_t register_publisher;
std_msgs__msg__Int32 register_msg;

//  Data for subscribing to id messages
rcl_subscription_t id_subscription;
std_msgs__msg__Int32 id_msg;

//  Data for ROS
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t setup_node;
rcl_node_t node;
rclc_executor_t setup_executor;
rclc_executor_t executor;

int id = -1;
bool configured = false;

uint timer_calls_without_vector = 0;

int64_t marker = 0;

Timer<10> timer;

//  Stops and prints an error.
void error_loop(){
  while(1){
    delay(25565);
  }
}

//  Draws an aruco marker to the screen
void drawMarker(u_int64_t data, uint32_t background) {
  M5.lcd.clear();

  int size = HEIGHT / 10;
  int side_inset = (WIDTH - HEIGHT) / 2;

  // Draw borders in the background colour
  M5.lcd.fillRect(0, 0, WIDTH, size, background);
  M5.lcd.fillRect(0, 0, size + side_inset, HEIGHT, background);
  M5.lcd.fillRect(0, HEIGHT - size, WIDTH, size, background);
  M5.lcd.fillRect(WIDTH - size - side_inset, 0, size + side_inset, HEIGHT, background);

  // Read the binary encoding of the marker and draw it to the screen
  for (u_int64_t i = 0; i < 36; i++) {
    bool white = (data & ((u_int64_t)1 << i)) != 0;

    int x = side_inset + (i % 6 + 2) * size;
    int y = (i / 6 + 2) * size;
    if (white) {
      M5.lcd.fillRect(x, y, size, size, WHITE);
    }
  }

  // Print the robot id
  char s[32]; 
  sprintf(s, "%d", id);
  M5.lcd.drawString(s, 0, 0);
}

// Handles vector messages recieved from a ROS subscription
void vector_callback(const void * msgin)
{
  //  Cast received message to vector
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;

  //  Converts message to i2c_status
  i2c_status_tx.x = msg->x;
  i2c_status_tx.y = msg->y;
  i2c_status_tx.theta = msg->z;
  i2c_status_tx.status = 0;
  i2c_status_tx.packet_type = FORCE_PACKET;

  //  Sends i2c_status to the 3Pi
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write((uint8_t*)&i2c_status_tx, sizeof(i2c_status_tx));
  Wire.endTransmission();

  timer_calls_without_vector = 0;
}

// Handles camera pose messages recieved from a ROS subscription
void camera_pose_callback(const void * msgin)
{
  //  Cast received message to pose
  const geometry_msgs__msg__Pose * msg = (const geometry_msgs__msg__Pose *)msgin;

  //  Converts message to i2c_status
  i2c_status_tx.x = msg->position.x;
  i2c_status_tx.y = msg->position.y;
  i2c_status_tx.theta = msg->orientation.z;
  i2c_status_tx.status = 0;
  i2c_status_tx.packet_type = POSE_PACKET;

  //  Sends i2c_status to the 3Pi
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write((uint8_t*)&i2c_status_tx, sizeof(i2c_status_tx));
  Wire.endTransmission();
}

// Handles uncertainty messages recieved from a ROS subscription
void uncertainty_callback(const void * msgin)
{
  //  Cast received message to vector
  const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;

  //  Converts message to i2c_status
  i2c_status_tx.x = msg->x;
  i2c_status_tx.y = msg->y;
  i2c_status_tx.theta = msg->z;
  i2c_status_tx.status = 0;
  i2c_status_tx.packet_type = UNCERTAINTY_PACKET;

  //  Sends i2c_status to the 3Pi
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  Wire.write((uint8_t*)&i2c_status_tx, sizeof(i2c_status_tx));
  Wire.endTransmission();
}

// Handles marker messages recieved from a ROS subscription
void marker_callback(const void * msgin)
{
  // Cast received message to int
  const std_msgs__msg__Int64 * msg = (const std_msgs__msg__Int64 *)msgin;

  // Sets the marker to the recieved value
  marker = msg->data;

  //  Draws the marker to the screen
  drawMarker(marker, colour);
}

// Handles id messages
void id_callback(const void * msgin) {
  //  Cast received message to int
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // Sets the id to the recieved value
  id = msg->data;

  // Prints acknowledgement of the id
  char s[32];
  sprintf(s, "Received id: %d", id);
  Serial.println(s);
  M5.lcd.println(s);
}

// Handles colour messages
void colour_callback(const void * msgin) {
  //  Cast received message to colour
  const std_msgs__msg__ColorRGBA * msg = (const std_msgs__msg__ColorRGBA *)msgin;

  // Converts colour floats (0 - 1) to bytes (0 - 255)
  uint32_t r = 255 * msg->r;
  uint32_t g = 255 * msg->g;
  uint32_t b = 255 * msg->b;

  // Sets the colour
  colour = M5.lcd.color565(r, g, b);

  // Redraws the marker with the new background colour
  drawMarker(marker, colour);
}

// Configures the robot to use its new id
void configure_robot() {
  //  Removes setup ros structures
  Serial.println("Removing setup ROS node.");
  RCCHECK(rclc_executor_remove_subscription(&setup_executor, &id_subscription));
  RCCHECK(rcl_publisher_fini(&register_publisher, &setup_node));
  RCCHECK(rcl_subscription_fini(&id_subscription, &setup_node));
  RCCHECK(rcl_node_fini(&setup_node));
  RCCHECK(rclc_executor_fini(&setup_executor));

  // Waits because micro ros doesn't 
  delay(500);
  
  Serial.println("Initialising unique ROS node.");

  //  Create a ROS node
  char node_name[32];
  sprintf(node_name, "robot%d", id);
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));

  //  Counts the number of handles (subscriptions, timers, etc) being used
  size_t handle_count = 0;

  // Creates a publisher for registration
  RCCHECK(rclc_publisher_init_default(
    &register_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/setup/register"));

  // Creates a publisher for heartbeats
  char heartbeat_topic_name[32];
  sprintf(heartbeat_topic_name, "/robot%d/heartbeat", id);
  RCCHECK(rclc_publisher_init_default(
    &heartbeat_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    heartbeat_topic_name));

  // Creates a publisher for poses
  char pose_topic_name[32];
  sprintf(pose_topic_name, "/robot%d/poses", id);
  RCCHECK(rclc_publisher_init_default(
    &pose_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
    pose_topic_name));

  //  Subscribe to the vector ROS topic, using Vector3 messages
  char vector_topic_name[32];
  sprintf(vector_topic_name, "/robot%d/vectors", id);
  RCCHECK(rclc_subscription_init_default(
    &vector_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    vector_topic_name));
  handle_count++;

  //  Subscribe to the camera poses ROS topic, using Pose messages
  char camera_pose_topic_name[32];
  sprintf(camera_pose_topic_name, "/robot%d/camera_poses", id);
  RCCHECK(rclc_subscription_init_default(
    &camera_pose_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose),
    camera_pose_topic_name));
  handle_count++;

  //  Subscribe to the uncertainty ROS topic, using Vector3 messages
  char uncertainty_topic_name[32];
  sprintf(uncertainty_topic_name, "/global/uncertainty", id);
  RCCHECK(rclc_subscription_init_default(
    &uncertainty_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
    uncertainty_topic_name));
  handle_count++;

  //  Subscribe to the marker ROS topic, using Int64 messages
  char marker_topic_name[32];
  sprintf(marker_topic_name, "/robot%d/markers", id);
  RCCHECK(rclc_subscription_init_default(
    &marker_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64),
    marker_topic_name));
  handle_count++;

  //  Subscribe to the colour ROS topic, using colour messages
  char colour_topic_name[32];
  sprintf(colour_topic_name, "/robot%d/colours", id);
  RCCHECK(rclc_subscription_init_default(
    &colour_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, ColorRGBA),
    colour_topic_name));
  handle_count++;

  // Sets up the executor
  RCCHECK(rclc_executor_init(&executor, &support.context, handle_count, &allocator));

  //  Adds the vector subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &vector_subscriber, &vector_msg,
    &vector_callback, ON_NEW_DATA));

  //  Adds the pose subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &camera_pose_subscriber, &camera_pose_msg,
    &camera_pose_callback, ON_NEW_DATA));

  //  Adds the vector subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &uncertainty_subscriber, &uncertainty_msg,
    &uncertainty_callback, ON_NEW_DATA));
  
  //  Adds the marker subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &marker_subscriber, &marker_msg,
    &marker_callback, ON_NEW_DATA));
  
  //  Adds the colour subscription to the executor
  RCCHECK(rclc_executor_add_subscription(
    &executor, &colour_subscriber, &colour_msg,
    &colour_callback, ON_NEW_DATA));

  // wait because microros doesnt and will break
  delay(500);

  // starts a timer to call the timer callback function at regular intervals
  timer.every(TIMER_RATE, timer_callback);

  // send acknowledgement of the recieved id back to the server
  Serial.println("Sending id acknowledgement.");
  register_msg.data = id;
  RCCHECK(rcl_publish(&register_publisher, &register_msg, NULL));
}

void setup() {
  //  Set up stack
  M5.begin();
  M5.lcd.setTextSize(3);

  //  Set up serial connection for debugging
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  //  Set up wire to communicate with 3Pi
  Wire.begin();

  //  Connect to micro ROS agent
  Serial.println("Connecting to WiFi.");
  set_microros_wifi_transports("TP-Link_102C", "35811152", "192.168.0.230", 8888);

  //  Wait because ros doesnt work properly
  delay(500);

  //  Set up ROS setup stuff
  Serial.println("WiFi connected. Initialising setup ROS node.");
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  //  Create a ROS node
  RCCHECK(rclc_node_init_default(&setup_node, "temporary_robot_setup_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
    &register_publisher,
    &setup_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/setup/register"));

  RCCHECK(rclc_subscription_init_default(
    &id_subscription,
    &setup_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/setup/ids"));

  //  Creates an executor to handle the subscriptions
  RCCHECK(rclc_executor_init(&setup_executor, &support.context, 1, &allocator));

  RCCHECK(rclc_executor_add_subscription(
    &setup_executor, &id_subscription, &id_msg,
    &id_callback, ON_NEW_DATA));

  delay(500);

  // Request an id from the server
  Serial.println("ROS initialised. Requesting id.");
  register_msg.data = -1;
  RCCHECK(rcl_publish(&register_publisher, &register_msg, NULL));
}

bool timer_callback(void* args) {
  RCSOFTCHECK(rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL));

  //  Gets current pose from 3Pi
  Wire.requestFrom(ROBOT_I2C_ADDR, sizeof(i2c_status_rx));
  Wire.readBytes((uint8_t*)&i2c_status_rx, sizeof(i2c_status_rx));

  //  Publishes current pose
  pose_msg.position.x = i2c_status_rx.x;
  pose_msg.position.y = i2c_status_rx.y;
  pose_msg.orientation.z = i2c_status_rx.theta;
  RCSOFTCHECK(rcl_publish(&pose_publisher, &pose_msg, NULL));

  // Send a zero vector to stop the 3pi if no vectors have been recieved recently
  if (timer_calls_without_vector >= VECTOR_STOP_TIME) {
    i2c_status_tx.x = 0;
    i2c_status_tx.y = 0;
    i2c_status_tx.theta = 0;
    i2c_status_tx.status = 0;
    i2c_status_tx.packet_type = FORCE_PACKET;

    //  Sends i2c_status to the 3Pi
    Wire.beginTransmission(ROBOT_I2C_ADDR);
    Wire.write((uint8_t*)&i2c_status_tx, sizeof(i2c_status_tx));
    Wire.endTransmission();
  }

  timer_calls_without_vector++;

  return true;
}

void loop() {
  //  Checks how long the timer has been running, and calls timer callback if needed
  timer.tick();

  if (!configured) {
    //  Checks for messages from the setup subscriptions if the robot isnt configured 
    RCCHECK(rclc_executor_spin_some(&setup_executor, RCL_MS_TO_NS(500)));

    //  Configures the robot if an id has been recieved
    if (id != -1) {
      configure_robot();
      configured = true;
    }
  }
  else {
    //  Checks for messages from the subscriptions
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500)));
  }
}

void printRXStatus() {
  Serial.println( i2c_status_rx.x ); 
  Serial.println( i2c_status_rx.y );
  Serial.println( i2c_status_rx.theta );
  Serial.println( i2c_status_rx.status );
}
