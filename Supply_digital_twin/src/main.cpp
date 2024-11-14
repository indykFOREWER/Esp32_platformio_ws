#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <std_msgs/msg/float32.h>

#include "supply_twin.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
// std_msgs__msg__Int32 msg;
std_msgs__msg__Float32 msg;
std_msgs__msg__Float32 msg_sub_1;
std_msgs__msg__Float32 msg_sub_2;
std_msgs__msg__Float32 msg_sub_3;

rcl_subscription_t subscriber1;
rcl_subscription_t subscriber2;
rcl_subscription_t subscriber3;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


TaskHandle_t Task1;
TaskHandle_t Task2;
TaskHandle_t Task3;

// LED pins
const int led1 = 4;
const int led2 = 2;

void Task1code( void * pvParameters );
void Task2code( void * pvParameters );
void Task3code( void * pvParameters );

// Error handle loop
void error_loop() {
  while(1) {
    digitalWrite(led2, HIGH);
    delay(100);
    digitalWrite(led2, LOW);
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // static float time = 0.0f;  // Начальное время
    // float current_1 = msg_sub_1.data;
    // msg.data = calculate_voltage(current_1, time);

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // msg.data++;
  }
}

void subscription_callback1(const void * msgin)
{  if (msgin == NULL) return;
  const std_msgs__msg__Float32 * msg_sub_1 = (const std_msgs__msg__Float32 *)msgin;
  // digitalWrite(led2, (msg_sub_1->data <= 0) ? LOW : HIGH);  
}

void subscription_callback2(const void * msgin)
{  
  if (msgin == NULL) return;
  const std_msgs__msg__Float32 * msg_sub_2 = (const std_msgs__msg__Float32 *)msgin;
  // digitalWrite(led2, (msg_sub_2->data <= 0) ? LOW : HIGH);  
}

void subscription_callback3(const void * msgin)
{  
  if (msgin == NULL) return;
  const std_msgs__msg__Float32 * msg_sub_3 = (const std_msgs__msg__Float32 *)msgin;
  // digitalWrite(led2, (msg_sub_3->data <= 0) ? LOW : HIGH);  
}

hw_timer_t * calc_timer = NULL;      //H/W timer defining (Pointer to the Structure)
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
bool calc_timer_flag = false;
void IRAM_ATTR onTimer() {      //Defining Inerrupt function with IRAM_ATTR for faster access
 portENTER_CRITICAL_ISR(&timerMux);
 calc_timer_flag = true;
 portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  // Serial.begin(921600); 
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  calc_timer = timerBegin(0, 80, true);           	// timer 0, prescalar: 80, UP counting
  timerAttachInterrupt(calc_timer, &onTimer, true); 	// Attach interrupt
  timerAlarmWrite(calc_timer, 1000, true);  		// Match value= 1000000 for 1 sec. delay.
  timerAlarmEnable(calc_timer);           			// Enable Timer with interrupt (Alarm Enable)

    Serial.begin(2000000); 
  // Serial.print("Task1 running on core ");
  // Serial.println(xPortGetCoreID());
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_platformio_node_publisher"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber1,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_arduino_subscriber1"));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber2,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "micro_ros_arduino_subscriber2"));

    // create subscriber
  // RCCHECK(rclc_subscription_init_best_effort(
  //   &subscriber3,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
  //   "micro_ros_arduino_subscriber3"));

  // create timer,
  const unsigned int timer_timeout = 1;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    // 500 * 1000,
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber1, &msg_sub_1, &subscription_callback1, ALWAYS));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber2, &msg_sub_2, &subscription_callback2, ALWAYS));
  // RCCHECK(rclc_executor_add_subscription(&executor, &subscriber3, &msg_sub_3, &subscription_callback3, ALWAYS));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;

  for(int i =0; i < 3; i++) {
    digitalWrite(led2, HIGH);
    delay(250);
    digitalWrite(led2, LOW);
    delay(250);
  }
  
}

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  

  for(;;){
    // RCSOFTCHECK(rclc_executor_spin(&executor));
    // RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500)));
    // RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    // msg.data++;
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());

  for(;;){
    // // Serial.print("Task2 running on core ");
    // // Serial.println(xPortGetCoreID());
    digitalWrite(led2, HIGH);
    delay(111);
    digitalWrite(led2, LOW);
    delay(111);

    // RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
    // RCSOFTCHECK(rclc_executor_spin(&executor));
  }
}

void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));

  static float time = 0.0f;  // Начальное время
  if (calc_timer_flag) {
    portENTER_CRITICAL(&timerMux);
    calc_timer_flag = false;
    portEXIT_CRITICAL(&timerMux);
    float voltage = 0;
    float current_1 = msg_sub_1.data;
    msg.data = calculate_voltage(current_1, time);
    time += TIME_STEP;
  }
}