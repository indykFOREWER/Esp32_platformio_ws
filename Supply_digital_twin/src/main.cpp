#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handle loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

TaskHandle_t Task1;
TaskHandle_t Task2;

// LED pins
const int led1 = 4;
const int led2 = 2;

void Task1code( void * pvParameters );
void Task2code( void * pvParameters );

void setup() {
  // Serial.begin(921600); 
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    2,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

//Task1code: blinks an LED every 1000 ms
void Task1code( void * pvParameters ){
  Serial.begin(921600); 
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
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_platformio_node_publisher"));

  // create timer,
  // const unsigned int timer_timeout = 1;
  // RCCHECK(rclc_timer_init_default(
  //   &timer,
  //   &support,
  //   RCL_MS_TO_NS(timer_timeout),
  //   timer_callback));

  // // create executor
  // RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;

  for(;;){
    // RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  } 
}

//Task2code: blinks an LED every 700 ms
void Task2code( void * pvParameters ){
  // Serial.print("Task2 running on core ");
  // Serial.println(xPortGetCoreID());

  for(;;){
    // Serial.print("Task2 running on core ");
    // Serial.println(xPortGetCoreID());
    digitalWrite(led2, HIGH);
    delay(111);
    digitalWrite(led2, LOW);
    delay(111);
  }
}

void loop() {
  
}