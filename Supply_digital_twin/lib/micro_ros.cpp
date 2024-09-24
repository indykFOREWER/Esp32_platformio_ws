#include "micro_ros.h"
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

typedef enum {
    eSerial = 0,
    eWifi = 1
}transport;

typedef struct 
{

} micro_ros_handler;

rcl_allocator_t allocator;
rclc_support_t support;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
void error_loop() {
  while(1) {
    delay(100);
  }
}

bool MicroRosInit(transport transport, Stream & serial) {

    switch (transport)
    {
    case eSerial:
        // Configure serial transport
        set_microros_serial_transports(serial);
        delay(2000);
    case eWifi:
        // TODO: implement wifi
        return false;
    default:
        return false;
    }

        allocator = rcl_get_default_allocator();

        //create init_options
        RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
        return true;
}

micro_ros_handler MicroRosMakeNode() {

}