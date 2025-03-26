//
// Created by leslier on 12/8/2024.
//



#if defined(ROS) || defined(ROS_DEBUG)
#include "microRosFunctions.h"
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

//Used to subscribe to raw joystick data or processed reference speed
#ifdef ROS_DEBUG
#include <wheelchair_sensor_msgs/msg/ref_speed.h>
#include <wheelchair_sensor_msgs/msg/dac_values.h>
#include <wheelchair_sensor_msgs/msg/sensors.h>
#elif ROS
#include <wheelchair_sensor_msgs/msg/ref_speed.h>
#endif

//Publisher
// rcl_publisher_t publisher;
// wheelchair_sensor_msgs__msg__Sensors sensorMsg;
rclc_executor_t executor;

//Subscriber
rcl_subscription_t subscriber;
#ifdef ROS_DEBUG
wheelchair_sensor_msgs__msg__Sensors refSpeedMsg;
rcl_publisher_t dacPublisher;
wheelchair_sensor_msgs__msg__DacValues dacMsg;
#elif ROS
wheelchair_sensor_msgs__msg__RefSpeed refSpeedMsg;
#endif

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

#ifdef ROS_DEBUG
void timer_callback(rcl_timer_t * inputTimer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
        RCSOFTCHECK(rcl_publish(&dacPublisher, &dacMsg, NULL));
    }
}

void transmitDac(int16_t leftDacValue, int16_t rightDacValue) {
    dacMsg.left_dac = leftDacValue;
    dacMsg.right_dac = rightDacValue;
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
#endif

void subscription_callback(const void *msgin)
{
#ifdef ROS_DEBUG
    const wheelchair_sensor_msgs__msg__Sensors *msg = (const wheelchair_sensor_msgs__msg__Sensors *)msgin;
    refSpeedMsg = *msg;
    //Try using Serial1 to use a Uart adapter to print this out
//    Serial1.print("Left Speed: ");
//    Serial1.println(msg->left_speed);
//    Serial1.print("Right Speed: ");
//    Serial1.println(msg->right_speed);


    // if (msg->left_speed == 100 || msg->right_speed == 100) {
    //     digitalWrite(LED_BUILTIN, HIGH);
    // } else {
    //     digitalWrite(LED_BUILTIN, LOW);
    // }
#elif ROS
    const wheelchair_sensor_msgs__msg__RefSpeed *msg = (const wheelchair_sensor_msgs__msg__RefSpeed *)msgin;
    refSpeedMsg = *msg;
#endif
}

void microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* subTopicName, const char* pubTopicName){
    set_microros_serial_transports(Serial);
    delay(2000);
    allocator = rcl_get_default_allocator();

    // Set the domain ID
    const size_t domain_id = 7; // Replace with your desired domain ID

    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // create node
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

//    // create publisher
//    RCCHECK(rclc_publisher_init_default(
//            &publisher,
//            &node,
//            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, sensorMsg, Sensors),
//            pubTopicName));

    // Create Subscriber
#ifdef ROS
    RCCHECK(rclc_subscription_init_best_effort(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, RefSpeed),
            subTopicName));
#elif ROS_DEBUG
    // RCCHECK(rclc_subscription_init_default(
    //         &subscriber,
    //         &node,
    //         ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Sensors),
    //         subTopicName));

    RCCHECK(rclc_subscription_init_best_effort(
            &subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, RefSpeed),
            subTopicName));

    // create publisher
    RCCHECK(rclc_publisher_init_best_effort(
        &dacPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, DacValues),
        "dac_value"));

#endif



    // create timer,
    //unsigned int timer_timeout = 1;
#ifdef ROS_DEBUG
     RCCHECK(rclc_timer_init_default(
             &timer,
             &support,
             RCL_MS_TO_NS(timer_timeout),
             timer_callback));
#endif

    //create executor
    //Number of handles = # timers + # subscriptions + # clients + # services
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    // add sub to executor
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &refSpeedMsg, &subscription_callback, ON_NEW_DATA));

#ifdef ROS_DEBUG
    // add pub to executor
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    // sensorMsg.left_speed = 0;
    // sensorMsg.right_speed = 0;
    dacMsg.left_dac = 0;
    dacMsg.right_dac = 0;
#endif
}

// void transmitMsg(refSpeed omegaRef){
//     sensorMsg.left_speed = omegaRef.leftSpeed;
//     sensorMsg.right_speed = omegaRef.rightSpeed;
//
//     RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
// }

void checkSubs() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

refSpeed getRefSpeed() {
    refSpeed refSpeed;
    refSpeed.leftSpeed = refSpeedMsg.left_speed;
    refSpeed.rightSpeed = refSpeedMsg.right_speed;
    return refSpeed;
}

#endif