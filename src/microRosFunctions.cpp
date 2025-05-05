#include "microRosFunctions.h"
#include "BatteryFunctions.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Include ROS message types
#ifdef ROS_DEBUG
#include <wheelchair_sensor_msgs/msg/ref_speed.h>
#include <wheelchair_sensor_msgs/msg/dac_values.h>
#include <wheelchair_sensor_msgs/msg/sensors.h>
#include <wheelchair_sensor_msgs/msg/battery.h>
#include <wheelchair_sensor_msgs/msg/motors.h>
#elif ROS
#include <wheelchair_sensor_msgs/msg/ref_speed.h>
#include <wheelchair_sensor_msgs/msg/motors.h>
#endif

// External variables for actual speed
extern float speedL;
extern float speedR;

// ROS variables
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_subscription_t subscriber;
#ifdef ROS_DEBUG
wheelchair_sensor_msgs__msg__Sensors refSpeedMsg;
rcl_publisher_t dacPublisher;
wheelchair_sensor_msgs__msg__DacValues dacMsg;

rcl_publisher_t batteryPublisher;
rcl_timer_t batteryTimer;
wheelchair_sensor_msgs__msg__Battery batteryMsg;
#else
wheelchair_sensor_msgs__msg__RefSpeed refSpeedMsg;
#endif

rcl_publisher_t motorPublisher;
rcl_timer_t motorTimer;
wheelchair_sensor_msgs__msg__Motors motorMsg;

rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) {} }

void error_loop() {
    while (1) {
        delay(100);
    }
}

#ifdef ROS_DEBUG
void timer_callback(rcl_timer_t *inputTimer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (inputTimer != NULL) {
        RCSOFTCHECK(rcl_publish(&dacPublisher, &dacMsg, NULL));
    }
}

void battery_timer_callback(rcl_timer_t *input_timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (input_timer != NULL) {
        float voltage = readBatteryVoltage();
        int8_t percent = calculateBatteryPercentage(voltage);
        batteryMsg.battery_percent = percent;
        RCSOFTCHECK(rcl_publish(&batteryPublisher, &batteryMsg, NULL));
    }
}
#endif

void motor_timer_callback(rcl_timer_t *input_timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (input_timer != NULL) {
        motorMsg.left_mph = speedL;
        motorMsg.right_mph = speedR;
        RCSOFTCHECK(rcl_publish(&motorPublisher, &motorMsg, NULL));
    }
}

void transmitDac(int16_t leftDacValue, int16_t rightDacValue) {
#ifdef ROS_DEBUG
    dacMsg.left_dac = leftDacValue;
    dacMsg.right_dac = rightDacValue;
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
#endif
}

void subscription_callback(const void *msgin) {
#ifdef ROS_DEBUG
    const wheelchair_sensor_msgs__msg__Sensors *msg = (const wheelchair_sensor_msgs__msg__Sensors *)msgin;
    refSpeedMsg = *msg;
#else
    const wheelchair_sensor_msgs__msg__RefSpeed *msg = (const wheelchair_sensor_msgs__msg__RefSpeed *)msgin;
    refSpeedMsg = *msg;
#endif
}

void microRosSetup(unsigned int timer_timeout, const char *nodeName, const char *subTopicName, const char *pubTopicName) {
    set_microros_serial_transports(Serial);
    delay(2000);
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

    RCCHECK(rclc_subscription_init_best_effort(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, RefSpeed),
        subTopicName));

#ifdef ROS_DEBUG
    RCCHECK(rclc_publisher_init_best_effort(
        &dacPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, DacValues),
        "dac_value"));

    RCCHECK(rclc_publisher_init_default(
        &batteryPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Battery),
        "battery_status"));

    RCCHECK(rclc_timer_init_default(
        &batteryTimer,
        &support,
        RCL_MS_TO_NS(3000),
        battery_timer_callback));
#endif

    RCCHECK(rclc_publisher_init_default(
        &motorPublisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(wheelchair_sensor_msgs, msg, Motors),
        pubTopicName));

    RCCHECK(rclc_timer_init_default(
        &motorTimer,
        &support,
        RCL_MS_TO_NS(1000),
        motor_timer_callback));

#ifdef ROS_DEBUG
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback));
#endif

    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &refSpeedMsg, &subscription_callback, ON_NEW_DATA));

#ifdef ROS_DEBUG
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_timer(&executor, &batteryTimer));
#endif

    RCCHECK(rclc_executor_add_timer(&executor, &motorTimer));

#ifdef ROS_DEBUG
    dacMsg.left_dac = 0;
    dacMsg.right_dac = 0;
    batteryMsg.battery_percent = 0;
    initBatterySensor();
#endif

    motorMsg.left_mph = 0;
    motorMsg.right_mph = 0;
}

void checkSubs() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

refSpeed getRefSpeed() {
    refSpeed refSpeed;
    refSpeed.leftSpeed = refSpeedMsg.left_speed;
    refSpeed.rightSpeed = refSpeedMsg.right_speed;
    return refSpeed;
}

