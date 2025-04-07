//
// Created by leslier on 12/8/2024.
//

#ifndef MOTORMICROCONTROLLER_MICROROSFUNCTIONS_H
#define MOTORMICROCONTROLLER_MICROROSFUNCTIONS_H

#include <Arduino.h>
#include "RefSpeed.h"

/**
 * Function to setup the microROS node.
 * @param timer_timeout The timeout for the timer.
 * @param nodeName The name of the node.
 * @param subTopicName The name of the topic to subscribe to.
 * @param pubTopicName The name of the topic to publish to.
 */
void microRosSetup(unsigned int timer_timeout, const char* nodeName, const char* subTopicName, const char* pubTopicName);

/**
 * Transmits the message over ROS. Not in use right now
 * @param /
 */
//void transmitMsg();

/**
 * Function to check the subscriptions.
 */
void checkSubs();

/**
 * Function to get the reference speed from the ROS topic.
 * @return The reference speed using the refSpeed struct.
 */
refSpeed getRefSpeed();

#ifdef ROS_DEBUG
void transmitDac(int16_t leftDacValue, int16_t rightDacValue);
#endif

#endif //MOTORMICROCONTROLLER_MICROROSFUNCTIONS_H
