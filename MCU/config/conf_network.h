/**
 * @file conf_network_example.h
 * @author Jakob Friedl (friedl.jak@gmail.com)
 * @brief This file contains the network parameters for the micro-ROS
 * communication.
 * @version 0.1
 * @date 2023-07-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CONF_NETWORK_H
#define CONF_NETWORK_H
#include <Arduino.h>

/**
 * @brief Definition of the network parameters
 *
 */
const char* SSID = "manny";
const char* SSID_PW = "qwertyui";
const uint16_t AGENT_PORT = 8888; // AGENT port number
#define AGENT_IP 192, 168, 248, 96       // Change to IP of the ROS2 messages recceiver

#endif