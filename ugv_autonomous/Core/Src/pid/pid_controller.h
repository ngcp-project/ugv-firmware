/**
 * @file pid.h
 * @author George Yu
 * @brief This file contains all function prototypes for the pid.c driver.
 * 
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "main.h"

/************************************* INCLUDE FILES ******************************************/
/*************************************** CONSTANTS ********************************************/
/************************************ TYPE DEFINITIONS ****************************************/


/*************************************** FUNCTIONS ********************************************/

/**
 * @brief Function to initialize a ugv_pid instance.
 */
float PID_controller_1(float set_value, float process_value, float Kp, float Ki, float Kd, float limit);

#endif 
