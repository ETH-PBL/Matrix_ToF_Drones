/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#ifndef FLY_CONTROL // include guard
#define FLY_CONTROL

// #include <string.h>
#include <stdint.h>
#include <stdbool.h>
// #include "deck.h"
// #include "debug.h"
// #include <string.h>
// #include <math.h>

#include "ToF_process.h"
// #include "param.h"
// #include "config_params.h"



#define COMMAND_VALID_TIME 4000

#define END_LANDING_HEIGHT 0.07f
#define MAX_HEIGHT 1.2f
#define MIN_HEIGHT 0.2f
#define MAX_YAW_RATE 60.0f

#define SLOW_VZ 0.05f
#define MAX_VZ 0.5f

#define DELAY_TO_LAUNCH 500  //ms
#define DELAY_AFTER_LAUNCH 300  //ms

#define MAX_FLIGHT_DURATION 1000 //ms

void fly_task_take_off();
void fly_task_land();
bool fly_task(FlyCommand_t command);


#endif /* FLY_CONTROL */
