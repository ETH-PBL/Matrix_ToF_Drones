/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#ifndef TOF_PROCESS // include guard
#define TOF_PROCESS

// #include <string.h>
#include <stdint.h>
// #include <bits/stdc++.h>
#include <stdbool.h>
#include "vl53l5cx_api.h"
// #include "morph.h"

//Parametrs
#define GROUND_BORDER 6
#define CELLING_BORDER 2
#define MIN_PIXEL_NUMBER 1
// Decision Parameters
#define DIS_GROUND_MIN 400 // mm
#define DIS_CEILING_MIN 600 // mm

// #define NO_TURN // For braking tests

#ifdef NO_TURN
	#define TURN_MAX 0.0f
	#define TURN_SLOW 0.0f
	#define TURN_FAST 0.0f
#else
	#define TURN_MAX 1.0f
	#define TURN_SLOW 0.7f
	#define TURN_FAST 3.0f
#endif
#define TURN_NOT 0.0f
#define VEL_STOP 0.0f
#define VEL_FEAR -0.2f
#define VEL_SCALE_MEDIUM 1.0f
#define VEL_SCALE_SLOW 0.7f
#define VEL_UP 0.2f
#define VEL_DOWN -0.5f
#define MAX_TURN_RATIO 0.8f
#define EPSILON 0.0001f

// #define IMAGE_PROCESS_DEBUG_PRINT


// Constants
#define ROW 8
#define COL 8
#define MAX_TARGET_NUM 6
//ToF
#define ToF_DISTANCES_LEN             (2*ROW*COL)
#define ToF_TARGETS_DETECTED_LEN      (ROW*COL)
#define ToF_TARGETS_STATUS_LEN        (ROW*COL)
#define TOF_FPS        (15.0f)
#define TOF_PERIOD        (1.0f/TOF_FPS)

// Process Parameter
#define MAX_DISTANCE_TO_PROCESS       (2000)  //mm


//structers

typedef struct pos {
	float x;
	float y;
}pos_t;


typedef struct zone {
	pos_t top_left;
	pos_t bottom_righ;
}zone_t;

typedef struct borders {
	int8_t top;
	int8_t left;
	int8_t right;
	int8_t bottom;
}borders_t;

typedef struct target {
	pos_t position;
    borders_t borders;
	uint16_t min_distance;
	uint16_t max_distance;
	uint16_t avg_distance;
    uint8_t pixels_number;
}target_t;

enum Region{RegionError = -1,
		    UpperLeft=0,
			UpperRight, 
			LowerLeft, 
			LowerRight};

enum MoveCommand{
				 CommandError=-1,
				 Stop = 0,
				 Fast_Right,
    			 Right,
				 care_forward_and_slow_right,
				 slow_forward_and_right,
				 Fast_Left,
				 left,
				 care_forward_and_slow_left,
				 slow_forward_and_left,
				 increase_altitude,
				 decrease_altitude,
				 care_forward,
				 slow_forward,
				 forward,
				 take_off,
				 land
				 };

typedef struct _FlyCommand {
	float command_velocity_x;
	float command_velocity_z;
	float command_turn; 
}FlyCommand_t;


bool isSafe(bool M[][COL], uint8_t row, uint8_t col, bool visited[][COL]);
void DFS(bool M[][COL], uint8_t row, uint8_t col, bool visited[][COL]);
uint8_t countIslands(bool M[][COL], bool objects[][ROW][COL]);
FlyCommand_t Process_ToF_Image(VL53L5CX_ResultsData* p_tof_results);
FlyCommand_t Decision_Making(target_t* targets, uint8_t object_num);
float Handle_Exception_Commands(float current_command);
// int ToF_process_test(void);


#endif /* TOF_PROCESS */