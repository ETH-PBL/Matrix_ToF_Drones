/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#include "system.h"
#include "FreeRTOS.h"
#include "task.h"
#include "motors.h"
#include "estimator_kalman.h"
#include "commander.h"
#include "deck.h"
#include "Fly_control.h"
#include <math.h>

#include "debug.h"
#include "log.h"
#include "param.h"
#include "pm.h"



uint32_t last_time = 0;
// static uint32_t delta_time = 0;
//flight parameters
volatile float current_height = 0.0;
//param
float MAX_VX = 1.0f;
float MAX_AX = 1.0f;
float MAX_NEG_AX = 10.0f;
float START_TAKE_OFF_HEIGHT= 0.6f;
static bool isFlying = false;
//
extern uint16_t ToFly;

static void TakeOff(float height);
static void Land(void);
static void headToSetpoint(float x, float y, float z, float yaw);
static setpoint_t CreateSetpoint(float x_vel, float y_vel, float z, float yaw_rate);
static void setHoverSetPoint(float x_vel, float y_vel, float z_vel, float yaw_rate);
static void positionSet(setpoint_t *setpoint, float x, float y, float z, float yaw);
static uint32_t get_time_stamp();

void fly_task_land()
{
    Land(); 
    isFlying = false;
}

void fly_task_take_off()
{
    isFlying = true;
    // Reset estimator
    estimatorKalmanInit();
    // Take off
    TakeOff(START_TAKE_OFF_HEIGHT);
}

bool fly_task(FlyCommand_t fly_command)
{

    //update time
    if (last_time == 0)
    {
        // delta_time = 0;
        last_time = get_time_stamp();
    }
    uint32_t delta =  get_time_stamp();
    if(
        (delta - last_time >= ToFly * MAX_FLIGHT_DURATION && isFlying == true) ||
        pmIsBatteryLow()
      )
    {
        Land();
        // DEBUG_PRINT("Landed\n"); 
        isFlying = false;
        return isFlying;
    }

    if (isFlying == false || pmIsChargerConnected())
    {
        // commanderNotifySetpointsStop(50);
        return isFlying;
    }

    float Vx = 0; 
    float Vy = 0;
    float Vz = 0;
    float yaw_rate = 0 ;

    Vx = fly_command.command_velocity_x;
    if (fly_command.command_velocity_x > 0.99f)
    {
        Vx = MAX_VX;
    }
    Vz = fly_command.command_velocity_z *  MAX_VZ;
    yaw_rate =  fly_command.command_turn * MAX_YAW_RATE;
   
    setHoverSetPoint(Vx, Vy, Vz, yaw_rate);

    return isFlying;
}


static void TakeOff(float height) {
    vTaskDelay(DELAY_TO_LAUNCH);
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    uint32_t endheight = (uint32_t)(100 * (height - 0.3f));
    for(uint32_t i=0; i<endheight; i++) {
        headToSetpoint(pos.x, pos.y, 0.3f + (float)i / 100.0f, 0);
        vTaskDelay(30);
    }

    current_height = height;
    vTaskDelay(DELAY_AFTER_LAUNCH); // wait for being stablize
}

static void Land(void) {
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    float height = pos.z;
    float current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));

    for(int i=(int)100*height; i>100*END_LANDING_HEIGHT; i--) {
        // setHoverAltitude((float)i / 100.0f);
        headToSetpoint(pos.x, pos.y, (float)i / 100.0f, current_yaw);
        vTaskDelay(40);
    }
    current_height = 0;
    vTaskDelay(300);
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    vTaskDelay(200);
}


static void headToSetpoint(float x, float y, float z, float yaw)
{
    setpoint_t setpoint;
    positionSet(&setpoint, x, y, z, yaw);
    commanderSetSetpoint(&setpoint, 3);
}


static void setHoverSetPoint(float x_vel, float y_vel, float z_vel, float yaw_rate)
{
    current_height += z_vel * (75.0f/1000.0f);

    if((float)fabs(z_vel - 0.0f) < 0.01f && (float)fabs(current_height - START_TAKE_OFF_HEIGHT) > 0.05f )
    {
        if (current_height > START_TAKE_OFF_HEIGHT)
            current_height -= SLOW_VZ * TOF_PERIOD;
        else
            current_height += SLOW_VZ * TOF_PERIOD;
    }

    //control values and send 
    if (current_height < MIN_HEIGHT)
        current_height = MIN_HEIGHT;
    else if (current_height > MAX_HEIGHT)
        current_height= MAX_HEIGHT;
    
    // check for accelerations
    static float previous_vx = 0;

    if ((x_vel > previous_vx) && ((x_vel - previous_vx) > (MAX_AX*TOF_PERIOD)))
    {
        x_vel = previous_vx + (MAX_AX*TOF_PERIOD);
    }
    if ((x_vel < previous_vx) && ((previous_vx - x_vel) > (MAX_NEG_AX*TOF_PERIOD)))
    {
        x_vel = previous_vx - (MAX_NEG_AX*TOF_PERIOD);
    }
    previous_vx = x_vel ;


    setpoint_t setpoint = CreateSetpoint(x_vel, y_vel, current_height, yaw_rate);
    commanderSetSetpoint(&setpoint, 3);

}


static void positionSet(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
    memset(setpoint, 0, sizeof(setpoint_t));

    setpoint->mode.x = modeAbs;
    setpoint->mode.y = modeAbs;
    setpoint->mode.z = modeAbs;

    setpoint->position.x = x;
    setpoint->position.y = y;
    setpoint->position.z = z;

    setpoint->mode.yaw = modeAbs;

    setpoint->attitude.yaw = yaw;

    setpoint->mode.roll = modeDisable;
    setpoint->mode.pitch = modeDisable;
    setpoint->mode.quat = modeDisable;
}

static setpoint_t CreateSetpoint(float x_vel, float y_vel, float z, float yaw_rate)
{
	setpoint_t setpoint;	
	memset(&setpoint, 0, sizeof(setpoint_t));
	setpoint.mode.x = modeVelocity;
	setpoint.mode.y = modeVelocity;
	setpoint.mode.z = modeAbs;
	setpoint.mode.yaw = modeVelocity;

	setpoint.velocity.x	= x_vel;
	setpoint.velocity.y	= y_vel;
	setpoint.position.z = z;
	setpoint.attitudeRate.yaw = yaw_rate;
	setpoint.velocity_body = true;
	return setpoint;
}

static uint32_t get_time_stamp()
{
  uint32_t timestamp = ((long long)xTaskGetTickCount())/portTICK_RATE_MS;
  //DEBUG_PRINT("ToFDeck timestamp: %d (size: %d)\n", timestamp,sizeof(timestamp));
  return timestamp;
}




PARAM_GROUP_START(ToF_FLY_PARAMS)
PARAM_ADD(PARAM_FLOAT, MAX_VX, &MAX_VX)
PARAM_ADD(PARAM_FLOAT, MAX_AX, &MAX_AX)
PARAM_ADD(PARAM_FLOAT, MAX_NEG_AX, &MAX_NEG_AX)
PARAM_ADD(PARAM_FLOAT, defaultH, &START_TAKE_OFF_HEIGHT)
PARAM_GROUP_STOP(ToF_FLY_PARAMS)