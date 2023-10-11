/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

/* FreeRtos includes */
#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "crtp.h"
#include "vl53l5cx_api.h"
#include "deck.h"
#include "param.h"



#define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)
#define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)
#define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)

#define DEBUG_MODULE "HELLOWORLD"

//-------------------Control MAcros-------------------//
#define SESNSOR_FORAWARD_ENABLE

// #define SEND_DATA
#define ON_BOARD_PROCESS
#define START_FLIGHT

//-------------------Custom libraries-------------------//
#include "I2C_expander.h"
#include "ToF_process.h"
#include "Fly_control.h"


//-------------------Global Defines-------------------//
//Sensors Addresses
#define VL53L5CX_FORWARD_I2C_ADDRESS            ((uint16_t)(VL53L5CX_DEFAULT_I2C_ADDRESS*4))
#define VL53L5CX_BACKWARD_I2C_ADDRESS            ((uint16_t)(VL53L5CX_FORWARD_I2C_ADDRESS+2))
//Data Length
// #define ToF_DISTANCES_LEN             (128)
// #define ToF_TARGETS_DETECTED_LEN      (64)
// #define ToF_TARGETS_STATUS_LEN        (64)
#define FRONT_SENSOR_OFFSET        (0)
#define BACK_SENSOR_OFFSET        (ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN)



//-------------------Global Variables-------------------//
#ifdef SESNSOR_FORAWARD_ENABLE
static VL53L5CX_Configuration vl53l5dev_f;
static VL53L5CX_ResultsData vl53l5_res_f;
#endif


// IRQ
volatile uint8_t irq_status = 0;
//uint8_t vl53l5_buffer[VL53L5CX_MAX_RESULTS_SIZE];

uint32_t timestamp;

//-------------------Functions defins-------------------//
void send_command(uint8_t command, uint8_t arg);
void send_data_packet(uint8_t *data, uint16_t data_len);
void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index);
//
bool initialize_sensors_I2C(VL53L5CX_Configuration *p_dev, uint8_t mode);
bool config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address);
bool get_sensor_data(VL53L5CX_Configuration *p_dev,VL53L5CX_ResultsData *p_results);

//
#ifdef START_FLIGHT
uint16_t ToFly = 0;
#else
uint16_t ToFly = 1;
#endif

//MAIN
void appMain()
{ 
  vTaskDelay(M2T(3000)); //no rush to start

  //-----------------------------------Initilaize The Deck -----------------------------------------------------//
  bool gpio_exp_status = false;
  bool sensors_status = true;
  gpio_exp_status = I2C_expander_initialize();
  DEBUG_PRINT("ToFDeck I2C_GPIO Expander: %s\n", gpio_exp_status ? "OK." : "ERROR!");  
  #ifdef SESNSOR_FORAWARD_ENABLE
    vTaskDelay(M2T(100)); 
    sensors_status = initialize_sensors_I2C(&vl53l5dev_f,1); //forward
    DEBUG_PRINT("ToFDeck Forward Sensor Initlaize 1: %s\n", sensors_status ? "OK." : "ERROR!");
  #endif


  if(gpio_exp_status == false || sensors_status == false)
  {
    DEBUG_PRINT("ERROR LOOP_1!"); 
    while (1)
    {//stay in ERROR LOOP
      vTaskDelay(M2T(10000)); 
    }
  }

  DEBUG_PRINT("ToFDeck GPIO & Interrupt Initlaized. \n");  

  //-----------------------------------Take Drone Off -----------------------------------------------------//
  //wait for start command to fly
  while(ToFly == 0)
    vTaskDelay(200);
  #ifdef START_FLIGHT
    fly_task_take_off();
    // fly_task((FlyCommand_t){take_off,0.0f});
  #endif
  //-----------------------------------Satrt Sensors Ranging -----------------------------------------------------//
  #ifdef SESNSOR_FORAWARD_ENABLE
    vTaskDelay(M2T(100));
    uint64_t t0 = xTaskGetTickCount();
    uint8_t ranging_start_res_f = vl53l5cx_start_ranging(&vl53l5dev_f);
    uint64_t t1 = xTaskGetTickCount();
    DEBUG_PRINT("Time start ranging %d. \n", (uint32_t)(t1-t0));  


    DEBUG_PRINT("ToFDeck Start Sensor Forward Ranging: %s\n", (ranging_start_res_f == VL53L5CX_STATUS_OK) ? "OK." : "ERROR!"); 
  #else
    uint8_t ranging_start_res_f = VL53L5CX_STATUS_OK;
  #endif

  if(ranging_start_res_f != VL53L5CX_STATUS_OK){
    DEBUG_PRINT("ERROR LOOP_2!"); 
    while (1)
    {//stay in ERROR LOOP
      vTaskDelay(M2T(10000)); 
    }
  }

  //-----------------------------------Collect and Send Data------------------------------------------------------//

  #ifdef SESNSOR_FORAWARD_ENABLE
    uint8_t get_data_success_f = false;
    #ifdef SEND_DATA
      uint8_t to_send_buffer_f[ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN]; 
    #endif
  #endif
  
  while(1) {
    if (ToFly == 0){
      fly_task_land();
      break;
    }
    vTaskDelay(M2T(67)); // Task Delay, we want to run this task at 15Hz
    // Collect Data
    #ifdef SESNSOR_FORAWARD_ENABLE
      get_data_success_f = get_sensor_data(&vl53l5dev_f, &vl53l5_res_f);
      if (get_data_success_f == true)
      {
        
        #ifdef SEND_DATA
          //
          send_command(1, (ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN)/28 + 1);  
          //
          memcpy(&to_send_buffer_f[0], (uint8_t *)(&vl53l5_res_f.distance_mm[0]), ToF_DISTANCES_LEN);
          memcpy(&to_send_buffer_f[ToF_DISTANCES_LEN], (uint8_t *)(&vl53l5_res_f.nb_target_detected[0]), ToF_TARGETS_DETECTED_LEN);
          memcpy(&to_send_buffer_f[ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN], (uint8_t *)(&vl53l5_res_f.target_status[0]), ToF_TARGETS_STATUS_LEN);
          //
          send_data_packet(&to_send_buffer_f[0], ToF_DISTANCES_LEN+ToF_TARGETS_DETECTED_LEN+ToF_TARGETS_STATUS_LEN);
        #endif

        #ifdef ON_BOARD_PROCESS

          FlyCommand_t flight_command = Process_ToF_Image(&vl53l5_res_f);
          bool flight_status = fly_task(flight_command);

        #endif

        // Clear Flag
        get_data_success_f = false;

      }
    #endif
  }
}

void send_data_packet(uint8_t *data, uint16_t data_len)
{
  uint8_t packets_nr = 0;
  if (data_len%28 > 0)
    packets_nr = data_len/28 + 1;
  else
    packets_nr = data_len/28;

  for (uint8_t idx=0; idx<packets_nr; idx++)
    if(data_len - 28*idx >= 28)
      send_data_packet_28b(&data[28*idx], 28, idx);
    else
      send_data_packet_28b(&data[28*idx], data_len - 28*idx, idx);
}

void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index)
{
  CRTPPacket pk;
  pk.header = CRTP_HEADER(1, 0); // first arg is the port number
  pk.size = size + 2;
  pk.data[0] = 'D';
  pk.data[1] = index;
  memcpy(&(pk.data[2]), data, size);
  crtpSendPacketBlock(&pk);
}


void send_command(uint8_t command, uint8_t arg)
{
  //uint32_t timestamp = get_time_stamp();
  CRTPPacket pk;
  pk.header = CRTP_HEADER(1, 0); // first arg is the port number
  pk.size = 7;
  pk.data[0] = 'C';
  pk.data[1] = command;
  pk.data[2] = arg;
  memcpy(&pk.data[3], (uint8_t *)(&timestamp), 4);
  //DEBUG_PRINT("cmd PK 1:%d, 2:%d, 3:%d, 4:%d, ",pk.data[3],pk.data[4],pk.data[5],pk.data[6]); //todo delete
  crtpSendPacketBlock(&pk);
}

bool config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address)
{
  p_dev->platform = VL53L5CX_DEFAULT_I2C_ADDRESS; // use default adress for first use

  // initialize the sensor
  uint8_t tof_res = vl53l5cx_init(p_dev);   if (tof_res != VL53L5CX_STATUS_OK) return false ;
  //DEBUG_PRINT("ToF Config Result: %d \n", tof_init_res);

  // Configurations
  //change i2c address
  tof_res = vl53l5cx_set_i2c_address(p_dev, new_i2c_address);if (tof_res != VL53L5CX_STATUS_OK) return false ;
  tof_res = vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);if (tof_res != VL53L5CX_STATUS_OK) return false ; 
  // 15hz
  tof_res = vl53l5cx_set_ranging_frequency_hz(p_dev, 15);if (tof_res != VL53L5CX_STATUS_OK) return false ; 
  tof_res = vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);if (tof_res != VL53L5CX_STATUS_OK) return false ;
  tof_res = vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);if (tof_res != VL53L5CX_STATUS_OK) return false ;
  //tof_res = vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_AUTONOMOUS);if (tof_res != VL53L5CX_STATUS_OK) return false ;// TODO test it

  //Check for sensor to be alive
  uint8_t isAlive;
  tof_res =vl53l5cx_is_alive(p_dev,&isAlive);if (tof_res != VL53L5CX_STATUS_OK) return false;
  if (isAlive != 1) return false;
  
  // All Good!
  return true;
}

bool initialize_sensors_I2C(VL53L5CX_Configuration *p_dev, uint8_t mode)
{
  bool status = false;

  //reset I2C  //configure pins out/in for forward only

  //status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,I2C_RST_BACKWARD_PIN,I2C_RST_FORWARD_PIN);if (status == false)return status;

  if (mode == 1 && p_dev != NULL){
    //enable forward only and config
    status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,LPN_FORWARD_PIN | LED_FORWARD_PIN );if (status == false)return status; 
    status = config_sensors(p_dev,VL53L5CX_FORWARD_I2C_ADDRESS);if (status == false)return status; 
  }
  if (mode == 2 && p_dev != NULL){
    //enable backward only and config
    status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,LPN_BACKWARD_PIN | LED_BACKWARD_PIN); if (status == false)return status; 
    status = config_sensors(p_dev,VL53L5CX_BACKWARD_I2C_ADDRESS);if (status == false)return status; 
  }
  //status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,0x00); //all off
  if (mode == 3){
    //enable both forward & backward
    status = I2C_expander_set_register(OUTPUT_PORT_REG_ADDRESS,LPN_BACKWARD_PIN | LED_BACKWARD_PIN|LPN_FORWARD_PIN | LED_FORWARD_PIN); if (status == false)return status; 
  }
  return status;
}

bool get_sensor_data(VL53L5CX_Configuration *p_dev,VL53L5CX_ResultsData *p_results){
 
  // Check  for data ready I2c
  uint8_t ranging_ready = 2;
  //ranging_ready --> 0 if data is not ready, or 1 if a new data is ready.
  uint8_t status = vl53l5cx_check_data_ready(p_dev, &ranging_ready);if (status != VL53L5CX_STATUS_OK) return false;

  // 1 Get data in case it is ready
  if (ranging_ready == 1){
    status = vl53l5cx_get_ranging_data(p_dev, p_results);if (status != VL53L5CX_STATUS_OK) return false;
  }else {
    //0  data in not ready yet
    return false;
  }

  // All good then
  //return false;// TODO deleet
  return true;
}


PARAM_GROUP_START(ToF_FLY_PARAMS)
PARAM_ADD(PARAM_UINT16, ToFly, &ToFly)
PARAM_GROUP_STOP(ToF_FLY_PARAMS)