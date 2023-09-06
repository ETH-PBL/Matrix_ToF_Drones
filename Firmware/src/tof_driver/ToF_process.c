/* 
 * Copyright (C) 2023 ETH Zurich
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms
 * of the GPL-3.0 license.  See the LICENSE file for details.
 *
 * Authors: Hanna Müller, Vlad Niculescu, Tommaso Polonelli, Iman Ostovar
 */

#include "ToF_process.h"
#include "commander.h"
#include "param.h"
#include "log.h"
#include "math.h"

// zones and position parameters
zone_t DRONE_ZONE = {{3.0f, 3.0f}, {5.0f, 4.0f}};
zone_t CARE_ZONE = {{2.0f, 2.0f}, {5.0f, 5.0f}};
const pos_t MIDDLE_POS = {3.0f, 3.5f};


uint16_t DIS_REACT = 1400; // mm
uint16_t DIS_SLOW = 700; // mm
uint16_t DIS_STOP = 400; // mm
uint16_t DIS_FEAR = 150; // mm

float command_velocity_x = 0.0f;
float command_velocity_z = 0.0f;
float command_turn = 0.0f;
float min_distance = 0.0f;
uint16_t border_right = 0;
uint16_t border_top = 0;
uint16_t border_bottom = 0;
uint16_t border_left = 0;

#ifdef IMAGE_PROCESS_DEBUG_PRINT
#include "debug.h"
                                       { 0, 0, 0, 0, 1, 1, 1, 1} };
#endif
// C++ Program to count islands in boolean 2D matrix

// A function to check if a given cell (row, col) can be included in DFS
bool isSafe(bool M[][COL], uint8_t row, uint8_t col, bool visited[][COL])
{
    // row number is in range, column number is in range and value is 1 and not yet visited
    return (row >= 0) && (row < ROW) && (col >= 0) && (col < COL) && (M[row][col] && !visited[row][col]);
}

// A utility function to do DFS for a 2D boolean matrix. It only considers the 8 neighbours as adjacent vertices
void DFS(bool M[][COL], uint8_t row, uint8_t col, bool visited[][COL])
{
    // These arrays are used to get row and column numbers of 8 neighbours of a given cell
    const static uint8_t neighbour_num = 8;
    const static int8_t rowNbr[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
    const static int8_t colNbr[] = { -1, 0, 1, -1, 1, -1, 0, 1 };

    // Mark this cell as visited
    visited[row][col] = true;

    // Recur for all connected neighbours
    for (int k = 0; k < neighbour_num; ++k)
        if (isSafe(M, row   + rowNbr[k], col + colNbr[k], visited))
            DFS(M, row + rowNbr[k], col + colNbr[k], visited);
}

// The main function that returns count of islands in a given boolean 2D matrix
uint8_t countIslands(bool M[][COL], bool objects[][ROW][COL])
{
    // Make a bool array to mark visited cells.
    // Initially all cells are unvisited
    bool visited[ROW][COL];
    bool old_visited[ROW][COL];

    memset(visited, 0, ROW*COL); //sizeof(visited)

    int count = 0;
    for (int i = 0; i < ROW; ++i)
        for (int j = 0; j < COL; ++j)
            if (M[i][j] && !visited[i][j]) {
                //save old visited status
                memcpy(old_visited,visited, sizeof(visited));

                // If a cell with value 1 is not visited yet, then new island found Visit all cells in this island.
                DFS(M, i, j, visited);

                //save new Island positions
                if (count < MAX_TARGET_NUM)
                {   
                    for (int x = 0; x<ROW; ++x)
                        for (int y = 0; y<COL; ++y)
                            objects[count][x][y] = old_visited[x][y] ^ visited[x][y];
                }

                // and increment island count
                ++count;
            }

    return count;
}


FlyCommand_t Process_ToF_Image(VL53L5CX_ResultsData* p_tof_results)
{
    // decode results
    uint16_t ToF_distances[ToF_DISTANCES_LEN/2]; 
    uint8_t ToF_targets[ToF_TARGETS_DETECTED_LEN]; 
    uint8_t ToF_status[ToF_TARGETS_STATUS_LEN]; 
    memcpy(ToF_distances, (uint8_t *)(&p_tof_results->distance_mm[0]), ToF_DISTANCES_LEN);
    memcpy(ToF_targets, (uint8_t *)(&p_tof_results->nb_target_detected[0]), ToF_TARGETS_DETECTED_LEN);
    memcpy(ToF_status, (uint8_t *)(&p_tof_results->target_status[0]), ToF_TARGETS_STATUS_LEN);
    
    // found invalid pixels and binarize the 
    bool invalid_mask[ROW*COL];
    for (int i= 0; i<ROW*COL; ++i)
        invalid_mask[i] = (ToF_status[i] != 5 && ToF_status[i] != 9) || ToF_targets[i] != 1;
    
    
    // binarize the image
    bool binary_matrix[ROW][COL];
    uint16_t ToF_distances_matrix[ROW][COL]; 

    for (int i= 0; i<ROW; ++i)
        for (int j= 0; j<COL; ++j)
        {
            if(! invalid_mask[j+COL*i]) // check if the pixel is valid
            {
                ToF_distances_matrix[i][j] = ToF_distances[j+COL*i];
                binary_matrix[i][j] = (ToF_distances_matrix[i][j] <= MAX_DISTANCE_TO_PROCESS);
            }
            else
            {
                ToF_distances_matrix[i][j] = UINT16_MAX; //this is invalid Value;  should care about not to be used later
                binary_matrix[i][j] = false;
            }
        }


    // detect objects
    static bool objects_matrixes[MAX_TARGET_NUM][ROW][COL];
    uint8_t object_num = countIslands(binary_matrix, objects_matrixes);

    // objects feature extraction
    target_t targets[MAX_TARGET_NUM] ;
    for (int k=0; k<object_num; ++k)
    {
        uint16_t dis_min = UINT16_MAX;
        uint16_t x_sum = 0;
        uint16_t y_sum = 0;
        uint8_t ones_count = 0;
        borders_t tar_borders = {INT8_MAX, INT8_MAX, INT8_MIN, INT8_MIN}; //top left right bottom

        for (int i= 0; i<ROW; ++i)
        {
            for (int j= 0; j<COL; ++j)
            {
                if (objects_matrixes[k][i][j])
                {
                    if(ToF_distances_matrix[i][j] < dis_min)
                        dis_min = ToF_distances_matrix[i][j];

                    x_sum += i;
                    y_sum += j;
                    ones_count ++;

                    //update borders 
                    if (i < tar_borders.top) //min i
                        tar_borders.top = i;
                    if (i > tar_borders.bottom) // max i
                        tar_borders.bottom = i;
                    if (j < tar_borders.left) // min j
                        tar_borders.left = j;
                    if (j > tar_borders.right) // max j
                        tar_borders.right = j;
                }
            }
        }
        targets[k].min_distance = dis_min;
        targets[k].position.x = (float)x_sum/ones_count;
        targets[k].position.y = (float)y_sum/ones_count;
        targets[k].pixels_number = ones_count;
        targets[k].borders = tar_borders; 
    }
    
    #ifdef IMAGE_PROCESS_DEBUG_PRINT
        // DEBUG_PRINT("ToFDeck P.I. N.Targets: %d\n",object_num); 
    #endif
    return Decision_Making(targets, object_num);

}


FlyCommand_t Decision_Making(target_t* targets, uint8_t object_num)
{  
    // find the highest priority target 
    uint16_t dis_global_min = UINT16_MAX;
    int8_t selected_target = -1;
    for (int k= 0; k<object_num; ++k)
    {
        if (targets[k].min_distance < dis_global_min && targets[k].pixels_number > MIN_PIXEL_NUMBER )
        {
            dis_global_min = targets[k].min_distance;
            selected_target = k;
        }
    }

    if (selected_target < 0) // no big target in front
        return (FlyCommand_t){1.0f, 0.0f, 0.0f};
    //TODO

    command_velocity_x = 0.0f;
    command_velocity_z = 0.0f;
    command_turn = 0.0f;
    //float current_pitch = logGetFloat(logGetVarId("stateEstimate", "pitch"));
    // float current_vx = logGetFloat(logGetVarId("stateEstimate", "vx"));
    //targets[selected_target].min_distance *= cos(current_pitch/180*3.14f);
    min_distance = (float)(targets[selected_target].min_distance/1000.0f); // just for logging
    border_right = targets[selected_target].borders.right;
    border_bottom = targets[selected_target].borders.bottom;
    border_left = targets[selected_target].borders.left;
    border_top = targets[selected_target].borders.top;

    if (targets[selected_target].min_distance <= DIS_FEAR)
    {
        command_velocity_x = VEL_FEAR;
        command_turn = TURN_NOT;
    }
    else if (targets[selected_target].borders.top >= GROUND_BORDER &&   
        targets[selected_target].min_distance < DIS_GROUND_MIN && 
        ( targets[selected_target].position.x >= MIDDLE_POS.x  )
        )//check for ground lower
    {
        //ground should be avoided
        // decision--> "/increase height"
        // target-->  '-G'
        command_velocity_x = VEL_STOP;
        command_velocity_z = VEL_UP;
    }
    else if (targets[selected_target].borders.bottom <= CELLING_BORDER &&
        targets[selected_target].min_distance < DIS_CEILING_MIN &&
        (targets[selected_target].position.x < MIDDLE_POS.x  )
        ) //check for celling upper
    {
        // celling should be avoided
        // decision--> "/Decrease height"
        // target-->  '-C'
        command_velocity_x = VEL_STOP;
        command_velocity_z = VEL_DOWN;
    }
    else if (targets[selected_target].min_distance < DIS_REACT) //check for front object
    {
        // scale the distance to the object to be in between 0 and 1 (negative values are zeroed later on)
        command_velocity_x = (float)(targets[selected_target].min_distance - DIS_STOP)/(float)(DIS_REACT - DIS_STOP);


        if (targets[selected_target].borders.right >= DRONE_ZONE.top_left.y  &&
            targets[selected_target].borders.bottom >= DRONE_ZONE.top_left.x &&
            targets[selected_target].borders.left <= DRONE_ZONE.bottom_righ.y &&
            targets[selected_target].borders.top <= DRONE_ZONE.bottom_righ.x ) // object is in drone zone
        {
            if (targets[selected_target].min_distance <= DIS_STOP)
            {
                command_velocity_x = VEL_STOP;
                command_turn = TURN_MAX;
            }
            else if (targets[selected_target].min_distance <= DIS_SLOW ) 
            {
                command_velocity_x *= VEL_SCALE_SLOW;
                command_turn = TURN_MAX;
            }
            else
            {
                command_velocity_x = ((float)(targets[selected_target].min_distance - DIS_SLOW)/(float)(DIS_REACT - DIS_STOP))*VEL_SCALE_MEDIUM + ((float)(DIS_SLOW - DIS_STOP)/(float)(DIS_REACT - DIS_STOP))*VEL_SCALE_SLOW;
                // make sure there are no "bumps" in velocity scaling
                // float tmp_slow_factor = (float)(DIS_REACT - targets[selected_target].min_distance)/(float)(DIS_REACT - DIS_STOP);
                // command_velocity_x -= (VEL_SCALE_MEDIUM - VEL_SCALE_SLOW)*tmp_slow_factor;
                command_turn = TURN_SLOW;
            }
        }
        else if (targets[selected_target].borders.right >= CARE_ZONE.top_left.y  &&
                 targets[selected_target].borders.bottom >= CARE_ZONE.top_left.x &&
                 targets[selected_target].borders.left <= CARE_ZONE.bottom_righ.y &&
                 targets[selected_target].borders.top <= CARE_ZONE.bottom_righ.x) // object is in care zone
        {
            if (targets[selected_target].min_distance <= DIS_STOP)
            {
                command_velocity_x = VEL_STOP;
                command_turn = TURN_MAX;
            }
            else
            {
                command_velocity_x *= VEL_SCALE_MEDIUM;
                command_turn = TURN_SLOW;
            }
        }
        else
        {
            command_velocity_x = VEL_SCALE_MEDIUM; // the objects are all in the outer regions of the FoV, we should not reduce the speed based on the distance to them
            command_turn = TURN_NOT;
        }
        if (targets[selected_target].position.y < MIDDLE_POS.y)
        {
            command_turn *= -1.0f;
        }

         
    }
    else
    {
        command_velocity_x = VEL_SCALE_MEDIUM;
        command_turn = TURN_NOT;
    }  


    // Check for special situations
    command_turn = Handle_Exception_Commands(command_turn);
    //
 

    FlyCommand_t fly_command = {command_velocity_x, command_velocity_z, command_turn};
    return fly_command;
}

float Handle_Exception_Commands(float current_turn_command)
{
    float refine_command = current_turn_command;
    #define HISTORY_LENGTH 5
    static float previous_command[HISTORY_LENGTH] = {CommandError} ;
    static uint8_t previous_command_index = 0;
    uint8_t left_commands = 0, right_commands=0;
    
    //hanlde convex situation
    for (int i =0; i< HISTORY_LENGTH; i++)
    {
        if (previous_command[i] < -TURN_MAX + EPSILON)
            right_commands++;
        else if (previous_command[i] > TURN_MAX - EPSILON)
            left_commands++; 
    }
    if ((current_turn_command < -TURN_MAX + EPSILON || current_turn_command > TURN_MAX - EPSILON) 
        && right_commands+left_commands >= (MAX_TURN_RATIO* HISTORY_LENGTH) ) 
    {
            // negativ means turn to the right 
            // (we always want to turn right if we are stuck, to avoid switching between right/left and to maximize explored area)
            if (right_commands > left_commands)
            {
                refine_command = - TURN_FAST; 
            }else{
                refine_command = TURN_FAST;
            }
    }
    previous_command[previous_command_index++] = current_turn_command;
    previous_command_index %= HISTORY_LENGTH;
    //
    return refine_command;
}

PARAM_GROUP_START(ToF_FLY_PARAMS)
PARAM_ADD(PARAM_UINT16, React_dis, &DIS_REACT)
PARAM_ADD(PARAM_UINT16, SLOW_DIS, &DIS_SLOW)
PARAM_ADD(PARAM_UINT16, Stop_dis, &DIS_STOP)
PARAM_ADD(PARAM_UINT16, Fear_dis, &DIS_FEAR)
PARAM_GROUP_STOP(ToF_FLY_PARAMS)

LOG_GROUP_START(tof_cmds)
LOG_ADD(LOG_FLOAT, vel_x, &command_velocity_x)
LOG_ADD(LOG_FLOAT, vel_z, &command_velocity_z)
LOG_ADD(LOG_FLOAT, turn, &command_turn)
LOG_ADD(LOG_FLOAT, min_dis, &min_distance)
LOG_ADD(LOG_UINT16, bor_t, &border_top)
LOG_ADD(LOG_UINT16, bor_l, &border_left)
LOG_ADD(LOG_UINT16, bor_r, &border_right)
LOG_ADD(LOG_UINT16, bor_b, &border_bottom)
LOG_GROUP_STOP(tof_cmds)