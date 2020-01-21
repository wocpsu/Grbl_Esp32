/*
  dxl_delta.h - Machine definition of the delta using 
  3 dynamixel XL430-250 servos

  Copyright (c) 2019 Barton Dring @buildlog
      
	 
  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

// Machine Geometery

#define KIN_ANGLE_CALC_OK   0
#define KIN_ANGLE_ERROR     -1

#define DXL_TIMER_INT_FREQ 30 // milliseconds between syncs

// Dynamixel motor info
#define DXL_MOTOR_0_ID      1       // protocol ID
#define DXL_MOTOR_0_FWD     true    // does this motor spin forward?
#define DXL_MOTOR_0_CENTER  (2048 - 33)    // what is the center (horizontal) index 

#define DXL_MOTOR_1_ID      2
#define DXL_MOTOR_1_FWD     true
#define DXL_MOTOR_1_CENTER  (2048 - 33)

#define DXL_MOTOR_2_ID      3
#define DXL_MOTOR_2_FWD     true
#define DXL_MOTOR_2_CENTER  (2048 - 33)

#define DXL_COUNT_PER_DEG   4095.0f / 360.0f

#define LAST_STEPPER_ENABLE_STATE_UNKOWN  0
#define LAST_STEPPER_ENABLE_STATE_TRUE    1
#define LAST_STEPPER_ENABLE_STATE_FALSE   2

#define SEGMENT_LENGTH 0.5 // segment length in mm
#define USE_M30

// ============= Begin CPU MAP ================
#define CPU_MAP_NAME "CPU_MAP_DXL_DELTA"
#define LIMIT_MASK 0  // no limit pins	

#define USE_MACHINE_INIT
#define USE_CUSTOM_HOMING
#define USE_DYNAMIXEL

// defaults
#define DEFAULT_STEP_PULSE_MICROSECONDS 3 
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 200 // 200ms

#define DEFAULT_STATUS_REPORT_MASK 1

#define DEFAULT_STEPPING_INVERT_MASK 0 // uint8_t
#define DEFAULT_DIRECTION_INVERT_MASK 0 // uint8_t
#define DEFAULT_INVERT_ST_ENABLE 0 // boolean

#define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
#define DEFAULT_HARD_LIMIT_ENABLE 0  // false

#define DEFAULT_HOMING_ENABLE 0
#define DEFAULT_HOMING_DIR_MASK 0

#define DEFAULT_X_STEPS_PER_MM 80 // value is actually arbitrary, but keep it smallish
#define DEFAULT_Y_STEPS_PER_MM 80
#define DEFAULT_Z_STEPS_PER_MM 80

// keep axis xyz axis values the same.
#define DEFAULT_X_MAX_RATE 40000.0 // mm/min
#define DEFAULT_Y_MAX_RATE 40000.0 // mm/min
#define DEFAULT_Z_MAX_RATE 40000.0 // mm/min

#define DEFAULT_X_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Y_ACCELERATION (200.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Z_ACCELERATION (200.0*60*60) 

// not used right now
#define DEFAULT_X_MAX_TRAVEL 200 // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 200 // mm NOTE: Must be a positive value.
#define DEFAULT_Z_MAX_TRAVEL 200 // mm NOTE: Must be a positive value.



#ifndef dxl_delta_h
    #define dxl_delta_h

    #include "grbl.h"    
    #include "grbl_dynamixel.h"

    void machine_init();
    void dynamixelSyncTask(void *pvParameters);
    void dxl_sync_position();
    void dxl_read_position();
    //void inverse_kinematics(float *target, plan_line_data_t *pl_data, float *position);
   // void forward_kinematics(float *position);
    int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3);
    int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0);
    int delta_calcAngleYZ(float x0, float y0, float z0, float &theta);

    uint16_t DXL_value(float degrees, uint16_t centalVal, bool forward);
    float DXL_value_degrees(int32_t value, int16_t centerVal, bool forward);

    void dxl_delta_torque_enable(bool enable);

    //bool kinematics_pre_homing(uint8_t cycle_mask);
    //void kinematics_post_homing();
    bool user_defined_homing();
    void user_m30();
    
    
#endif