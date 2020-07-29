/*
    6_pack_thc.h

    Covers all V1 versions V1p0, V1p1, etc

    Part of Grbl_ESP32
    Pin assignments for the ESP32 I2S 6-axis board
    2018    - Bart Dring
    2020    - Mitch Bradley
    2020    - Michiyasu Odaki
	2020    - William Curry 
    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    Grbl is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with Grbl_ESP32.  If not, see <http://www.gnu.org/licenses/>.
	This 6-Pack Torch Height Control Custom File Was Created By: William Curry
*/
#define MACHINE_NAME            "6 Pack Controller with Plasma Torch Height Control"
#define CUSTOM_CODE_FILENAME "Custom/torchHeightControl.cpp"//define the torch height control custom code file
#ifdef N_AXIS
        #undef N_AXIS
#endif
#define N_AXIS 3
//Enable the SD Card
#ifndef ENABLE_SD_CARD
    #define ENABLE_SD_CARD
#endif

#define DEFAULT_THC_DEBUG 1 // Boolean //If true this will print torch height debug information out
#define DEFAULT_THC_TARGET_VOLTAGE 0 // Volts // default target voltage (a real value would be 100 volts)
#define DEFAULT_THC_DEBUG_PRINT_MILLIS 1000 //milliseconds // debug print time in milliseconds
#define DEFAULT_THC_ARC_DELAY_TIME 250 //milliseconds // Time for Arc to Start before running THC
#define DEFAULT_THC_VOLTAGE_FILTER_VALUE 0.5 //ND// Torch Voltage Filter Time Constant
#define DEFAULT_THC_ITER_FREQ 20 //milliseconds// Torch Height Control Time Between Calls, this will directly effect ...
// ... the rate at which the Z axis moves, the machine must be restarted after changing this for it to take effect

// === Special Features

// I2S (steppers & other output-only pins)
#define USE_I2S_OUT
// Define USE_I2S_OUT_STREAM if buffering is used.
// (there will be a delay between the specified I/O operation and the actual I/O execution)
#define USE_I2S_OUT_STREAM
#undef USE_RMT_STEPS
#define STEP_PULSE_DELAY 10 //Mircoseconds to delay between setting direction pin and setting step pin high
//#define USE_STEPSTICK   //Bill comment makes sure MS1,2,3 !reset and !sleep are set

#define I2S_OUT_BCK      GPIO_NUM_22
#define I2S_OUT_WS       GPIO_NUM_17
#define I2S_OUT_DATA     GPIO_NUM_21


#define STEPPER_X_MS3           I2SO(3)   // X_CS
#define STEPPER_Y_MS3           I2SO(6)   // Y_CS
#define STEPPER_Z_MS3           I2SO(11)  // Z_CS
//#define STEPPER_A_MS3           I2SO(14)  // A_CS
//#define STEPPER_B_MS3           I2SO(19)  // B_CS
//#define STEPPER_C_MS3           I2SO(22)  // C_CS

#define STEPPER_RESET           GPIO_NUM_19

#define X_DISABLE_PIN           I2SO(0)
#define X_DIRECTION_PIN         I2SO(1)
#define X_STEP_PIN              I2SO(2)

#define Y_DIRECTION_PIN         I2SO(4)
#define Y_STEP_PIN              I2SO(5)
#define Y_DISABLE_PIN           I2SO(7)

#define Z_DISABLE_PIN           I2SO(8)
#define Z_DIRECTION_PIN         I2SO(9)
#define Z_STEP_PIN              I2SO(10)
/* Bill Comment out these axis'
#define A_DIRECTION_PIN         I2SO(12)
#define A_STEP_PIN              I2SO(13)
#define A_DISABLE_PIN           I2SO(15)

#define B_DISABLE_PIN           I2SO(16)
#define B_DIRECTION_PIN         I2SO(17)
#define B_STEP_PIN              I2SO(18)

#define C_DIRECTION_PIN         I2SO(20)
#define C_STEP_PIN              I2SO(21)
#define C_DISABLE_PIN           I2SO(23)
*/
#define X_LIMIT_PIN             GPIO_NUM_33
#define Y_LIMIT_PIN             GPIO_NUM_32
#define Z_LIMIT_PIN             GPIO_NUM_35
//#define A_LIMIT_PIN             GPIO_NUM_34
//#define B_LIMIT_PIN             GPIO_NUM_39
//#define C_LIMIT_PIN             GPIO_NUM_36

#define PROBE_PIN               GPIO_NUM_25
#define THC_VOLTAGE_PIN         GPIO_NUM_5
///Resistance Values needed to determine arc voltage i.e. Vout = (Vs*R2)/(R1+R2)
#define VOLTAGE_DIVIDER_R1      470 ///470K Ohms
#define VOLTAGE_DIVIDER_R2      7.6 ///7.6K Ohms


// 0-10v CNC Module in Socket #3
// Control...Set PD001 to 1 if enable is connected 0 is panel should be used
// Freq...Set PD002 to 0
// Freq input...Set PD070 to 0 for 0-10V
/*
#define SPINDLE_TYPE            SPINDLE_TYPE_10V
#define SPINDLE_OUTPUT_PIN      GPIO_NUM_26
#define SPINDLE_FORWARD_PIN     GPIO_NUM_4
#define SPINDLE_REVERSE_PIN     GPIO_NUM_16
*/


// Example 5V output CNC module in socket #3
/*
#define SPINDLE_TYPE            SPINDLE_TYPE_PWM
#define SPINDLE_OUTPUT_PIN      GPIO_NUM_26
#define SPINDLE_ENABLE_PIN      GPIO_NUM_4
#define SPINDLE_DIR_PIN         GPIO_NUM_16
#define COOLANT_MIST_PIN        GPIO_NUM_27
*/


// Example (4x) 5V Buffer Output on socket #5
#define SPINDLE_TYPE            SPINDLE_TYPE_RELAY
#define SPINDLE_OUTPUT_PIN      I2SO(24)
#define SPINDLE_DIR_PIN         I2SO(25)
#define COOLANT_MIST_PIN        I2SO(26)
#define COOLANT_FLOOD_PIN       I2SO(27)


/*
// RS485 In socket #3
#define SPINDLE_TYPE            SPINDLE_TYPE_HUANYANG // only one spindle at a time
#define HUANYANG_TXD_PIN        GPIO_NUM_26
#define HUANYANG_RTS_PIN        GPIO_NUM_4
#define HUANYANG_RXD_PIN        GPIO_NUM_16
*/
void THCSyncTask(void* pvParameters); //Task called for Torch height controlled thats defined in torchHeightControl.cpp
void THCVoltageTask(void* pvParameters); //Task called for Torch height controlled thats defined in torchHeightControl.cpp
// === Default settings
#define DEFAULT_STEP_PULSE_MICROSECONDS I2S_OUT_USEC_PER_PULSE
