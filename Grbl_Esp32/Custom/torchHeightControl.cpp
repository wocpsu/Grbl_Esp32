/*
	torchHeightControl.cpp
	Part of Grbl_ESP32

	copyright (c) 2018 -	Bart Dring This file was modified for use on the ESP32
	CPU. Do not use this with Grbl for atMega328P

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

	--------------------------------------------------------------

 	This contains all the special features required for "Plasma Torch Height Control"
*/
// This file is enabled by defining CUSTOM_CODE_FILENAME "torchHeightControl.cpp"
// in Machines/torchHeightControl.h, thus causing this file to be included
// from ../custom_code.cpp
extern uint32_t stepZTHC;
static TaskHandle_t THCSyncTaskHandle = 0;
unsigned long THCCounter = 0;
bool debugTHC = true;
unsigned long lastDebugPrintTimeMillis;
unsigned long arcOnTime;
unsigned long arcDelayTime = 250; //How long to wait before starting THC routine and voltage filtering in milliseconds
float torchFilterValue = 0.75;
float torchVoltageFiltered;

void thcStepZDown(){
		if (thc_debug_setting->get() && ((millis() - lastDebugPrintTimeMillis) > thc_debugprint_millis->get()) )
		{
            grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "THC Voltage = %d Moving Z Down\n", torchVoltageFiltered);
		    lastDebugPrintTimeMillis = millis();
		}
        //digitalWrite(Z_STEP_PIN, (onMask & bit(Z_AXIS)));
}

void thcStepZUp(){
		if (thc_debug_setting->get() && ((millis() - lastDebugPrintTimeMillis) > thc_debugprint_millis->get()) )
		{
            grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "THC Voltage = %d Moving Z Up\n", torchVoltageFiltered);
		    lastDebugPrintTimeMillis = millis();
		}
        stepZTHC = 1;

}

void machine_init() {
    //solenoid_pull_count = 0; // initialize
    grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "Bill's THC Initialized");
    // setup PWM channel
    //solenoid_pwm_chan_num = sys_get_next_PWM_chan_num();
    //ledcSetup(solenoid_pwm_chan_num, SOLENOID_PWM_FREQ, SOLENOID_PWM_RES_BITS);
    //ledcAttachPin(SOLENOID_PEN_PIN, solenoid_pwm_chan_num);
    //pinMode(SOLENOID_DIRECTION_PIN, OUTPUT); // this sets the direction of the solenoid current
    //pinMode(REED_SW_PIN, INPUT_PULLUP);		 // external pullup required
    // setup a task that will do torch height control
    xTaskCreatePinnedToCore(THCSyncTask,   // task
                            "THCSyncTask", // name for task
                            4096,				// size of task stack
                            NULL,				// parameters
                            1,					// priority
                            &THCSyncTaskHandle,
                            0 // core
                           );
}

// this task tracks the Z position and sets the solenoid
void THCSyncTask(void* pvParameters) {
    int32_t current_position[N_AXIS]; // copy of current location
    float m_pos[N_AXIS];			  // machine position in mm
    TickType_t xLastWakeTime;
    const TickType_t xTHCFrequency = 50; // in ticks (typically ms)
    xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.
    while (true) {
        // don't ever return from this or the task dies
		unsigned long voltageInt = analogRead(5);
		float torchVoltage = voltageInt * (3.3/4095);
        uint8_t plasmaState = coolant_get_state(); //Using the coolant flood output to turn on the plasma cutter
        if(plasmaState) ///Plasma Has Been Turned On, Start THC Routine
        {
            if((millis()- arcOnTime) >arcDelayTime)
            {
                torchVoltageFiltered = torchVoltageFiltered * (torchFilterValue) + torchVoltage * (1-torchFilterValue); //Rough filter for voltage input
                if(torchVoltageFiltered > thc_voltage_setting -> get()) //Voltage is too high step Z down
                {
                    thcStepZDown();
                }
                else if(torchVoltageFiltered < thc_voltage_setting -> get()) //Voltage is too low step Z up
                {
                    thcStepZUp();
                }
                
            }
            else
            {
                torchVoltageFiltered = torchVoltage;
            }
        }
        else
        {
            torchVoltageFiltered = torchVoltage;
            arcOnTime = millis();
            if (thc_debug_setting->get() && ((millis() - lastDebugPrintTimeMillis) > thc_debugprint_millis->get()) )
            {
                grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "THC Interation # %d\n", THCCounter);
                grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "THC Voltage = %d\n", torchVoltageFiltered);
                lastDebugPrintTimeMillis = millis();
            }
        }
        
        memcpy(current_position, sys_position, sizeof(sys_position)); // get current position in step
        system_convert_array_steps_to_mpos(m_pos, current_position);  // convert to millimeters
        THCCounter ++;

        //calc_solenoid(m_pos[Z_AXIS]);								  // calculate kinematics and move the servos
/* 		    sprintf(gcode_line, "G0Z%3.2f\r", ATARI_TOOL_CHANGE_Z); // go to tool change height
			inputBuffer.push(gcode_line);
			for (uint8_t i = 0; i < move_count; i++) {
				sprintf(gcode_line, "G0X%3.2f\r", ATARI_HOME_POS); //
				inputBuffer.push(gcode_line);
				inputBuffer.push("G0X0\r");
			} */
        vTaskDelayUntil(&xLastWakeTime, xTHCFrequency);
    }
}