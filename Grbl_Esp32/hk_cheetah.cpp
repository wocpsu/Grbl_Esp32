/*
  hk_cheetah.cpp 
  
  Copyright (c) 2019 Barton Dring @buildlog
  
  !!! Experimental Use Only !!!
  
  This feature controls a Hobby King Cheetah motor
  The motors travel is mapped to machine space.
  Where the motor's 16 bit position units are treated as steps
  Currently testing with X_AXIS only
      
	  
 TODO:
 
 Consider different ways to disable the motor...with a little torque?
	 
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
#include "grbl.h"

#ifdef CPU_MAP_CHEETAH
	//#include <CAN.h> // https://github.com/sandeepmistry/arduino-CAN


static TaskHandle_t cheetahSyncTaskHandle = 0;  // Task to sync motor with Grbl

// values read from cheetah motor
uint8_t cheetah_id;
uint16_t cheetah_pos;
uint16_t cheetah_vel;
uint16_t cheetah_current;

// Torque values
uint16_t cheetah_kp = 40;
uint16_t cheetah_kd = 40;
uint16_t cheetah_feed_forward = 40;

bool cheetah_torque_enable_state = false; // is the motor enabled?

uint16_t cheetah_startup_delay; // give time for all features to get ready

uint16_t cheetah_idle_counter;

 void machine_init()
{
	CAN.onReceive(onCanReceive); // setup a CAN response listener
	
	// Cheetah motor use 1Mbs CAN
	grbl_sendf(CLIENT_SERIAL, "[MSG:Cheetah Init]\r\n");
	if (!CAN.begin(1000E3)) { 
		grbl_sendf(CLIENT_SERIAL, "[MSG:Starting CAN failed!]\r\n");		
		while (1);
	}
	grbl_sendf(CLIENT_SERIAL, "[MSG:CAN Started]\r\n");

	cheetah_startup_delay = CHEETAH_STARTUP_DELAY; // give time for all features to get ready
	
	
	// setup a task that will sync motor position with grbl position		
	xTaskCreatePinnedToCore(	cheetahSyncTask,    // task
								"cheetahSyncTask", // name for task
								4096,   // size of task stack
								NULL,   // parameters
								1, // priority
								&cheetahSyncTaskHandle,
								0 // core
							);
}

// this is the RTOS task
void cheetahSyncTask(void *pvParameters)
{ 
  TickType_t xLastWakeTime;
  const TickType_t xdynamixelSyncFrequency = CHEETAH_TIMER_INT_FREQ;  // in ticks (typically ms)  

  xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.
  while(true) { // Note: Don't ever return from this or the task dies		
		// let the startup dealy expire before doing anything.
		if (cheetah_startup_delay != 0) {
			cheetah_startup_delay--;
		}
		else {
			cheetah_sync_position();			
		}
		vTaskDelayUntil(&xLastWakeTime, xdynamixelSyncFrequency);
  }
}

// 
void onCanReceive(int packetSize) {

  int can_msg[packetSize]; // Cheetah response packets are always 6 bytes

  for (uint8_t i = 0; i < packetSize; i++) {
     can_msg[i] = CAN.read();
  }
  
  if (!stepper_idle) // If not disabled ignore the response, Grbl is in control
	  return;

  /*
   * 8 bit motor ID
    16 bit position, scaled between P_MIN and P_MAX in CAN_COM.cpp
    12 bit velocity, between 0 and 4095, scaled V_MIN and V_MAX
    12 bit current, between 0 and 4095, scaled to -40 and 40 Amps, corresponding to peak phase current.    
  */


  cheetah_id = can_msg[0];
  cheetah_pos = (can_msg[1] << 8) + can_msg[2];
  cheetah_vel = (can_msg[3] << 4) + ((can_msg[4] & 0xF0) >> 4);
  cheetah_current = ((can_msg[4] & 0x0F) << 8) + can_msg[5];
  
  //grbl_sendf(CLIENT_SERIAL, "[MSG:Cheetah Pos %d]\r\n", cheetah_pos);
	//return;
  
  
  
  // update Grbl posiiton
    sys_position[X_AXIS] = cheetah_pos;
	//sys_position[Y_AXIS] = y * settings.steps_per_mm[Y_AXIS];
	//sys_position[Z_AXIS] = z * settings.steps_per_mm[Z_AXIS];

    gc_sync_position();
    plan_sync_position();
  

}

// this is called by the RTOS task
void cheetah_sync_position() {
	// see if we need to change the enable state
	if (stepper_idle == cheetah_torque_enable_state) { // they are opposites, so if they are equal change...
		if (!stepper_idle)
			cheetahMotorMode(MOTOR_ON);
		//else
			//cheetah_idle_counter = CHEETAH_IDLE_TIMER;
		
		//grbl_sendf(CLIENT_SERIAL, "[MSG:Cheetah Mode %d]\r\n", cheetah_torque_enable_state);
		
	}
	
	if (stepper_idle) {
		// read the position and update grbl
		cheetahMotorMode(MOTOR_OFF); // a way to get a response
	}
	else {
		// determine where the motor should be and send the move command
		uint16_t target_pos = sys_position[X_AXIS];
		//grbl_sendf(CLIENT_SERIAL, "[MSG:Cheetah Move %d]\r\n", target_pos);
		hk_sendMoveCmd(	CHEETAH_ID,
						target_pos, 0,
						settings.machine_int16[0],
						settings.machine_int16[1],
						settings.machine_int16[2]);
	}
}

void hk_sendMoveCmd(uint8_t id, uint16_t new_pos, uint16_t new_vel, uint16_t new_kp, uint16_t new_kd, uint16_t new_ff)
{
  char can_msg[8];

  can_msg[0] = (new_pos & 0xFF00) >> 8;
  can_msg[1] = new_pos & 0x00FF;
  can_msg[2] = (new_vel >> 4) & 0xFF;
  can_msg[3] = ((new_vel & 0x000F) << 4) + ((new_kp >> 8) & 0xFF);
  can_msg[4] = new_kp & 0xFF;
  can_msg[5] = new_kd >> 4;
  can_msg[6] = ((new_kd & 0x000F)<<4) + (new_ff >> 8);
  can_msg[7] = new_ff & 0xff;

  

  CAN.beginPacket(0x01);
  for (uint8_t i = 0; i < 8; i++) {
     CAN.write(can_msg[i]);
  }
  CAN.endPacket();
  
}

void cheetahMotorMode(int mode)
{
	
	//grbl_sendf(CLIENT_SERIAL, "[MSG:Cheetah Mode %d]\r\n", mode);
	
  CAN.beginPacket(0x01);
  
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  if (mode == MOTOR_ON) {
    CAN.write(0xFC);
  }
  else {
    CAN.write(0xFD);
  }
    
  CAN.endPacket();
  
  cheetah_torque_enable_state = (mode == MOTOR_ON); // save so it can be referenced elsewhere
  
}

void cheetahEncoderZero()
{
  CAN.beginPacket(0x01);
  
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFF); 
  CAN.write(0xFF);
  CAN.write(0xFF);
  CAN.write(0xFE);
    
  CAN.endPacket();
}
#endif
