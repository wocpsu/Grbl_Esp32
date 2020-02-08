/*
  dxl_delta.cpp - Machine definition of the delta using 
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

    FYI: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/

    Better: http://hypertriangle.com/~alex/delta-robot-tutorial/

*/


#include "grbl.h"
#ifdef CPU_MAP_DXL_DELTA

// trigonometric constants
 const float sqrt3 = 1.732050807;
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1.0/sqrt3;

 // the geometry of the delta
 const float rf = 70.0; // radius of the fixed side (length of motor craks)
 const float re =  133.5; // radius of end effector side (length of linkages)

const float f = 179.437f; // sized of fixed side triangel
const float e = 86.6025f; // size of end effector side triangle

float delta_z_offset;

static TaskHandle_t dynamixelSyncTaskHandle = 0;  // TO DO rename tap delta

uint8_t my_servo_ids[N_AXIS] = {DXL_MOTOR_0_ID, DXL_MOTOR_1_ID, DXL_MOTOR_2_ID};
int32_t my_pos[N_AXIS];
bool dxl_delta_torque_enable_state = false;

void machine_init() {  
    
	grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "DXL Delta Init"); // print a message 
	
	// Calculate the Z offset at the motor zero angles ... 
	// Z offset is the z distance from the motor axes to the end effector axes at zero angle
	float x0,y0;
	delta_calcForward(0.0, 0.0, 0.0, x0, y0, delta_z_offset);	
	// grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "DXL Delta Z Offset at 0,0,0 is:%4.3f", delta_z_offset);  // uncomment if you want to see the z offset
	
	
    dxl_init(); // setup the UART
       
    // ping the servos...they will [MSG:...] their m/n and f/w rev. 
    dxl_ping(DXL_MOTOR_0_ID);
    dxl_ping(DXL_MOTOR_1_ID);
    dxl_ping(DXL_MOTOR_2_ID);

    
    // turn off torque so we can set EEPROM registers...it gets turned back on in dynamixelSyncTask
    dxl_delta_torque_enable(false);

    // put into position mode
    dxl_operating_mode(DXL_MOTOR_0_ID, DXL_CONTROL_MODE_POSITION);
    dxl_operating_mode(DXL_MOTOR_1_ID, DXL_CONTROL_MODE_POSITION);
    dxl_operating_mode(DXL_MOTOR_2_ID, DXL_CONTROL_MODE_POSITION);

    // get the current position
    dxl_read_position();

	// setup a task that will sync servo position with grbl position		
	xTaskCreatePinnedToCore(	dynamixelSyncTask,    // task
								"dynamixelSyncTask", // name for task
								4096,   // size of task stack
								NULL,   // parameters
								1, // priority
								&dynamixelSyncTaskHandle,
								0 // core
							);

    return;
}

// this is the task
void dynamixelSyncTask(void *pvParameters)
{ 
  TickType_t xLastWakeTime;
  const TickType_t xdynamixelSyncFrequency = DXL_TIMER_INT_FREQ;  // in ticks (typically ms)  

  xLastWakeTime = xTaskGetTickCount(); // Initialise the xLastWakeTime variable with the current time.
  while(true) { // don't ever return from this or the task dies			
        dxl_sync_position(); // move servos to grbl position		
		vTaskDelayUntil(&xLastWakeTime, xdynamixelSyncFrequency);
  }
} 

bool user_defined_homing()
{
	grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "Homing is not used on this machine"); // print a message
	return true; // true = do not continue with normal Grbl homing
}

void dxl_sync_position()
{
     // track the state of this so we can react to changes (servos start in idle)
    
	
    float theta1, theta2, theta3;
    uint8_t axis;
    bool error = false;
    bool moved = false;
	
    float mpos[N_AXIS]; // current machine position
    float last_mpos[N_AXIS]; // last machine position
    float dxl_z_pos;    

    // check if the state has changed    
    if (stepper_idle == dxl_delta_torque_enable_state) {  // they are opposites, so if they are equal change...
        // sync the servo torque registers to the steppers setting
        dxl_delta_torque_enable(!stepper_idle);
    }

    int32_t current_position[N_AXIS]; // copy of current location	
	memcpy(current_position,sys_position,sizeof(sys_position));  // get current position in step	
	system_convert_array_steps_to_mpos(mpos,current_position); // convert to millimeters
   

    // make sure end effector does not go below the bed
    /*
    if (mpos[Z_AXIS] < 0 ) {
        grbl_sendf(CLIENT_SERIAL, "[MSG: DXL Z Out of range]\r\n");
        return;
    }
    */

    // check to see if the position moved since the last time thorugh    
    for (int axis=0; axis<N_AXIS; axis++){
        if (sys_position[axis] != last_mpos[axis]) {
            moved = true;
            break;
        }
    }

    moved = true; // override it for now
        
    if (stepper_idle) {
        // sync from manual moves
        dxl_read_position();
    } else {
        if (moved) {
            int status = delta_calcInverse(mpos[X_AXIS] , mpos[Y_AXIS] , mpos[Z_AXIS] , theta1, theta2, theta3);

            if (status != KIN_ANGLE_CALC_OK) {
                grbl_sendf(CLIENT_SERIAL, "[MSG: DXL Out of range]\r\n"); 
            } else {              
                my_pos[0] =  DXL_value(theta1, DXL_MOTOR_0_CENTER, DXL_MOTOR_0_FWD);
                my_pos[1] =  DXL_value(theta2, DXL_MOTOR_1_CENTER, DXL_MOTOR_1_FWD);
                my_pos[2] =  DXL_value(theta3, DXL_MOTOR_2_CENTER, DXL_MOTOR_2_FWD);
                dxl_sync_goal_position(my_servo_ids, my_pos, N_AXIS);                
            }
        }
    } 
    memcpy(last_mpos,sys_position,sizeof(sys_position)); 
}

 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
     theta1 = theta2 = theta3 = 0;
     int status = delta_calcAngleYZ(x0, y0, z0, theta1);

     if (status == KIN_ANGLE_CALC_OK) 
        status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg

     if (status == KIN_ANGLE_CALC_OK) 
        status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg

     return status;
}

int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
        

     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
     if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;
     return 0;
 }
 
 
void dxl_read_position() {
    uint32_t encoder[3]; // actual values read from servo in encoder counts 0-4095
    float theta[3];      // calculated degrees from encoder count
    float x, y, z;    // calculated xyz position from servo angles

    // read the raw encoder values
    encoder[0] = dxl_present_position(DXL_MOTOR_0_ID);
    encoder[1] = dxl_present_position(DXL_MOTOR_1_ID);
    encoder[2] = dxl_present_position(DXL_MOTOR_2_ID);

    //grbl_sendf( CLIENT_SERIAL, "[MSG:Servo counts %d %d %d]\r\n", encoder[0], encoder[1], encoder[2]);

    // convert encoder values to degrees
    theta[0] = DXL_value_degrees((int32_t)encoder[0], DXL_MOTOR_0_CENTER, DXL_MOTOR_0_FWD);
    theta[1] = DXL_value_degrees((int32_t)encoder[1], DXL_MOTOR_1_CENTER, DXL_MOTOR_1_FWD);
    theta[2] = DXL_value_degrees((int32_t)encoder[2], DXL_MOTOR_2_CENTER, DXL_MOTOR_2_FWD);

    //grbl_sendf( CLIENT_SERIAL, "[MSG:Servo angles %3.2f %3.2f %3.2f]\r\n", theta[0], theta[1], theta[2]);

    // use degrees to find XYZ positions
    delta_calcForward( theta[0],  theta[1],  theta[2], x, y, z);

    //grbl_sendf( CLIENT_SERIAL, "[MSG:Postion %3.2f %3.2f %3.2f]\r\n\r\n", x, y, z);

    // update Grbl posiiton
    sys_position[X_AXIS] = x * settings.steps_per_mm[X_AXIS];
	sys_position[Y_AXIS] = y * settings.steps_per_mm[Y_AXIS];
	sys_position[Z_AXIS] = z * settings.steps_per_mm[Z_AXIS];

    gc_sync_position();
    plan_sync_position();
  
}

 // helper functions, calculates angle theta1 (for YZ-pane)
 int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 
     if (d < 0) return -1; // non-existing point
     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
     return 0;
 }
 
 


// convert degrees to Dynamixel counts. For wiring purposes some may be mounted inverted.
// Use the forward parameter to correct for backwards rotation.
 uint16_t DXL_value(float degrees, uint16_t centalVal, bool forward)
 {
     if (forward) {
        return centalVal + (degrees * DXL_COUNT_PER_DEG);
    } else {
        return centalVal - (degrees * DXL_COUNT_PER_DEG);
    }    
 }

 float DXL_value_degrees(int32_t value, int16_t centerVal, bool forward)
 {
    float degrees = (float)(value - centerVal) * .088f;

    // positive degrees is down 
    if (!forward)
        degrees *= -1.0f;

    return degrees;
    
 }

void dxl_delta_torque_enable(bool enable)
{
    dxl_torque_enable(DXL_MOTOR_0_ID, enable);
    dxl_torque_enable(DXL_MOTOR_1_ID, enable);
    dxl_torque_enable(DXL_MOTOR_2_ID, enable);

    dxl_delta_torque_enable_state = enable;

    //grbl_sendf(CLIENT_SERIAL, "[MSG:Torque Enable %d]\r\n", enable);

}

void forward_kinematics(float *position)
{
	float calc_fwd[N_AXIS];
	
	//grbl_sendf(CLIENT_ALL,"[MSG:Fwd Kin Calc 1....%4.3f %4.3f %4.3f]\r\n", position[X_AXIS], position[Y_AXIS], position[Z_AXIS]);
	
	position[X_AXIS] += gc_state.coord_system[X_AXIS]+gc_state.coord_offset[X_AXIS];
	position[Y_AXIS] += gc_state.coord_system[Y_AXIS]+gc_state.coord_offset[Y_AXIS];
	position[Z_AXIS] += gc_state.coord_system[Z_AXIS]+gc_state.coord_offset[Z_AXIS];
	
	//grbl_sendf(CLIENT_ALL,"[MSG:Fwd Kin Calc 2....%4.3f %4.3f %4.3f]\r\n", position[X_AXIS], position[Y_AXIS], position[Z_AXIS]);
	
	if (delta_calcForward(position[X_AXIS], position[Y_AXIS], position[Z_AXIS], calc_fwd[X_AXIS], calc_fwd[Y_AXIS], calc_fwd[Z_AXIS]) == 0) {
		position[X_AXIS] = calc_fwd[X_AXIS] - (gc_state.coord_system[X_AXIS]+gc_state.coord_offset[X_AXIS]);
		position[Y_AXIS] = calc_fwd[Y_AXIS] - (gc_state.coord_system[Y_AXIS]+gc_state.coord_offset[Y_AXIS]);
		position[Z_AXIS] = calc_fwd[Z_AXIS] - (gc_state.coord_system[Z_AXIS]+gc_state.coord_offset[Z_AXIS]) - delta_z_offset;
		//grbl_sendf(CLIENT_ALL,"[MSG:Fwd Kin Calc....3 %4.3f %4.3f %4.3f %4.3f]\r\n", position[X_AXIS], position[Y_AXIS], position[Z_AXIS], delta_z_offset);
	}
	else {
		grbl_send(CLIENT_SERIAL, "[MSG:Fwd Kin Error]\r\n");
	}
}

// handle the M30 command
void user_m30() {
	inputBuffer.push("$H\r");
}



#endif

