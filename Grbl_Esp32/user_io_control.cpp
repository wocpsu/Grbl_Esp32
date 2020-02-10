/*
	user_io_control.cpp - Implements simple inverse kinematics for Grbl_ESP32
	Part of Grbl_ESP32

	Copyright (c) 2020 Barton Dring @buildlog


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
	
	========================================================================

	User I/O Control allows gcode to control an I/O pin
	
	There are 4 (1-4) channels that can be used. These are mapped to any
	I/O pin that can do digital output
	
	There are 3 modes of operation
		1. Standard On/Off Mode: The output is digital with full on and full off. 
			M62 Px Turns the pin on where x is the channel number
			M63 Px Turns it off
			
			This mode is perfect for things like relays
			
		2. Spike Hold Duration Mode: This initially turns on PWM at a spike level, followed
			by a hold level. The hold will stay on unless you provide a length (duration)
			M62 Px Lnnnn Turns on the pin. The channel number is x and the duration is nnnn in milliseconds (32bit)
			If you do not provide an Lnnnn, duration the pin stay on until the M63 Px command
			
			This mode is perfect for things like solenoids that like an initial strong pull, but
			could overheat if left at full power.
			
		3. PWM Low/High Mode: This allow you to toggle between an On PWM duty and an off PWM duty
			M62 Px switches to an on PWM level
			M63 Px switches to an off  PWM level		
			The default is low at power on and after any reset, etc.				
			
			The could work with a hobby servo where low and high would be PWMs associated with travel points
			If your servo moves opposite of what you want, just reverse the values
	
			50Hz is a typical PWM to use with servos
			You would want the resolution to be 16 bitset
			
			#define SERVO_MIN_PULSE_SEC 0.001 // min pulse in seconds
			#define SERVO_MAX_PULSE_SEC 0.002 // max pulse in seconds
			
			#define SERVO_TIME_PER_BIT  ((1.0 / (float)SERVO_PULSE_FREQ) / ((float)SERVO_PULSE_RES_COUNT) ) // seconds
			#define SERVO_MIN_PULSE    (uint16_t)(SERVO_MIN_PULSE_SEC / SERVO_TIME_PER_BIT) // in timer counts
			#define SERVO_MAX_PULSE    (uint16_t)(SERVO_MAX_PULSE_SEC / SERVO_TIME_PER_BIT) // in timer counts
			
*/



#include "grbl.h"


static TaskHandle_t userIoSyncTaskHandle = 0;


#ifdef USER_DIGITAL_PIN_1
	UserIoControl Pin1_UserIoControl(1, USER_DIGITAL_PIN_1, sys_get_next_pwm_channel(), USER_DIGITAL_PIN_1_MODE);
#endif

#ifdef USER_DIGITAL_PIN_2
	UserIoControl Pin2_UserIoControl(2, USER_DIGITAL_PIN_2, sys_get_next_pwm_channel(), USER_DIGITAL_PIN_2_MODE);
#endif

#ifdef USER_DIGITAL_PIN_3
	UserIoControl Pin3_UserIoControl(3, USER_DIGITAL_PIN_3, sys_get_next_pwm_channel(), USER_DIGITAL_PIN_3_MODE);
#endif

#ifdef USER_DIGITAL_PIN_4
	UserIoControl Pin4_UserIoControl(4, USER_DIGITAL_PIN_4, sys_get_next_pwm_channel(), USER_DIGITAL_PIN_4_MODE);
#endif


void user_io_control_init() {
	bool needsTimer = false;
	
	#ifdef USER_DIGITAL_PIN_1
		Pin1_UserIoControl.init();
		if (Pin1_UserIoControl.needsTimerUpdates())
			needsTimer = true;
	#endif	
	#ifdef USER_DIGITAL_PIN_2
		Pin2_UserIoControl.init();	
		if (Pin2_UserIoControl.needsTimerUpdates())
			needsTimer = true;
	#endif
	#ifdef USER_DIGITAL_PIN_3
		Pin3_UserIoControl.init();
		if (Pin3_UserIoControl.needsTimerUpdates())
			needsTimer = true;
	#endif
	#ifdef USER_DIGITAL_PIN_4
		Pin4_UserIoControl.init();
		if (Pin4_UserIoControl.needsTimerUpdates())
			needsTimer = true;
	#endif
	
	if (needsTimer) {
		// setup a task that will calculate the determine and set the servo position		
		xTaskCreatePinnedToCore(	userIoSyncTask,    // task
   													"userIoSyncTask", // name for task
													4095,   // size of task stack
													NULL,   // parameters
													2, // priority
													&userIoSyncTaskHandle,
													0 // core
													);
	}
	
}

// this is the task
void userIoSyncTask(void *pvParameters)
{	
	while(true) { // don't ever return from this or the task dies	
			#ifdef USER_DIGITAL_PIN_1
				Pin1_UserIoControl.update();
			#endif
			#ifdef USER_DIGITAL_PIN_2
				Pin2_UserIoControl.update();
			#endif
			#ifdef USER_DIGITAL_PIN_3
				Pin3_UserIoControl.update();
			#endif
			#ifdef USER_DIGITAL_PIN_4
				Pin4_UserIoControl.update();
			#endif
			vTaskDelay(USER_IO_TASK_DELAY); // sets how often the loop runs
    }
}

// turns on User I/O. Use mask to select items to turn on. Optional duration in milliseconds (0 = stay on)
void sys_io_control(uint8_t io_num_mask, bool turnOn, uint16_t duration) {
	protocol_buffer_synchronize();  // make sure this waits until the planner is caught up.
	#ifdef USER_DIGITAL_PIN_1
		if (io_num_mask & 1<<1) {
			Pin1_UserIoControl.on(turnOn, duration);
			return;
		}
	#endif
	#ifdef USER_DIGITAL_PIN_2
		if (io_num_mask & 1<<2) {
			Pin2_UserIoControl.on(turnOn, duration);
			return;
		}
	#endif
	#ifdef USER_DIGITAL_PIN_3
		if (io_num_mask & 1<<3) {			
			Pin3_UserIoControl.on(turnOn, duration);
			return;
		}
	#endif
	#ifdef USER_DIGITAL_PIN_4
		if (io_num_mask & 1<<4) {
			Pin4_UserIoControl.on(turnOn, duration);
			return;
		}
	#endif
}


// ==================== Class Functions ===================



UserIoControl::UserIoControl(uint8_t gcode_number, int pin_num, uint8_t channel_num, uint8_t mode) // constructor
{
	_pin_num = pin_num;
	_channel_num = channel_num;
	_mode = mode;
	_gcode_num = gcode_number;
}

void UserIoControl::init()
{
	grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "User I/O %d on Pin %d PWM chan %d", _gcode_num, _pin_num, _channel_num);
	switch (_mode) {
		case USER_IO_MODE_ON_OFF:
			pinMode(_pin_num, OUTPUT);
			off();
			break;
		default:
			ledcSetup(_channel_num, _pwm_freq, _pwm_resolution_bits);
			ledcAttachPin(_pin_num, _channel_num);
			
			if (_mode == USER_IO_MODE_SPIKE_HOLD_OFF) {
				off();
			} else if (_mode == USER_MODE_PWM_LOW_HIGH) {
				_write_pwm(USER_MODE_PWM_LOW);
			}
			
			break;
	}
	
	
}

void UserIoControl::_write_pwm(uint32_t duty)
{
	//grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "User I/O PWM Duty %d", duty);
	if (ledcRead(_channel_num) != duty) { // only write if it is changing    
		ledcWrite(_channel_num, duty);
	}
}

uint32_t UserIoControl::_write_percent(uint8_t percent)
{
	uint32_t duty = mapConstrain(percent, 0.0, (1<<USER_IO_PULSE_RES_BITS), 0.0, 100.0);
	_write_pwm(duty);
}

void UserIoControl::on(bool isOn, uint32_t duration)
{
	switch(_mode) {
		case USER_IO_MODE_ON_OFF:
			digitalWrite(_pin_num, isOn);
			break;		
		case USER_IO_MODE_SPIKE_HOLD_OFF:
			if (isOn) {
				_phase = USER_IO_PHASE_SPIKE;
				_spike_end = esp_timer_get_time() + (USER_IO_SPIKE_DURATION * 1000); // The time the spike phase ends
				if (duration == 0) {
					// continuous hold
					_hold_end = 0;
				}
				else {
					// off after duration
					_hold_end = esp_timer_get_time() + (duration * 1000);
				}								
				_write_percent(_spike_percent);
			}
			else {
				_write_pwm(0);
			}			
			break;
		case USER_MODE_PWM_LOW_HIGH:
			if (isOn) {
				_write_pwm(_pwm_duty_high);
			}
			else {
				_write_pwm(_pwm_duty_low);
			}
			break;
		default:
			if (isOn)
				_write_pwm((1<<USER_IO_PULSE_RES_BITS));
			else
				_write_pwm(0);
			break;
	}
	_isOn = isOn;
}

void UserIoControl::off()
{
	switch(_mode) {
		case USER_IO_MODE_ON_OFF:
			digitalWrite(_pin_num, false);			
			break;
		case USER_MODE_PWM_LOW_HIGH:
			_write_pwm(_pwm_duty_low);
			break;
		default:
			_write_pwm(0);
			break;
	}
	_isOn = false;
}

bool UserIoControl::isOn()
{
	return _isOn;
}

void UserIoControl::set_hold_length(uint32_t length)
{
	_hold_length = length;
}

bool UserIoControl::needsTimerUpdates()
{
	return (_mode == USER_IO_MODE_SPIKE_HOLD_OFF);
		
}

// This is called by the main program on a regular interval
// It controls the timing of the spike hold and auto off modes of the I/O
void UserIoControl::update()
{
	if (! _isOn) // no need to do anything if if the i/O is off
		return;
	
	// no need to do anything for modes that don't need an update
	if ( (_mode == USER_IO_MODE_ON_OFF) || (_mode == USER_MODE_PWM_LOW_HIGH) )
		return;	
	
	// ====== USER_IO_MODE_SPIKE_HOLD_OFF mode =========
	if (_mode == USER_IO_MODE_SPIKE_HOLD_OFF) {
		
		if (_phase == USER_IO_PHASE_SPIKE) {
			if (esp_timer_get_time() > _spike_end){
				_phase = USER_IO_PHASE_HOLD;
				_write_percent(_hold_percent);
				return;
			}			
		}		
		
		if (_phase == USER_IO_PHASE_HOLD) {
			if (_hold_end == 0) {
				return;
			}
			if (esp_timer_get_time() > _hold_end){
				_write_pwm(0);
				_isOn = false;
				return;
			}	
		}
	}
}

//========================================




