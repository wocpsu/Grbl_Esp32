/*
	user_io_control.h - 

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

#ifndef user_io_control_h
  #define user_io_control_h
  
#define USER_IO_TASK_DELAY 10

#define USER_IO_PULSE_FREQ 5000 // default frequency
#define USER_IO_PULSE_RES_BITS 8 // bits of resolution of PWM (16 is max)

#define USER_IO_MODE_ON_OFF 		0   // M62 Px = On .... M63 Px = Off
#define USER_IO_MODE_SPIKE_HOLD_OFF 1   // M62 Px Lx = On at spike level...then goes to a hold level...then goes off after L milliseconds
#define USER_MODE_PWM_LOW_HIGH	2   // Analog M62 Px = High and M63 Px = Low (Can work for servos)

#define USER_IO_PHASE_NONE		0
#define USER_IO_PHASE_SPIKE 	1
#define USER_IO_PHASE_HOLD   	2

#define USER_IO_SPIKE_DURATION	100
#define USER_IO_HOLD_DURATION   1000

#define USER_MODE_PWM_LOW   0 // not percent actual duty count
#define USER_MODE_PWM_HIGH   (1<<USER_IO_PULSE_RES_BITS)-1 // not percent actual duty count



//void user_io_control_init_old();
//void sys_io_control_old(uint8_t io_num_mask, bool turnOn);

void user_io_control_init();
void sys_io_control(uint8_t io_num_mask, bool turnOn, uint16_t duration = 0);
void userIoSyncTask(void *pvParameters);

class UserIoControl{
	public:
		UserIoControl(uint8_t gcode_number, int pin_num, uint8_t channel_num, uint8_t mode); // constructor		
		void init();
		void on(bool isOn, uint32_t duration = 0);
		void off();
		bool isOn();
		bool needsTimerUpdates();
		void update(); // updates spike and hold phases
		
		void set_mode(uint8_t mode);
		void set_spike_length(uint16_t length);
		void set_hold_length(uint32_t length);
		void set_pwm_freq_bits(uint32_t pwm_freq, uint8_t bit_num);
		void set_spike_hold_perecent(uint8_t spike_percent, uint8_t hold_percent);
		void set_pwm_low_high(uint16_t pwm_duty_low, uint16_t pwm_duty_high);
		
	private:
		uint8_t _gcode_num = 0;
		int _pin_num; // The GPIO pin being used
		int _mode = USER_IO_MODE_ON_OFF;
		uint8_t _phase = USER_IO_PHASE_NONE;
		int _channel_num; // The PWM channel
		bool _isOn = false;
		uint8_t _pwm_resolution_bits = USER_IO_PULSE_RES_BITS;
		uint32_t _pwm_freq = USER_IO_PULSE_FREQ;
		uint16_t _spike_length = USER_IO_SPIKE_DURATION;
		uint32_t _hold_length = USER_IO_HOLD_DURATION;
		
		uint16_t _pwm_duty_low = USER_MODE_PWM_LOW;
		uint16_t _pwm_duty_high = USER_MODE_PWM_HIGH;
		
		uint8_t _spike_percent = 100;
		uint8_t _hold_percent = 50;
		
		int64_t _spike_end; // microseconds ... gets compared to esp_timer_get_time()
		int64_t _hold_end; // microseconds ... gets compared to esp_timer_get_time()
		
		
		// ===== functions ==========
		void _write_pwm(uint32_t duty);
		void _write_percent(uint8_t percent);
		
		
		
};

// allow other modules to have access to the objects for specific machine setup
#ifdef USER_DIGITAL_PIN_1
	extern UserIoControl Pin1_UserIoControl;
#endif
#ifdef USER_DIGITAL_PIN_2
	extern UserIoControl Pin2_UserIoControl;
#endif
#ifdef USER_DIGITAL_PIN_3
	extern UserIoControl Pin3_UserIoControl;
#endif
#ifdef USER_DIGITAL_PIN_4
	extern UserIoControl Pin4_UserIoControl;
#endif
#ifdef USER_DIGITAL_PIN_5
	extern UserIoControl Pin5_UserIoControl;
#endif
#ifdef USER_DIGITAL_PIN_6
	extern UserIoControl Pin6_UserIoControl;
#endif

#endif