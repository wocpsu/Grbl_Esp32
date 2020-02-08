/*
User I/O is gcode controllable analog output. 

	Could be a standard M62 full on, M63 full off. (On = M62 P1) (Off = M63 P1)
	Could be a solenoid with a spike level followed by a hold level, then M63 off. 
	Could pulsed solenoid where M62 does a spike/hold then auto off after a duration.
	Could be used for Servo positions by setting on/off duty cycles to servo positions. M62 moves servo to one position and M63 move it to another


I/O Types 
	M62 On ----- M63 Off   
	M62 On Spike/Hold ----- M63 Off
	M62 Spike/Hold/DurtionOff

	Parameters
		PWM Frequency
		Spike Level (duty percent)
		Spike Duration (milliseconds)
		On Level (duty percent) 
		Off Level (duty percent)
		Hold Duration (milliseconds)
		
		Available Gcode letters
		P Parameter is which I/O pin
		L Parameter is Pulse Duration
		I tbd
		J
		K
*/

void user_io_control_init();
void sys_io_control(uint8_t io_num_mask, bool turnOn);