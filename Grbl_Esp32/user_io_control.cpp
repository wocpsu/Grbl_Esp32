// This is M62 M63 stuff

#include "grbl.h"

void user_io_control_init()
{
	#ifdef USER_DIGITAL_PIN_1
		pinMode(USER_DIGITAL_PIN_1, OUTPUT);
		grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "User I/O 1 on pin %d", USER_DIGITAL_PIN_1);	
		sys_io_control(1<<1, false); // turn off
	#endif

	#ifdef USER_DIGITAL_PIN_2
		pinMode(USER_DIGITAL_PIN_2, OUTPUT);
		grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "User I/O 2 on pin %d", USER_DIGITAL_PIN_2);
		sys_io_control(1<<2, false); // turn off
	#endif

	#ifdef USER_DIGITAL_PIN_3
		pinMode(USER_DIGITAL_PIN_3, OUTPUT);
		grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "User I/O 3 on pin %d", USER_DIGITAL_PIN_3);
		sys_io_control(1<<3, false); // turn off
	#endif

	#ifdef USER_DIGITAL_PIN_4
		pinMode(USER_DIGITAL_PIN_4, OUTPUT);
		grbl_msg_sendf(CLIENT_SERIAL, MSG_LEVEL_INFO, "User I/O 4 on pin %d", USER_DIGITAL_PIN_4);
		sys_io_control(1<<4, false); // turn off
	#endif	
}

// io_num is the virtual pin# and has nothing to do with the actual esp32 GPIO_NUM_xx
// It uses a mask so all can be turned of in ms_reset
void sys_io_control(uint8_t io_num_mask, bool turnOn) {
	protocol_buffer_synchronize();
	#ifdef USER_DIGITAL_PIN_1
		if (io_num_mask & 1<<1) {
			digitalWrite(USER_DIGITAL_PIN_1, turnOn);
			return;
		}
	#endif
	#ifdef USER_DIGITAL_PIN_2
		if (io_num_mask & 1<<2) {
			digitalWrite(USER_DIGITAL_PIN_2, turnOn);
			return;
		}
	#endif
	#ifdef USER_DIGITAL_PIN_3
		if (io_num_mask & 1<<3) {
			digitalWrite(USER_DIGITAL_PIN_3, turnOn);
			return;
		}
	#endif
	#ifdef USER_DIGITAL_PIN_4
		if (io_num_mask & 1<<4) {
			digitalWrite(USER_DIGITAL_PIN_4, turnOn);
			return;
		}
	#endif
}