#include "grbl.h"
//#include <HardwareSerial.h>
#include "driver/uart.h"
#include "grbl_dynamixel.h"

/*
    grbl_dynamixel.cpp
    part of Grbl_ESP32

    This contains functions for communicating with Dynamixel Servos 
    using Robotis Protocol 2 http://emanual.robotis.com/docs/en/dxl/protocol2/

    Copyright (c) 2019 Barton Dring @buildlog	
	
    Grbl_ESP32 is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Grbl_ESP32 is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifdef USE_DYNAMIXEL

const uart_port_t uart_num = DYNAMIXEL_UART_PORT;

char dxl_tx_message[50]; // outgoing to dynamixel
uint8_t dxl_rx_message[50]; // received from dynamixel

void dxl_init()
{
    grbl_send(CLIENT_SERIAL, "[MSG:Init Dynamixel]\r\n");

    // setup the comm port as half duplex
    uart_config_t uart_config = {
        .baud_rate = DXL_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
  

    // Configure UART parameters    
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, DYNAMIXEL_TXD, DYNAMIXEL_RXD, DYNAMIXEL_RTS, UART_PIN_NO_CHANGE);
    uart_driver_install(uart_num, DXL_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
}

// ping is an good way to check communications with servos. 
// Returns model number and firmware reviison
void dxl_ping(uint8_t id)
{
	uint16_t len = 3;
	 
	dxl_tx_message[DXL_MSG_INSTR] = DXL_INSTR_PING;
	
	dxl_finish_message(dxl_tx_message, id, len);    
	
	len = dxl_get_response(PING_RSP_LEN); // wait for and get response
	
    if (len == PING_RSP_LEN) {
        uint16_t model_num = dxl_rx_message[10] << 8 | dxl_rx_message[9];
        if (model_num == 1060) {
            grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Detected ID %d Model XL430-W250 F/W Rev %x]\r\n", id, dxl_rx_message[11]);
        } else {
            grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Detected ID %d M/N %d F/W Rev %x]\r\n", id, model_num, dxl_rx_message[11]);
        }
        
    }
    else {
        grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Ping failed]\r\n", id);
	}
}

// wait for and get the servo response
uint16_t dxl_get_response(uint16_t length) {
    length = uart_read_bytes(DYNAMIXEL_UART_PORT, dxl_rx_message, length, DXL_RESPONSE_WAIT_TICKS);
    return length;
}

void dxl_read(uint8_t id, uint16_t address, uint16_t data_len)
{
	uint8_t msg_len = 3 + 4;
	
	dxl_tx_message[DXL_MSG_INSTR] = DXL_READ;
	dxl_tx_message[DXL_MSG_START] = (address & 0xFF); // low-order address value
	dxl_tx_message[DXL_MSG_START + 1] = ((address & 0xFF00) >> 8); // High-order address value
	dxl_tx_message[DXL_MSG_START + 2] = (data_len & 0xFF); // low-order data length value
	dxl_tx_message[DXL_MSG_START + 3] = ((data_len & 0xFF00) >> 8); // high-order address value
	
	dxl_finish_message(dxl_tx_message, id, msg_len);
}

/*
	This writes byte to the servo registers. You give it a start address and a count
	of bytes you are sending

	id:			The id of the servo
	address:		The starting address for the bytes to be placed
	paramCount:	The count of bytes supplies in the va_list
	...			The bytes as ints

*/
void dxl_write(uint8_t id, uint16_t address, uint8_t paramCount,  ...)
{
	
	dxl_tx_message[DXL_MSG_INSTR] = DXL_WRITE;
	dxl_tx_message[DXL_MSG_START] = (address & 0xFF); // low-order address value
	dxl_tx_message[DXL_MSG_START + 1] = ((address & 0xFF00) >> 8); // High-order address value
	
	uint8_t msg_offset = 1; // this is the offset from DXL_MSG_START in the message
	
	va_list valist;
	
	/* Initializing arguments  */
    va_start ( valist, paramCount);           
    
    for ( int x = 0; x < paramCount; x++ )   
    {
		msg_offset++;		
		dxl_tx_message[DXL_MSG_START + msg_offset] = (uint8_t)va_arg(valist, int);		
    }
    va_end ( valist );                  // Cleans up the list
	
	dxl_finish_message(dxl_tx_message, id, msg_offset + 4);	

    uint16_t len = 11; // response length
    len = dxl_get_response(len);

    if (len == 11) {
        uint8_t err = dxl_rx_message[8];
        switch (err) {
            case 1:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write fail error]\r\n", id);
                break;
            case 2:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write instruction error]\r\n", id);
                break;
            case 3:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write access error]\r\n", id);
                break;
            case 4:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write data range error]\r\n", id);
                break;
            case 5:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write data length error]\r\n", id);
                break;
            case 6:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write data limit error]\r\n", id);
                break;
            case 7:
                grbl_sendf(CLIENT_SERIAL, "[MSG:Dynamixel Servo ID %d Write access error]\r\n", id);
                break;
            default:
                break;
        }
    }
    else {
        // timeout
    }   
   
}

void dxl_torque_enable(uint8_t id, bool enabled)
{
	uint8_t param_count = 1;
	
	if (enabled)
		dxl_write(id, DXL_ADDR_TORQUE_EN, param_count, 1);
	else
		dxl_write(id, DXL_ADDR_TORQUE_EN, param_count, 0);
}

void dxl_moving_threshold(uint8_t id, uint16_t threshold)
{
    uint8_t param_count = 4;

    if (threshold > 0 && threshold < 1023)
    {
        dxl_write(id, DXL_MOVING_THRESHOLD, param_count, (threshold & 0xFF), (threshold & 0xFF00)>>8, 0, 0);
    }
}

void dxl_LED_on(uint8_t id, bool on)
{
	uint8_t param_count = 1;
	
	if (on)
		dxl_write(id, DXL_ADDR_LED_ON, param_count, 1);
	else
		dxl_write(id, DXL_ADDR_LED_ON, param_count, 0);
}

void dxl_operating_mode(uint8_t id, uint8_t mode) {
	uint8_t param_count = 1;
	dxl_write(id, DXL_OPERATING_MODE, param_count, mode);
}

void dxl_goal_position(uint8_t id, int32_t position) {
	uint8_t param_count = 4;
	
	dxl_write(id, DXL_GOAL_POSITION, param_count, (position & 0xFF), (position & 0xFF00)>>8, (position & 0xFF0000)>>16, (position & 0xFF000000)>>24);
}

void dxl_bulk_goal_position(uint8_t *ids, int32_t *positions, uint8_t count)
{
    uint16_t msg_index = DXL_MSG_INSTR; // index of the byte in the message we are currently filling
    
    dxl_tx_message[msg_index] = DXL_BULK_WRITE;

    for (uint8_t id=0; id<count; id++) {
        grbl_sendf(CLIENT_SERIAL, "-ID:%d-", id);
        dxl_tx_message[++msg_index] = ids[id]; // ID of the servo
        dxl_tx_message[++msg_index] = DXL_GOAL_POSITION & 0xFF; // low order address
        dxl_tx_message[++msg_index] = (DXL_GOAL_POSITION & 0xFF00) >> 8;// high order address
        dxl_tx_message[++msg_index] = 4; // low order data length
        dxl_tx_message[++msg_index] = 0; // high order data length
        dxl_tx_message[++msg_index] = positions[id] & 0xFF;// data
        dxl_tx_message[++msg_index] = (positions[id] & 0xFF00) >> 8;  // data
        dxl_tx_message[++msg_index] = (positions[id] & 0xFF0000) >> 16;  // data
        dxl_tx_message[++msg_index] = (positions[id] & 0xFF000000) >> 24;  // data
    }    
	dxl_finish_message(dxl_tx_message, DXL_BROADCAST_ID, (count * 9)+3);
}

void dxl_sync_goal_position(uint8_t *ids, int32_t *positions, uint8_t count)
{
    uint16_t msg_index = DXL_MSG_INSTR; // index of the byte in the message we are currently filling
    
    dxl_tx_message[msg_index] = DXL_SYNC_WRITE;
    dxl_tx_message[++msg_index] = DXL_GOAL_POSITION & 0xFF; // low order address
    dxl_tx_message[++msg_index] = (DXL_GOAL_POSITION & 0xFF00) >> 8;// high order address
    dxl_tx_message[++msg_index] = 4; // low order data length
    dxl_tx_message[++msg_index] = 0; // high order data length

    for (uint8_t id=0; id<count; id++) {        
        dxl_tx_message[++msg_index] = ids[id]; // ID of the servo        
        dxl_tx_message[++msg_index] = positions[id] & 0xFF;// data
        dxl_tx_message[++msg_index] = (positions[id] & 0xFF00) >> 8;  // data
        dxl_tx_message[++msg_index] = (positions[id] & 0xFF0000) >> 16;  // data
        dxl_tx_message[++msg_index] = (positions[id] & 0xFF000000) >> 24;  // data
    }    
	dxl_finish_message(dxl_tx_message, DXL_BROADCAST_ID, (count * 5)+7);
}



uint32_t dxl_present_position(uint8_t id) {
	uint8_t data_len = 4;
    uint32_t position;

	dxl_read(id, DXL_PRESENT_POSITION, data_len);   

    data_len = dxl_get_response(15);

    if (data_len == 15) {
        position = dxl_rx_message[9] | dxl_rx_message[10]<<8 | dxl_rx_message[11]<<16 | dxl_rx_message[12]<<24;        
        return position;
    }
    else {
        return 0;
	}
}

/*
    This is a helper function to complete and send the message
    The body of the message should be in msg, at the correct location
    before calling this function.

    This function will add the header, length bytes and CRC
    It will then send the message
*/
void dxl_finish_message(char *msg, uint8_t servo_id, uint16_t msg_len)
{
	//uint16_t msg_len;
	uint16_t crc = 0;
	// header
	msg[DXL_MSG_HDR1] = 0xFF;
    msg[DXL_MSG_HDR2] = 0xFF;
    msg[DXL_MSG_HDR3] = 0xFD;
	// 
	// reserved
    msg[DXL_MSG_RSRV] = 0x00;
	msg[DXL_MSG_ID] = servo_id;
	// length
	msg[DXL_MSG_LEN_L] = msg_len & 0xFF;
    msg[DXL_MSG_LEN_H] = (msg_len & 0xFF00) >> 8;
	
	// the message should already be here
		
	crc = dxl_update_crc(crc, msg, 5 + msg_len);
	
	msg[msg_len + 5] = crc & 0xFF; // CRC_L
    msg[msg_len + 6] = (crc & 0xFF00) >> 8;

    // debug
    /*
    grbl_sendf(CLIENT_SERIAL, "[MSG: TX:");
    for (uint8_t index = 0; index < msg_len + 7; index++) {
        grbl_sendf(CLIENT_SERIAL, " 0x%02X", msg[index]);
    }
    grbl_sendf(CLIENT_SERIAL, "]\r\n");
	*/

	uart_write_bytes(uart_num, msg, msg_len + 7);
}

// from http://emanual.robotis.com/docs/en/dxl/crc/
uint16_t dxl_update_crc(uint16_t crc_accum, char *data_blk_ptr, uint8_t data_blk_size)
{
    uint16_t i, j;
    uint16_t crc_table[256] = {
        0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
        0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
        0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
        0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
        0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
        0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
        0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
        0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
        0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
        0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
        0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
        0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
        0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
        0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
        0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
        0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
        0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
        0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
        0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
        0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
        0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
        0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
        0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
        0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
        0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
        0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
        0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
        0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
        0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
        0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
        0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
        0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
    };

    for(j = 0; j < data_blk_size; j++)
    {
        i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
        crc_accum = (crc_accum << 8) ^ crc_table[i];
    }

    return crc_accum;
}


#endif