/*
    grbl_dynamixel.h
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

// Half Duplex UART Setup
#define DYNAMIXEL_TXD           GPIO_NUM_4
#define DYNAMIXEL_RXD           GPIO_NUM_13
#define DYNAMIXEL_RTS           GPIO_NUM_17
#define DYNAMIXEL_UART_PORT     UART_NUM_2
#define ECHO_TEST_CTS           UART_PIN_NO_CHANGE
#define DXL_BUF_SIZE            127
#define DXL_BAUD_RATE           1000000
#define DXL_RESPONSE_WAIT_TICKS 20 // how long to wait for a response

// protocol 2 byte positions
#define DXL_MSG_HDR1 0
#define DXL_MSG_HDR2 1
#define DXL_MSG_HDR3 2
#define DXL_MSG_RSRV  3 // reserved byte
#define DXL_MSG_ID   4
#define DXL_MSG_LEN_L 5
#define DXL_MSG_LEN_H 6
#define DXL_MSG_INSTR 7
#define DXL_MSG_START 8

#define DXL_BROADCAST_ID  0xFE

// protocol 2 instruction numbers
#define DXL_INSTR_PING 0x01
#define PING_RSP_LEN 14
#define DXL_READ   0x02
#define DXL_WRITE  0x03
#define DXL_SYNC_WRITE 0x83
#define DXL_BULK_WRITE 0x93

// protocol 2 register locations
#define DXL_OPERATING_MODE	11
#define DXL_MOVING_THRESHOLD 24
#define DXL_ADDR_TORQUE_EN	64
#define DXL_ADDR_LED_ON 		65
#define DXL_GOAL_POSITION	116  // 0x74
#define DXL_PRESENT_POSITION	132

// control modes 
#define DXL_CONTROL_MODE_VELOCITY		1
#define DXL_CONTROL_MODE_POSITION		3
#define DXL_CONTROL_MODE_EXT_POSITION	4
#define DXL_CONTROL_MODE_PWM				16




#ifndef grbl_dynamixel_h
    #define grbl_dynamixel_h

    #include "grbl.h"
    #include <HardwareSerial.h>
    #include "driver/uart.h"

    void dxl_init();
    void dxl_finish_message(char *msg, uint8_t servo_id, uint16_t msg_len);
    uint16_t dxl_update_crc(uint16_t crc_accum, char *data_blk_ptr, uint8_t data_blk_size);
    
    uint16_t dxl_get_response(uint16_t length);
    void dxl_ping(uint8_t id);
    void dxl_read(uint8_t id, uint16_t address, uint16_t data_len);

    void dxl_write(uint8_t id, uint16_t address, uint8_t paramCount,  ...);
    void dxl_torque_enable(uint8_t id, bool enabled);
    void dxl_moving_threshold(uint8_t id, uint16_t threshold);
    void dxl_LED_on(uint8_t id, bool on);
    void dxl_operating_mode(uint8_t id, uint8_t mode);
    void dxl_goal_position(uint8_t id, int32_t position);
    void dxl_bulk_goal_position(uint8_t *ids, int32_t *positions, uint8_t count);
    void dxl_sync_goal_position(uint8_t *ids, int32_t *positions, uint8_t count);
    uint32_t dxl_present_position(uint8_t id);

#endif