/*************************************************************************
* Title		: hd44780_i2c.c
* Author	: Dimitri Dening
* Created	: 08.03.2025 22:06:32
* Software	: Microchip Studio V7
* Hardware	: Atmega2560
* License	: MIT License
* Usage		: see Doxygen manual
*
*       Copyright (C) 2025 Dimitri Dening
*
*       Permission is hereby granted, free of charge, to any person obtaining a copy
*       of this software and associated documentation files (the "Software"), to deal
*       in the Software without restriction, including without limitation the rights
*       to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*       copies of the Software, and to permit persons to whom the Software is
*       furnished to do so, subject to the following conditions:
*
*       The above copyright notice and this permission notice shall be included in all
*       copies or substantial portions of the Software.
*
*       THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*       IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*       FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*       AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*       LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*       OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*       SOFTWARE.
*
*************************************************************************/

/* General libraries */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>

/* User defined libraries */
#include "hd44780_i2c.h"
#include "i2c.h"

#define I2C_DEVICE_ADDR 0x27 
#define ENABLE_LATCH 0x02
#define BACKLIGHT_ON 0x03

static device_t* i2c_device;

void hd44780_i2c_init(void){
	
	i2c_device = i2c_create_device(I2C_DEVICE_ADDR);
	
	// Force 8-bit mode
	_delay_ms(40);
	hd44780_i2c_send_8_bit_instruction(RWRS00, 0x30);
	_delay_ms(5);
	hd44780_i2c_send_8_bit_instruction(RWRS00, 0x30);
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS00, 0x30);
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS00, 0x20);
	
	// Init phase
	hd44780_i2c_send_8_bit_instruction(RWRS00, FUNCTION_SET_4_BIT_MODE);
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS00, DISPLAY_OFF);
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS00, CLEAR_DISPLAY);
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS00, CURSOR_DIR_LEFT_SHIFT); // Entry mode
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS00, DISPLAY_ON | CURSOR_ON | BLINK_ON);
	_delay_ms(100);
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'd');
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'd');
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'e');
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'n');
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'i');
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'n');
	hd44780_i2c_send_8_bit_instruction(RWRS01, 'g');
}

void hd44780_i2c_send_8_bit_instruction(uint8_t opcode, uint8_t instruction){
	
	uint8_t* data;
	payload_t* payload;
	uint8_t high_nibble;
	uint8_t low_nibble;
	
	data = (uint8_t*)malloc(sizeof(uint8_t) * 4);
	
	if (data == NULL) {
		return;
	}
	
	high_nibble = instruction & 0xF0;
	low_nibble = (instruction & 0x0F) << 4;
	
	data[0] = high_nibble | (opcode << 0) | (1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[1] = high_nibble | (opcode << 0) & ~(1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);;
	data[2] = low_nibble  | (opcode << 0) | (1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);;
	data[3] = low_nibble  | (opcode << 0) & ~(1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);;
	
	payload = payload_create_i2c(PRIORITY_NORMAL, i2c_device, data, 4, NULL);
	
	if (payload == NULL) {
		return;
	}
	
	i2c_write(payload);
}