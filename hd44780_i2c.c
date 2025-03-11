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
#include <avr/interrupt.h>
#include <util/delay.h>

/* User defined libraries */
#include "hd44780_i2c.h"
#include "i2c.h"
#include "led_lib.h"
#include "ringbuffer.h"

#define I2C_DEVICE_ADDR 0x27 
#define ENABLE_LATCH 0x02
#define BACKLIGHT_ON 0x03

typedef enum {
	LCD_STATE_IDLE,
	LCD_STATE_CHECK_BUSY,
	LCD_STATE_SEND_COMMAND
} lcd_fsm_state_t;

volatile lcd_fsm_state_t lcd_fsm_state = LCD_STATE_IDLE;

static queue_t q;
static queue_t* queue = NULL;
static device_t* i2c_device;
static uint8_t g_busy_flag;

static void timer1_init(void);
static void _hd44780_send_command(void);
static void _hd44780_i2c_request_busy_flag(void);
static void _hd44780_i2c_start_check_busy_flag_routine(void);

void static _hd44780_i2c_send_8bit_init(uint8_t opcode, uint8_t instruction) {
	
	uint8_t* data;
	uint8_t high_nibble;
	uint8_t low_nibble;
	payload_t* payload;
	
	data = (uint8_t*)malloc(sizeof(uint8_t) * 4);
	
	if (data == NULL) {
		return;
	}
	
	high_nibble = instruction & 0xF0;
	low_nibble = (instruction & 0x0F) << 4;
		
	data[0] = high_nibble | (opcode << 0) | (1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[1] = high_nibble | (opcode << 0) & ~(1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[2] = low_nibble  | (opcode << 0) | (1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[3] = low_nibble  | (opcode << 0) & ~(1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	
	payload = payload_create_i2c(PRIORITY_NORMAL, i2c_device, data, 4, NULL);
	
	if (payload == NULL) {
		return;
	}
	
	i2c_write(payload);
}

void hd44780_i2c_init(void) {
	
	queue = queue_init(&q);
	
	timer1_init();
	
	i2c_device = i2c_create_device(I2C_DEVICE_ADDR);
	
	// Force 8-bit mode
	_delay_ms(100); //_delay_ms(40);
	_hd44780_i2c_send_8bit_init(RWRS00, 0x30);
	_delay_ms(100); // _delay_ms(5);
	_hd44780_i2c_send_8bit_init(RWRS00, 0x30);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, 0x30);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, 0x20);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, FUNCTION_SET_4_BIT_MODE);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, FUNCTION_SET_4_BIT_MODE);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, DISPLAY_OFF);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, CLEAR_DISPLAY);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, CURSOR_DIR_LEFT_SHIFT);
	_delay_ms(100);
	_hd44780_i2c_send_8bit_init(RWRS00, DISPLAY_ON | CURSOR_ON | BLINK_ON);
	_delay_ms(100);
	
	_hd44780_i2c_send_8bit_init(RWRS01, 'A');
	
	//queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, DISPLAY_OFF));
	//queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, CLEAR_DISPLAY));
	//queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, CURSOR_DIR_LEFT_SHIFT));
	//queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, DISPLAY_ON | CURSOR_ON | BLINK_ON));
	
	//hd44780_i2c_puts("Hallo Queue Test");
	//queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, CLEAR_DISPLAY));
	//hd44780_i2c_puts("XYZ Test");
	//_hd44780_send_command();
}

void _hd44780_send_command(void) {
	
	uint8_t opcode;
	uint8_t instruction;
	payload_t* payload;
	
	payload = queue_dequeue(queue);
	
	if (payload == NULL) {
		led_toggle(LED7);
		lcd_fsm_state = LCD_STATE_IDLE;
		return;
	}
	
	opcode = payload->protocol.hd44780.opcode;
	
	instruction = payload->protocol.hd44780.instruction;
	
	lcd_fsm_state = LCD_STATE_SEND_COMMAND;
	
	hd44780_i2c_send_8_bit_instruction(opcode, instruction);
}

void hd44780_i2c_send_8_bit_instruction(uint8_t opcode, uint8_t instruction){
	
	uint8_t* data;
	uint8_t high_nibble;
	uint8_t low_nibble;
	payload_t* payload;
	
	data = (uint8_t*)malloc(sizeof(uint8_t) * 4);
	
	if (data == NULL) {
		return;
	}
	
	high_nibble = instruction & 0xF0;
	low_nibble = (instruction & 0x0F) << 4;
	
	data[0] = high_nibble | (opcode << 0) | (1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[1] = high_nibble | (opcode << 0) & ~(1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[2] = low_nibble  | (opcode << 0) | (1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	data[3] = low_nibble  | (opcode << 0) & ~(1 << ENABLE_LATCH) | (1 << BACKLIGHT_ON);
	
	payload = payload_create_i2c(PRIORITY_NORMAL, i2c_device, data, 4, _hd44780_i2c_request_busy_flag); // _hd44780_i2c_request_busy_flag
	
	if (payload == NULL) {
		return;
	}
	
	i2c_write(payload);
}

static _hd44780_i2c_request_busy_flag(void){

	uint8_t* data;
	uint8_t high_nibble;
	uint8_t low_nibble;
	uint8_t opcode;
	uint8_t instruction;
	payload_t* payload;
	
	data = (uint8_t*)malloc(sizeof(uint8_t) * 2);
	
	if (data == NULL) {
		return;
	}
	
	opcode = 0x00;
	instruction = RWRS10;
	
	high_nibble = instruction & 0xF0;
	low_nibble = (instruction & 0x0F) << 4;
	
	data[0] = high_nibble | (opcode << 0) | (1 << ENABLE_LATCH);
	data[1] = high_nibble | (opcode << 0) & ~(1 << ENABLE_LATCH);
	
	// TODO: Change direction of PCF to input here (?)
	
	payload = payload_create_i2c(PRIORITY_NORMAL, i2c_device, data, 2, _hd44780_i2c_start_check_busy_flag_routine);
	
	if (payload == NULL) {
		return;
	}
	
	i2c_read(payload);
}

static void _hd44780_i2c_start_check_busy_flag_routine(void) {
	
	// Assume LCD is busy
	g_busy_flag = 0x80; 
	
	// Start Timer
	// TIMSK3 |= (1 << OCIE3A);
	
	// Get Busy Flag
	g_busy_flag = 0x00; // TODO: TWDR & 0x80;
	
	lcd_fsm_state = LCD_STATE_CHECK_BUSY;
}

void hd44780_i2c_set_stream_out(stream_out_t* stream);

void hd44780_i2c_update(stream_out_t* stream);

static void _hd44780_i2c_putc(uint8_t c){
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS01, c));
}

void hd44780_i2c_puts(char* string){
	while(*string){
		_hd44780_i2c_putc(*string++);
	}
}

void hd44780_i2c_clear(void){
	hd44780_i2c_send_8_bit_instruction(RWRS00, CLEAR_DISPLAY);
}

void hd44780_i2_move_cursor(uint8_t line, uint8_t offset){
	hd44780_i2c_send_8_bit_instruction(RWRS00, DDRAM_ADDR + line + offset);
}

void hd44780_i2_shift_cursor_left(void){
	hd44780_i2c_send_8_bit_instruction(RWRS00, SHIFT_INSTRUCTION | SHIFT_CURSOR_LEFT);
}

void hd44780_i2_shift_cursor_right(void){
	hd44780_i2c_send_8_bit_instruction(RWRS00, SHIFT_INSTRUCTION | SHIFT_CURSOR_RIGHT);
}

void hd44780_i2_shift_display_left(void){
	hd44780_i2c_send_8_bit_instruction(RWRS00, SHIFT_INSTRUCTION | SHIFT_DISPLAY_LEFT);
}

void hd44780_i2_shift_display_right(void){
	hd44780_i2c_send_8_bit_instruction(RWRS00, SHIFT_INSTRUCTION | SHIFT_DISPLAY_RIGHT);
}

void hd44780_i2_shift_display_up(void){
	// TODO:
}

void hd44780_i2_shift_display_down(void){
	// TODO:
}

static void timer1_init(void) {
	// Set Timer1 to CTC mode
	TCCR3B |= (1 << WGM32);
	
	// Set prescaler to 1024
	TCCR3B |= (1 << CS32); // | (1 << CS30);

	// Set compare match value for 1s interval (assuming 10MHz clock)
	OCR3A = 9765;

	// Enable Timer1 Compare Match A interrupt
	TIMSK3 |= (1 << OCIE3A);

	// Enable global interrupts
	// sei();
}

ISR(TIMER3_COMPA_vect) {
	
	switch(lcd_fsm_state) {
		
		case LCD_STATE_IDLE: {
			led_toggle(LED1);
			break;
		}
		
		case LCD_STATE_SEND_COMMAND: {
			led_toggle(LED2);
			break;
		}
		
		case LCD_STATE_CHECK_BUSY: {
			led_toggle(LED3);
			if (g_busy_flag) {
				_hd44780_i2c_request_busy_flag();
			} else {
				
				lcd_fsm_state = LCD_STATE_IDLE;
				
				// TIMSK3 &= ~(1 << OCIE3A); // Stop Timer
				
				_hd44780_send_command();
			}
			
			break;
		}
		
		default: {
			break;
		}
	}
}