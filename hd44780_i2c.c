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
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

/* User defined libraries */
#include "hd44780_i2c.h"
#include "i2c.h"
#include "ringbuffer.h"
#include "utils.h"
#include "memory.h"

#define ENABLE_LATCH 0x02
#define BACKLIGHT_ON 0x03

typedef enum {
	LCD_STATE_IDLE,
	LCD_STATE_CHECK_BUSY,
	LCD_STATE_SEND_COMMAND,
	LCD_STATE_PREPARE_COMMAND
} lcd_fsm_state_t;

volatile static lcd_fsm_state_t lcd_fsm_state = LCD_STATE_IDLE;

static queue_t q;
static queue_t* queue = NULL;
static device_t* _i2c_device = NULL;
static payload_t* _payload = NULL;
static uint8_t _busy_flag = 0x00;
static uint8_t _sensor_index = 0;
static sensor_data_t _sensor_data;
static sensor_data_t* _ptr_sensor_data = NULL; 

static void* _hd44780_i2c_set_to_4bit_operation(void);
static uint8_t* _hd44780_i2c_send_8bit_handler(uint8_t opcode, uint8_t instruction);
static void* _hd44780_i2c_send_8_bit_instruction(uint8_t opcode, uint8_t instruction, callback_fn callback);
static payload_t* _hd44780_prepare_command(void);
static void* _hd44780_send_command(payload_t* payload);
static void* _hd44780_i2c_request_busy_flag(void* context);
static void _hd44780_i2c_update_busy_flag(void* context);
static void _hd44780_i2c_init_sensor_data(sensor_data_t* sensor_data);
static void _hd44780_i2c_timer_init(void);
static void _hd44780_i2c_start_timer(void);
static void _hd44780_i2c_stop_timer(void);

void hd44780_i2c_init(void) {
	
	_delay_ms(100); // Boot Time
	
	_busy_flag = 0x00;
	_ptr_sensor_data = &_sensor_data;
	
	queue = queue_init(&q);
	
	_i2c_device = i2c_create_device(I2C_DEVICE_ADDR);
	
	_hd44780_i2c_init_sensor_data(_ptr_sensor_data);
	
	_hd44780_i2c_timer_init();
		
	// Force 8-bit mode
	_hd44780_i2c_send_8_bit_instruction(RWRS00, 0x30, NULL);
	_delay_ms(5); 
	_hd44780_i2c_send_8_bit_instruction(RWRS00, 0x30, NULL);
	_delay_ms(1);
	_hd44780_i2c_send_8_bit_instruction(RWRS00, 0x30, NULL);
	_delay_ms(1);
	
	// Operates in 4-bit mode from here on
	_hd44780_i2c_set_to_4bit_operation(); 
	_delay_ms(1);
	
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, FUNCTION_SET_4_BIT_MODE_5x8));
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, DISPLAY_OFF));
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, CLEAR_DISPLAY));
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, CURSOR_DIR_LEFT_NO_SHIFT));
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, DISPLAY_ON));
	
	hd44780_i2c_update();	
	
	_hd44780_i2c_start_timer();
}

/*
	In order to set the HD44780 into 4-bit mode we need to send a single 8-bit command with only
	one write operation.

	HD44780 Datasheet Table 12:
	> Sets to 4-bit operation. In this case, operation is handled as 8 bits by initialization, 
	> and only this instruction completes with one write.
*/
static void* _hd44780_i2c_set_to_4bit_operation(void) {
	
	uint8_t* data;
	uint8_t high_nibble;
	payload_t* payload;
	
	data = (uint8_t*)malloc(sizeof(uint8_t) * 2);
	
	if (data == NULL) {
		return NULL;
	}
	
	high_nibble = 0x20 & 0xF0;
	
	data[0] = (high_nibble | (RWRS00 << 0) | (1 << BACKLIGHT_ON)| (1 << ENABLE_LATCH));
	data[1] = (high_nibble | (RWRS00 << 0) | (1 << BACKLIGHT_ON)) & ~(1 << ENABLE_LATCH);

	payload = (payload_t*)payload_create_i2c(PRIORITY_NORMAL, _i2c_device, data, 2, NULL);
	
	if (payload == NULL) {
		free(data);
		return NULL;
	}
	
	free(data);
	
	i2c_write(payload);
	
	return NULL;
}

static uint8_t* _hd44780_i2c_send_8bit_handler(uint8_t opcode, uint8_t instruction) {
	
	uint8_t* data;
	uint8_t high_nibble;
	uint8_t low_nibble;
	
	data = (uint8_t*)malloc(sizeof(uint8_t) * 4);
	
	if (data == NULL) {
		return NULL;
	}
	
	high_nibble = instruction & 0xF0;
	low_nibble = (instruction & 0x0F) << 4;
	
	data[0] = (high_nibble | (opcode << 0) | (1 << BACKLIGHT_ON) | (1 << ENABLE_LATCH));
	data[1] = (high_nibble | (opcode << 0) | (1 << BACKLIGHT_ON)) & ~(1 << ENABLE_LATCH);
	data[2] = (low_nibble  | (opcode << 0) | (1 << BACKLIGHT_ON) | (1 << ENABLE_LATCH));
	data[3] = (low_nibble  | (opcode << 0) | (1 << BACKLIGHT_ON)) & ~(1 << ENABLE_LATCH);
	
	return data;
}

static void* _hd44780_i2c_send_8_bit_instruction(uint8_t opcode, uint8_t instruction, callback_fn callback) {
	
	uint8_t* data;
	payload_t* payload;
	
	data = _hd44780_i2c_send_8bit_handler(opcode, instruction);
	
	if (data == NULL) {
		return NULL;
	}

	payload = (payload_t*)payload_create_i2c(PRIORITY_NORMAL, _i2c_device, data, 4, callback);
	
	if (payload == NULL) {
		free(data);
		return NULL;
	}
	
	free(data);
	
	i2c_write(payload);
	
	return NULL;
}

static payload_t* _hd44780_prepare_command(void) {
	
	payload_t* payload;
	
	payload = queue_dequeue(queue);
	
	if (payload == NULL) {
		return NULL;
	}
		
	return payload;
}

static void* _hd44780_send_command(payload_t* payload) {
	
	uint8_t opcode;
	uint8_t instruction;
	
	opcode = payload->hd44780.opcode;
	
	instruction = payload->hd44780.instruction;
	
	payload_free_hd44780(payload);
	
	_hd44780_i2c_send_8_bit_instruction(opcode, instruction, NULL); // skip read process => _hd44780_i2c_start_check_busy_flag_routine ; _hd44780_i2c_request_busy_flag
	
	return NULL;
}

static void* _hd44780_i2c_request_busy_flag(void* context) {

	uint8_t* data;
	payload_t* payload;
	
	data = _hd44780_i2c_send_8bit_handler(0xFF, RWRS10);
	
	if (data == NULL) {
		return NULL;
	}
	
	payload = payload_create_i2c(PRIORITY_NORMAL, _i2c_device, data, 4, _hd44780_i2c_update_busy_flag);
	
	if (payload == NULL) {
		free(data);
		return NULL;
	}
	
	free(data);
	
	i2c_read(payload);
	
	return NULL;
}

static void _hd44780_i2c_update_busy_flag(void* context) {
	
	// TODO: Get Busy Flag, TWDR & 0x80, LCD always responds with 0x80 (Busy)
	_busy_flag = 0x00; 
	
	lcd_fsm_state = LCD_STATE_CHECK_BUSY;
}

static void _hd44780_i2c_init_sensor_data(sensor_data_t* sensor_data) {
	
	strcpy(sensor_data->sensors[TEMPERATURE_SENSOR].name, "Temperature");
	strcpy(sensor_data->sensors[HUMIDITY_SENSOR].name, "Humidity");
	strcpy(sensor_data->sensors[PRESSURE_SENSOR].name, "Pressure");
	strcpy(sensor_data->sensors[CO2_SENSOR].name, "CO2");
	strcpy(sensor_data->sensors[LIGHT_SENSOR].name, "Light"); 
	strcpy(sensor_data->sensors[SOUND_SENSOR].name, "Sound"); 
	
	strcpy(sensor_data->sensors[TEMPERATURE_SENSOR].unit, "C");
	strcpy(sensor_data->sensors[HUMIDITY_SENSOR].unit, "%");
	strcpy(sensor_data->sensors[PRESSURE_SENSOR].unit, "Pa");
	strcpy(sensor_data->sensors[CO2_SENSOR].unit, "ppm");
	strcpy(sensor_data->sensors[LIGHT_SENSOR].unit, "lx");
	strcpy(sensor_data->sensors[SOUND_SENSOR].unit, "dB");

	// Example sensor values
	sensor_data->sensors[TEMPERATURE_SENSOR].value = -22.05;
	sensor_data->sensors[HUMIDITY_SENSOR].value = 55.31;
	sensor_data->sensors[PRESSURE_SENSOR].value = 1031.2;
	sensor_data->sensors[CO2_SENSOR].value = 400.0;
	sensor_data->sensors[LIGHT_SENSOR].value = 150.0;
	sensor_data->sensors[SOUND_SENSOR].value = 65.2;
}

void hd44780_i2c_set_sensor_data(sensor_index_t sensor, float value) {
	_sensor_data.sensors[sensor].value = value;
}

void hd44780_i2c_update(void) {
	char buffer[MAX_DISPLAY_CHAR_LENGTH + 1]; // +1 for null terminator
	char name_buffer[MAX_DISPLAY_CHAR_LENGTH + 1];
	char unit_buffer[MAX_DISPLAY_CHAR_LENGTH + 1];
	char value_buffer[MAX_DISPLAY_CHAR_LENGTH + 1];
	
	hd44780_i2c_clear();

	for (uint8_t sensor_index = _sensor_index, line = LINE1; 
		 sensor_index < _sensor_index + LINE_COUNT; 
		 sensor_index++, line += 0x40) {
		
		if (sensor_index < SENSOR_COUNT) {
			// Convert float to string with 2 decimal places
			size_t value_size = float_to_string(_ptr_sensor_data->sensors[sensor_index].value, value_buffer, sizeof(value_buffer), 2);
			
			// Calculate max allowed length for name
			size_t unit_size = strlen(_ptr_sensor_data->sensors[sensor_index].unit);
			size_t available_name_length =  MAX_DISPLAY_CHAR_LENGTH - (value_size + unit_size + 2); // +2 accounts for the colon(:) and space() in the final format
			
			snprintf(unit_buffer, sizeof(unit_buffer), "%s", _ptr_sensor_data->sensors[sensor_index].unit);
			snprintf(name_buffer, available_name_length + 1, "%s", _ptr_sensor_data->sensors[sensor_index].name);
			snprintf(buffer, sizeof(buffer), "%s:%s %s", name_buffer, value_buffer, unit_buffer);
			
			hd44780_i2_move_cursor(line, 0);
			hd44780_i2c_puts(buffer);
		}
	}
}

static void _hd44780_i2c_putc(char c) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS01, c));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_puts(char* string) {
	
	while(*string) {
		_hd44780_i2c_putc(*string++);
	}
}

void hd44780_i2c_clear(void) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, CLEAR_DISPLAY));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_move_cursor(uint8_t line, uint8_t offset) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, DDRAM_ADDR + line + offset));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_shift_cursor_left(void) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, SHIFT_INSTRUCTION | SHIFT_CURSOR_LEFT));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_shift_cursor_right(void) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, SHIFT_INSTRUCTION | SHIFT_CURSOR_RIGHT));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_shift_display_left(void) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, SHIFT_INSTRUCTION | SHIFT_DISPLAY_LEFT));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_shift_display_right(void) {
	queue_enqueue(queue, payload_create_hd44780(PRIORITY_NORMAL, RWRS00, SHIFT_INSTRUCTION | SHIFT_DISPLAY_RIGHT));
	_hd44780_i2c_start_timer();
}

void hd44780_i2c_shift_display_up(void) {
		
	if (_sensor_index == 0) {
		return;
	}
	
	_sensor_index -= 1;
	
	if (_sensor_index < 0) {
		_sensor_index = 0;
	}
	
	hd44780_i2c_update();
}

void hd44780_i2c_shift_display_down(void) {
		
	if (_sensor_index == SENSOR_COUNT - LINE_COUNT) {
		return;
	}
	
	_sensor_index += 1;
	
	if (_sensor_index > SENSOR_COUNT) {
		_sensor_index = SENSOR_COUNT;
	}
	
	hd44780_i2c_update();
}

static void _hd44780_i2c_timer_init(void) {
	
	// Set Timer1 to CTC mode
	TCCR3B |= (1 << WGM32);
	
	// Set prescaler to 1024
	TCCR3B |= (1 << CS32) | (1 << CS30);

	// Set compare match value for 200 Hz interval (assuming 10MHz clock)
	OCR3A = 47;
}

static void _hd44780_i2c_start_timer(void) {
	
	// Activate Timer only if not already enabled
	if (TIMSK3 & (1 << OCIE3A)) {
		// Do nothing
	} else {
		// Reset timer counter to 0
		TCNT3 = 0;
		
		// Enable Timer1 Compare Match A interrupt
		TIMSK3 |= (1 << OCIE3A);
	}
}

static void _hd44780_i2c_stop_timer(void) {
	// Disable Timer1 Compare Match A interrupt
	TIMSK3 &= ~(1 << OCIE3A);
}

ISR(TIMER3_COMPA_vect) {
			
	switch(lcd_fsm_state) {
		
		case LCD_STATE_IDLE: {
			
			if (queue_empty(queue)) {
				_hd44780_i2c_stop_timer();
			} else {
				lcd_fsm_state = LCD_STATE_PREPARE_COMMAND;
				
				_hd44780_i2c_start_timer();
			}
			
			break;
		}
		
		case LCD_STATE_PREPARE_COMMAND: {
			
			_hd44780_i2c_stop_timer();
			
			_payload = _hd44780_prepare_command();
				
			if (_payload == NULL) {
				lcd_fsm_state = LCD_STATE_IDLE;
			} else {
				lcd_fsm_state = LCD_STATE_SEND_COMMAND;
				
				_hd44780_i2c_start_timer();
			}		
			
			break;
		}
		
		case LCD_STATE_SEND_COMMAND: {
					
			if (_payload != NULL) {
				_hd44780_send_command(_payload);
			} else {
				
			}
			
			_payload = NULL;
			
			lcd_fsm_state = LCD_STATE_CHECK_BUSY;
			
			break;
		}
		
		case LCD_STATE_CHECK_BUSY: {
			
			lcd_fsm_state = LCD_STATE_PREPARE_COMMAND;
			
			break;
		}
		
		default: {
			break;
		}
	}
}