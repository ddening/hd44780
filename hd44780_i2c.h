/*************************************************************************
* Title		: hd44780_i2c.h
* Author	: Dimitri Dening
* Created	: 08.03.2025 22:07:18
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

#ifndef HD44780_I2C_H_
#define HD44780_I2C_H_

/* General libraries */

/* User defined libraries */
#include "i2c.h"

#define I2C_DEVICE_ADDR 0x27 

/* HD44780 */
#define LINE_COUNT  0x02
#define LINE_LENGTH 0x14
#define LINE1	0x00
#define LINE2	LINE1 + 0x40
#define LINE3	LINE1 + LINE_LENGTH
#define	LINE4 	LINE2 + LINE_LENGTH
#define DDRAM_ADDR 0x80

#define RW_PIN       0x00
#define RS_PIN       0x01
#define ENABLE_PIN   0x02

/* OPCODE */
#define RWRS00 0x00
#define RWRS01 0x01
#define RWRS10 0x02
#define RWRS11 0x03

/* DISPLAY CONTROL */
#define DISPLAY_OFF     0x08
#define DISPLAY_ON      0x0C
#define CURSOR_ON       0x0A 
#define BLINK_ON        0x09  
#define CLEAR_DISPLAY   0x01
#define RETURN_HOME     0x02
#define BUSY_FLAG		0x80

/* FUNCTION SET DEFINITION (EUROPEAN ONLY) */
#define FUNCTION_SET_4_BIT_MODE_5x8    0x28 // 4 Bit Mode, 2 Lines, 5x8 Dots
#define FUNCTION_SET_4_BIT_MODE_5x10   0x2C // 4 Bit Mode, 2 Lines, 5x10 Dots

/* ENTRY MODE SET */
#define CURSOR_DIR_LEFT_SHIFT     0x07
#define CURSOR_DIR_LEFT_NO_SHIFT  0x06
#define CURSOR_DIR_RIGHT_SHIFT    0x05
#define CURSOR_DIR_RIGHT_NO_SHIFT 0x04

/* CHARACTER GRAPHICS MODE W/ INTERNAL POWER */
#define CHARACTER_MODE_INTERNAL_PWR    0x17
#define CHARACTER_MODE_NO_INTERNAL_PWR 0x13
#define GRAPHIC_MODE_NO_INTERNAL_PWR   0x1B
#define GRAPHIC_MODE_INTERNAL_PWR      0x1F

/* Moves cursor and shifts display without changing DDRAM contents */
#define SHIFT_INSTRUCTION   0x10
#define SHIFT_CURSOR_LEFT   0x00
#define SHIFT_CURSOR_RIGHT  0x04
#define SHIFT_DISPLAY_LEFT  0x08
#define SHIFT_DISPLAY_RIGHT 0x0C

/* Typdef for Display Output */
#define MAX_DISPLAY_CHAR_LENGTH 16

// Enum for named sensor indices
typedef enum {
	TEMPERATURE_SENSOR,
	HUMIDITY_SENSOR,
	PRESSURE_SENSOR,
	CO2_SENSOR,
	LIGHT_SENSOR,
	SOUND_SENSOR,
	SENSOR_COUNT // Automatically gives total number of sensors
} sensor_index_t;

typedef struct {
	char name[MAX_DISPLAY_CHAR_LENGTH];
	char unit[MAX_DISPLAY_CHAR_LENGTH];
	float value;
} sensor_t;

typedef struct {
	sensor_t sensors[SENSOR_COUNT]; // Use SENSOR_COUNT from the enum
} sensor_data_t;

void hd44780_i2c_init(void);
void hd44780_i2c_puts(char* string);
void hd44780_i2c_update(void);
void hd44780_i2c_clear(void);
void hd44780_i2c_move_cursor(uint8_t, uint8_t);
void hd44780_i2c_shift_cursor_left(void);
void hd44780_i2c_shift_cursor_right(void);
void hd44780_i2c_shift_display_left(void);
void hd44780_i2c_shift_display_right(void);
void hd44780_i2c_shift_display_up(void);
void hd44780_i2c_shift_display_down(void);
void hd44780_i2c_set_sensor_data(sensor_index_t sensor, float value);

#endif /* HD44780_I2C_H_ */