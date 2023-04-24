/*************************************************************************
* Title     : hd44780.c
* Author    : Dimitri Dening
* Created   : 24.04.2023
* Software  : Microchip Studio V7
* Hardware  : Atmega2560, HD44780
        
DESCRIPTION:
    HD44780 Display Driver.
USAGE:
    see <hd44780.h>
NOTES:
                       
*************************************************************************/

/* General libraries */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

/* User defined libraries */
#include "hd44780.h"

static void _hd44780_check_busy ( void );
static void _hd44780_check_fake_busy ( void );
static void _hd44780_putc( uint8_t c );

static stream_out_t* stream_out = NULL;
static uint8_t CURRENT_STREAM_LINE = 0x00;

void hd44780_init() {
    
    HD44780_DDR = 0xFF;     // Configure PORT as OUTPUT
    HD44780_PORT = 0x00;    // Set all PINS LOW
    
    _hd44780_check_fake_busy();
    hd44780_send_8_bit_instruction( RSRW00, 0x30 ); // Enable 4 Bit Mode
    _hd44780_check_fake_busy();
    hd44780_send_8_bit_instruction( RSRW00, 0x30 ); // Enable 4 Bit Mode
    _hd44780_check_fake_busy();
    hd44780_send_8_bit_instruction( RSRW00, 0x30 ); // Enable 4 Bit Mode
    
    _hd44780_check_fake_busy();
    hd44780_send_8_bit_instruction( RSRW00, FUNCTION_SET_4_BIT_MODE ); // Enable 4 Bit Mode
    _hd44780_check_fake_busy();
    hd44780_send_4_bit_instruction( RSRW00, FUNCTION_SET_4_BIT_MODE ); // Define Function Set
    hd44780_send_4_bit_instruction( RSRW00, DISPLAY_ON | CURSOR_ON | BLINK_ON );
    hd44780_send_4_bit_instruction( RSRW00, CURSOR_DIR_LEFT_NO_SHIFT );
    hd44780_send_4_bit_instruction( RSRW11, WRITE_TEST_CHAR );
}

void hd44780_send_8_bit_instruction( uint8_t opcode, uint8_t instruction) {
    HD44780_PORT = instruction| opcode;
    HD44780_PORT |= ( 1 << ENABLE_PIN );              
    _delay_us(1);
    HD44780_PORT &= ~( 1 << ENABLE_PIN );              
}

void hd44780_send_4_bit_instruction( uint8_t opcode, uint8_t instruction) {
	
    /* Data Package Layout: D7 D6 D5 D4 X E RW RS, X:= Don't Care */  
    
    HD44780_PORT = (instruction & 0xF0) | opcode;         // Upper Byte
    HD44780_PORT |= ( 1 << ENABLE_PIN );                  // Enable ON
    _delay_us(1);
    
    HD44780_PORT = ((instruction & 0x0F) << 4) | opcode ; // Lower Byte
    HD44780_PORT &= ~( 1 << ENABLE_PIN );                 // Enable OFF
    _delay_us(1);
    
    _hd44780_check_fake_busy();
}

// TODO: Analyze real response signal from device. Use check_fake_busy() meanwhile.
static void _hd44780_check_busy ( void ) {
    
}

static void _hd44780_check_fake_busy ( void ) {
    _delay_ms(50); // Fake Busy Response
}

static void _hd44780_putc( uint8_t c ) {
    _hd44780_check_fake_busy();
}

void hd44780_puts( char* string ) {
    while( *string ) {
        _hd44780_putc( *string++ );
    }
}

void hd44780_put_stream( char* stream ) {
}

void hd44780_clear( void ) {
}

void hd44780_update( stream_out_t* stream ) {
	
}

void hd44780_set_stream_out( stream_out_t* stream ) {
    stream_out = stream;
}