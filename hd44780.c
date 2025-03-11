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
        
    // Reset Controller by sending 0x30 with different delay intervals inbetween.
    _delay_ms(15);
    hd44780_send_8_bit_instruction( RWRS00, 0x30 );
    _delay_ms(4.1);
    hd44780_send_8_bit_instruction( RWRS00, 0x30 );
    _delay_ms(0.1);
    hd44780_send_8_bit_instruction( RWRS00, 0x30 );
        
    _hd44780_check_fake_busy();
    hd44780_send_8_bit_instruction( RWRS00, FUNCTION_SET_4_BIT_MODE ); // Enable 4 Bit Mode
    _hd44780_check_fake_busy();
    hd44780_send_4_bit_instruction( RWRS00, FUNCTION_SET_4_BIT_MODE ); // Define Function Set
    hd44780_send_4_bit_instruction( RWRS00, DISPLAY_ON | CURSOR_ON | BLINK_ON );
    hd44780_send_4_bit_instruction( RWRS00, CURSOR_DIR_LEFT_NO_SHIFT );
}

void hd44780_send_8_bit_instruction( uint8_t opcode, uint8_t instruction) {
    HD44780_PORT = instruction | opcode;
    HD44780_PORT |= ( 1 << ENABLE_PIN );              
    _delay_us(1);
    HD44780_PORT &= ~( 1 << ENABLE_PIN );              
}

void hd44780_send_4_bit_instruction( uint8_t opcode, uint8_t instruction) {
	
    /* Data Package Layout: D7 D6 D5 D4 X E RW RS, X:= Don't Care */  
    
    HD44780_PORT = (instruction & 0xF0) | opcode;         // Upper Byte
    _delay_us(1);
    HD44780_PORT |= ( 1 << ENABLE_PIN );                  // Enable ON
    _delay_us(1);
    
    HD44780_PORT = ((instruction & 0x0F) << 4) | opcode ; // Lower Byte
    _delay_us(1);
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

void hd44780_move_cursor( uint8_t line, uint8_t offset ) {
    hd44780_send_4_bit_instruction( RWRS00, DDRAM_ADDR + line + offset );
}

void hd44780_shift_cursor_left( void ) {
    hd44780_send_4_bit_instruction( RWRS00, SHIFT_INSTRUCTION | SHIFT_CURSOR_LEFT );
}

void hd44780_shift_cursor_right( void ) {
    hd44780_send_4_bit_instruction( RWRS00, SHIFT_INSTRUCTION | SHIFT_CURSOR_RIGHT );
}

void hd44780_shift_display_left( void ) {
    hd44780_send_4_bit_instruction( RWRS00, SHIFT_INSTRUCTION | SHIFT_DISPLAY_LEFT );
}

void hd44780_shift_display_right( void ) {
    hd44780_send_4_bit_instruction( RWRS00, SHIFT_INSTRUCTION | SHIFT_DISPLAY_RIGHT);
}

void hd44780_shift_display_up( void ) {
    if ( CURRENT_STREAM_LINE != 0 ) {
        CURRENT_STREAM_LINE--;
    }
    
    hd44780_update( stream_out );
}

void hd44780_shift_display_down( void ) {
    if ( CURRENT_STREAM_LINE != (MAX_OUTPUT_STREAMS - LINE_COUNT ) ) {
        CURRENT_STREAM_LINE++;
    }
    
    hd44780_update( stream_out );
}

void hd44780_clear( void ) {
    hd44780_send_4_bit_instruction( RWRS00, CLEAR_DISPLAY );
}

void hd44780_update( stream_out_t* stream ) {
	
    if ( stream == NULL ) {
        return;
    }
    
    if ( CURRENT_STREAM_LINE > MAX_OUTPUT_STREAMS - LINE_COUNT ||
         CURRENT_STREAM_LINE < 0 ) {
        return;
    }
    
    char* _stream[MAX_OUTPUT_STREAMS] = {
        stream->data0,
        stream->data1,
        stream->data2,
        stream->data3,
        stream->data4,
        stream->data5
    };
    
    uint8_t _stream_line[LINE_COUNT] = { LINE1, LINE2 };
    
    uint8_t _CURRENT_STREAM_LINE_ = CURRENT_STREAM_LINE;
    
    // hd44780_clear(); // -> Doesn't work with hd44780_put_stream yet. Produces wrong line output instead of resetting to addr. 0
    
    for (uint8_t line = 0; line < LINE_COUNT; line++) {
        hd44780_move_cursor( _stream_line[line], 0 );
        hd44780_puts( _stream[_CURRENT_STREAM_LINE_] );
        _CURRENT_STREAM_LINE_++;
    }
}

void hd44780_set_stream_out( stream_out_t* stream ) {
    stream_out = stream;
}