/*************************************************************************
* Title     : hd44780.c
* Author    : Dimitri Dening
* Created   : 11.04.2023
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
#include <avr/interrupt.h>
#include <util/delay.h>

/* User defined libraries */
#include "hd44780.h"

static void _hd44780_check_busy ( void );
static void _hd44780_check_fake_busy ( void );
static void _hd44780_putc( uint8_t c );

static stream_out_t* stream_out = NULL;
static uint8_t CURRENT_STREAM_LINE = 0x00;

void hd44780_init( uint8_t cs ) {
    
    hd44780_send_8_bit_instruction( FUNCTION_SET_EUROPEAN ); // has to be sent first!
    hd44780_send_8_bit_instruction( DISPLAY_OFF );
    hd44780_send_8_bit_instruction( CURSOR_DIR_LEFT_NO_SHIFT );
    hd44780_send_8_bit_instruction( CHARACTER_MODE_INTERNAL_PWR );
    hd44780_send_8_bit_instruction( CLEAR_DISPLAY ); 
    hd44780_send_8_bit_instruction( RETURN_HOME );
    hd44780_send_8_bit_instruction( DISPLAY_ON | CURSOR_ON | BLINK_ON );  
}

void hd44780_send_8_bit_instruction( uint8_t instruction) {
	
}

void hd44780_send_4_bit_instruction( uint8_t instruction) {
	
}

// TODO: Analyze real response signal from device. Use check_fake_busy() meanwhile.
static void _hd44780_check_busy ( void ) {
    
}

static void _hd44780_check_fake_busy ( void ) {
    _delay_ms(50); // Fake Busy Response
}

static void _hd44780_putc( uint8_t c ) {
    _hd44780_check_fake_busy();
    _hd44780_send_fake_10_bit_instruction( RSRW10, c );
}

void hd44780_puts( char* string ) {
    while( *string ) {
        _hd44780_putc( *string++ );
    }
}

void hd44780_put_stream( char* stream ) {
    _hd44780_check_fake_busy();
}

void hd44780_clear( void ) {
    hd44780_send_8_bit_instruction( RSRW00, CLEAR_DISPLAY );
}

void hd44780_update( stream_out_t* stream ) {
	
}

void hd44780_set_stream_out( stream_out_t* stream ) {
    stream_out = stream;
}