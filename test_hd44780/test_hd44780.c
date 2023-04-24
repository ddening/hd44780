#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

#include "hd44780.h"
#include "suite.h"
#include "uart.h"
#include "led_lib.h"
#include "switches.h"

/* Define CPU frequency in Hz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 10000000UL
#endif

static stream_out_t stream_out;

int main( void ) {
    
    cli(); 
         
    led_init();
    uart_init();
    switch_init();
     
    sei();
    
    hd44780_init();  
    // hd44780_move_cursor(LINE1, 0);
    // hd44780_puts("Hello AVR");
    
    /* Fill Test Stream With Data */
    //sprintf(stream_out.data0, "SENSOR00");
    //sprintf(stream_out.data1, "SENSOR01");
    //sprintf(stream_out.data2, "SENSOR02");
    //sprintf(stream_out.data3, "SENSOR03");
    //sprintf(stream_out.data4, "SENSOR04");
    //sprintf(stream_out.data5, "SENSOR05");
            
   while (1) { /* Busy-wait forever. */ }
}