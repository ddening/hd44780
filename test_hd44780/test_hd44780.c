#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>

#include "hd44780_i2c.h"
#include "suite.h"
#include "uart.h"
#include "led_lib.h"
#include "switches.h"

/* Define CPU frequency in Hz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 10000000UL
#endif

void test_hd44780(void) {
    
    cli(); 
         
    led_init();
    uart_init();
    switch_init();
     
    sei();
}