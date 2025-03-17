# HD44780 I2C Library for ATmega2560

\
*A non-blocking I2C-based driver for HD44780 LCD on ATmega2560*

## Overview

This library provides an interrupt-driven I2C interface to control an HD44780 LCD using an ATmega2560 microcontroller. It eliminates blocking delays by utilizing a finite state machine (FSM) for command execution.

**Communication with the HD44780 LCD is handled via the PCF8574 I/O expander over I2C.**

## Features

- Non-blocking I2C communication
- Interrupt-driven command execution
- FIFO queue for LCD instructions
- Supports 4-bit mode initialization
- Timer-based task management

## Dependencies

Ensure you have the following libraries in your project:

- `i2c.c/h` (I2C driver)
- `ringbuffer.c/h` (Queue management)
- `utils.c/h` (Utility functions)
- `memory.c/h` (Dynamic memory handling)

## Installation

Clone this repository and include the necessary files in your project:

```sh
$ git clone https://github.com/yourusername/hd44780_i2c.git
```

Include the header in your code:

```c
#include "hd44780_i2c.h"
```

## Usage

### Initialize the LCD

```c
hd44780_i2c_init();
```

### Print Text to LCD

```c
hd44780_i2c_puts("Hello, World!");
```

### Move Cursor

```c
hd44780_i2_move_cursor(0, 5); // Move to first line, 6th character
```

### Clear Display

```c
hd44780_i2c_clear();
```

### Shift Display

```c
hd44780_i2_shift_display_left();
```

### Update Sensor Data

```c
hd44780_i2c_set_sensor_data(TEMPERATURE_SENSOR, 25.5);
hd44780_i2c_update();
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
