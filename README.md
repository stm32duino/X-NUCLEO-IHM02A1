# X-NUCLEO-IHM02A1

The X-NUCLEO-IHM02A1 is a two axis stepper motor driver expansion board based on the L6470.
This sensor uses SPI to communicate. A SPI instance is required to access to the motor driver.
It provides an affordable and easy-to-use solution for low voltage motor control driving for 
stepper motors in your STM32 Nucleo project. The expansion board includes two L6470s, 
a fully-integrated micro stepping motor driver used to control stepper motors by means 
of high-end motion control commands received through SPI. It is capable of driving one 
or two stepper motors when plugged into an STM32 Nucleo board.

## Examples

There is 1 example with the X-NUCLEO-IHM02A1 library.
* X_NUCLEO_IHM02A1_HelloWorld: This application provides a simple example of usage of the X-NUCLEO-IHM02A1 
Stepper Motor Control Expansion Board. It shows how to use two stepper motors connected in daisy chain 
configuration to the board, moving the rotors to specific positions, with given speed values, direction of rotations, etc.

## Documentation

You can find the source files at  
https://github.com/stm32duino/X-NUCLEO-IHM02A1

The L6470 datasheet is available at  
http://www.st.com/content/st_com/en/products/motor-drivers/stepper-motor-drivers/l6470.html