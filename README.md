# Brake actuator

This project aims to test both an incremental magnetic encoder and a stepper motor using the STM32 Nucleo F446RE board.

## Encoder
For the encoder, only the A and B pins are needed, in addition to ground and VCC.

- Channel A (pin 3) is connected to pin PA0.

- Channel B (pin 5) is connected to pin PA1.

- The I pin can be used for detecting a complete revolution but is not required for the basic functionality.

In this implementation, a signed counter is used to allow for both positive and negative values, unlike the standard unsigned counter typically used with HAL.

## Step motor
To control the stepper motor, a driver requiring only 3 pins is used:
- Enable as pin PC0
- Step as pin PC1
- Dir as pin PC2

To move the motor, another timer is used, allowing for control of the stepper motor’s movements velocity.

At the moment each step should take 1ms meaning that to move of 1cm it is required 1 second. This movement is slow but can be modified whenever we are sure that the actual implementation works.