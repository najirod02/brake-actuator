# Brake actuator

This project aims to test both an incremental magnetic encoder and a stepper motor using the STM32 Nucleo F446RE board.

In more detail, the components used are:

- [NME2-UVW-U15-05-O –  High-resolution magnetic encoder](https://www.nanotec.com/us/en/products/8482-nme2-uvw-u15-05-o)

- [LGA561S20-B-TSCA-019 –  Captive linear actuator – NEMA 23](https://www.nanotec.com/us/en/products/8546-lga561s20-b-tsca-019)

- [STM32 Nucleo F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)

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

At the moment on each second, 8 pulses are generated. With this configuration we expect the motor to move at a sustained speed that allows to visually see if it works.