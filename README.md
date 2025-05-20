# Brake actuator

This project aims to test both a linear actuator and an incremental magnetic encoder using the STM32 Nucleo F446RE board to precisley move the screw 
to a given position.

## Components

- [LGA561S20-B-TSCA-019 –  Captive linear actuator – NEMA 23](https://www.nanotec.com/us/en/products/8546-lga561s20-b-tsca-019)

    - [Product specification - article number](https://www.nanotec.com/eu/en/knowledge-base-article/captive-linear-actuators)

- [NME2-UVW-U15-05-O –  High-resolution magnetic encoder](https://www.nanotec.com/us/en/products/8482-nme2-uvw-u15-05-o)

- [A4988 stepper motor driver](https://www.pololu.com/file/0j450/a4988_dmos_microstepping_driver_with_translator.pdf)

- [STM32 Nucleo F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)

## Linear actuator
The actuator is controlled through the A4988 driver, which requires only two main control signals from the STM32:
- Step to PA6

- Dir to PC2

Other relevant pin connections:
- Enable to GND or PC0

- Sleep to VCC or PB0

- Reset to VCC or PC3

where VCC can be either 3.3V or 5V.

All microstepping (MSX) pins are ignored for now, as the actuator is operated in full-step mode. Other necessary driver connections should follow the A4988 datasheet.

### Current implementation
The actuator will simply reach a defined final position. The velocity will be updated at constant time
by a pid controller.

Note: The DIR pin appears to require more current than the STM32 can supply directly. A MOSFET has been used to ensure proper operation; otherwise, the pin remains low. Possibly the driver pin is not behaving correclty.

Note: The actuator requires 2A for each winding. As the driver is only able to deliver 2A, high frequencies are not supported precisely, the maximum achieved was 1kHz.

## Encoder
Currently, only channels A and B of the encoder are used on the board:

- Channel A is encoder pin 3 set to PA0.

- Channel B is encoder pin 5 set to PA1.

Power and ground must also be connected.\

In the current implementation we read the encoder counter to know how much the actuator moved and use such
value as feedback for the pid.