# Brake actuator

This project aims to test both a linear actuator and an incremental magnetic encoder using the STM32 Nucleo F446RE board.

## Components

- [LGA561S20-B-TSCA-019 –  Captive linear actuator – NEMA 23](https://www.nanotec.com/us/en/products/8546-lga561s20-b-tsca-019)

- [NME2-UVW-U15-05-O –  High-resolution magnetic encoder](https://www.nanotec.com/us/en/products/8482-nme2-uvw-u15-05-o)

- [A4988 stepper motor driver](https://www.pololu.com/file/0j450/a4988_dmos_microstepping_driver_with_translator.pdf)

- [STM32 Nucleo F446RE](https://www.st.com/en/evaluation-tools/nucleo-f446re.html)

## Linear actuator
The actuator is controlled through the A4988 driver, which requires only two main control signals from the STM32:
- Step to PC1

- Dir to PC2

Other relevant pin connections:
- Enable to GND or PC0

- Sleep to VCC or PB0

- Reset to VCC or PC3

where VCC can be either 3.3V or 5V.

All microstepping (MSX) pins are ignored for now, as the actuator is operated in full-step mode. Other necessary driver connections should follow the A4988 datasheet.

### Current implementation
The actuator performs a simple back-and-forth movement:
- It starts from an initial position $h$, extends to $h + l$ (determined by the number of steps), and returns to $h$.

- The movement is handled through a loop that toggles the STEP and DIR pin.

Note: The DIR pin appears to require more current than the STM32 can supply directly. A MOSFET has been used to ensure proper operation; otherwise, the pin remains low. Possibly the driver pin is not behaving correclty.

## Encoder
Currently, only channels A and B of the encoder are used on the board:

- Channel A is encoder pin 3.

- Channel B is encoder pin 5.

Power and ground must also be connected.\
In the current implementation, we only read the encoder counter without using it for distance measurements.