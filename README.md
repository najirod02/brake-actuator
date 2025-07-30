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
The actuator will move accordinlgy to the pid output which is based on the target pressure and the pressure value read.\
Such values are obtained through the UART in particular, from the spare pins PC12 and PD2 of the ECU board.

Note: The DIR pin appears to require more current than the STM32 can supply directly. A MOSFET has been used to ensure proper operation; otherwise, the pin remains low. Possibly the driver pin is not behaving correclty.

Note: The actuator requires 2A for each winding. As the driver is only able to deliver 2A, high frequencies are not supported precisely, the maximum achieved was 1kHz.

### UART commands
As briefly explained before, the ECU can command the actuator by sending some speciifc command to the nucleo board that will manage the enabling/disabling and pwm generation. In particular:
- CX: Allows to enable or disable the actuator in more details:
    - C1: to ENABLE
    - C2: to DISABLE

- SX: Allows to set a new target point (a pressure value). For example:
    - S9: will set the target point at 9 bar.
    - Saa: will generate an error which ultimately will disable the actuator.

- Px: Allows to update the latest pressure value read from the car (a pressure value). For example:
    - S3.34: will set the pressure at 3.34 bar.
    - Saa: will generate an error which ultimately will disable the actuator.

In case of errors during the convertion of float value and/or unrecognized commands, for security reasons, the actuator will be disabled so that we can preserve the integrity of it.

That means, for example, if after sending a 'S8', 'C1' (this means set as target value 8 bar and enabling the actuator), we send a 'Pdd2', 'P1!3e' or 'E1.23' we will consider it as a bad command and so, stop the actuator.

To activate again the actuator, it will be needed to send again a 'C1'. The target value won't be changed in any case unless an explicit 'SX' command is sent.

## Encoder
Unfortunately, we cannot implement any boundary control over the actuator movement as the encoder is not available.