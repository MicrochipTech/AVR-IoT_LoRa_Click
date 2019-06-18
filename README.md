AN3076 Adding LoRa® RN2483 Click to AVR-IoT WG Board
===

This repository contains the code associated with AN3076 Adding LoRa® RN2483 Click to AVR-IoT WG Board Application Note.

This document describes an example application that uses basic LoRaWAN™ operations to transmit data
from the sensors of the AVR-IoT WG Board. It was developed for the Microchip LoRa® Technology
RN2483 modules on MikroElektronika LoRa click boards.

Every minute, the application sends light intensity and temperature data to the The Things Network (TTN)
server. When the LoRaWAN node is not transmitting, the MCU core and the RN module are both put into
a Sleep state. The source code for the AVR-IoT WG Board and the LoRa click was configured and
generated using Atmel START.
