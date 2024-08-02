# Pico Ethanol Content Analyzer - Raspberry Pi Pico/RP2040

This will read the frequency(Ethanol %) and low pulse time(C Temp) of a standard GM Ethanol content sensor, print it to USB serial, and then transmit a CANBUS message. CANBUS message format can be set to whatever. Default is set to mirror the Zeitronix format.

Uses the PIO for CANBUS, using this library: https://github.com/KevinOConnor/can2040

![Wiring Diagram.png](https://github.com/Sixline/PicoECA/blob/31fcc2172cd30a571ddddda284e793472f792b68/Wiring%20Diagram.png)

Tested with:
* VDO / CONTINENTAL SE1004S or GM 13577429. Connector for the sensor is a Delphi 13511132. Cheap connectors on AliExpress work great.
* Texas Instruments SN65HVD230 CANBUS transceiver. Again, cheap breakout boards on AliExpress. Depending on your application you may not may not need a 120 Ohm termination resistor. The boards I bought from AliExpress included them.
* QS-1205CME-3A buck converter, from AliExpress, to step down the vehicle 12V to 5V needed for the Pico.

Total cost was around $100.

![Completed Module.png](https://github.com/Sixline/PicoECA/blob/31fcc2172cd30a571ddddda284e793472f792b68/Completed%20Module.png)

```
Output Ethanol Content: % (Frequency Hz - 50)
Fuel temperature °C: [41.5 x low pulsewidth (ms)] - 81.25
Self diagnostics: Embedded (fuel contamination more likely as readings increase above 150Hz)
```
