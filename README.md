Keywords: Permanent Magnet Synchronous Motor(PMSM); Field Oriented Control(FOC); PI Speed Control

Hardware:

MCU: STM32F767NIHx

Current Sensor: CKSR-15NP, use ADC to read the data

Position/Speed Sensor: TS2620N21E11(resolver)+AD2S1210, use SPI to read the data

Control Integrated Power System: IKCM15H60GA

Monitor: TLV5630, use SPI to write data(variables), DACs creat real-time waveform of the monitored variables(for debug)

Communication: UART, to receive command (to be continue...)


Software:

SVPWM, please see: ...\Src\Controller.c

PI Control, including current loop and speed loop, please see: ...\Src\Controller.c



