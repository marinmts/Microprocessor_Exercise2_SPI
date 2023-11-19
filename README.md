# Microprocessor_Exercise2_SPI
This repository contain 3 files :
  - Exercise2_SPI.sim1 is the simulation circuit using SimulIDE
  - Master_main.asm is the program to be load (after it built with Microchip Studio, .hex file) in the MCU mega324-1_Master
  - Slave_main.asm is the program to be load (after it built with Microchip Studio, .hex file) in the MCU mega324-2_Slave

A simulation has already done and recorded.
Record link : https://youtu.be/G6VPbCtdsUY

Comments :
  Wrong receiving of the odd number. 
  Transmitting has already checked with the LCD and BARLED connected to the Slave. 
  The value received by the Master is immidiately display on the BARLED. Here we see that for odd number, the Master receive the odd number with the MSB set.
  Then the number is convert to ASCII and send to LCD and UART0 monitor.
  To solve this issue you can just clear the MSB of the register DATA_TEMP which contain the received value. 
