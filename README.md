4/26/25, I2C Test Program:
BMS_I2C -> Test program for Master Nucleo
BMS_I2C_Test -> Test program for Slave Nucleo

Description: I2C ought to work and transmit "Hello, World" message to slave

4/30/25, I2C BMS integration:
Removed BMS_I2C test program for master NUCLEO and replaced it with latest BSM_I2C integration code.
What works:
-I2C communication can be viewed on logic analyzer
-Numbers can be viewed in debug
What doesn't work:
-Values are not correct
-They are not be updating consistently (or at all)
