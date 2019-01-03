# BNO080
Arduino sketches for BNO080 9DOF with sensor fusion
The _2 version uses the standard wire library necessary for M0/ M4 etc controllers
The nrf52 version works on Adafruit nrf52 feather and displays data on Adafruit Bluefruit LE app

The BNO080 by http://hillcrestlabs.com/ is a successor of the BNO055 from Bosch Sensortec www.bosch-sensortec.com. It offers more features and a significantly improved performance however with a quite different interface. 
Breakout boards are now also available http://hillcrestlabs.com/store/fsm300/ . Hillcrestlabs has issued a code using a ST Nucleo eval board. https://github.com/hcrest/bno080-nucleo-demo.
There is also a universal breakout distributed by Sparkfun https://www.sparkfun.com/products/14686.

The BNO080 generates high performance quaternions, linear acceleration etc: at an extraordinary high data rate (up to 400Hz). Therefore  the device is best suited for robotics and other IMU applications where  is desirable to use also Atmega 328 controllers like Arduino Nano and others.

This sketch provides a basic functionality for testing and further developments. 
The BNO080 provides more robust quaternions than the BNO055 and with improved accuracy. The  calibration algorithm can be controlled and results can be stored easily in the flash. The tare function is very useful in  case of relative movements.

Note that for a correct heading  a proper magnetometer function is needed. Thus the sensor should be >1m away from iron masses (a vehicle´s roof). There is real time heading estimation accuracy reporting available.

It is recommended to degauss the board at least once to ensure that the sensor and adjacent components (crystal, resistors , capacitors) does not have a significant remanent magnetic field. 

Degaussing can be done by a diminishing AC magentic field. You may use the coil of a magnetic valve for that purpose. Use such a coil with iron core otherwise its getting very hot. (Be aware of working with dangerous voltages!)  switch on the coil at a distance (> 0.5m) , then move over the breakout, move away  again and switch the coil off.)
https://en.wikipedia.org/wiki/Degaussing

You may do this also with screw drivers, tweezers etc  - old wisdom from  magnetic tape recorders

