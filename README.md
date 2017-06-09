# BNO080
Arduino sketch for BNO080 9DOF with sensor fusion
The BNO080 by http://hillcrestlabs.com/ is a successor of the BNO055 from Bosch Sensortec www.bosch-sensortec.com. It offers more features and a significantly improved performance however with a quite different interface. Breakout boards may appear soon on the marked. Hillcrestlabs has issued a code using a ST Nucleo eval board. https://github.com/hcrest/bno080-nucleo-demo.
As the BNO080 generates high performance quaternions, linear acceleration etc: at an extraordinary high data rate (up to 400Hz). Therefore  the device is best suited for robotics and other IMU applications where  is desirable to use also Atmega 328 controllers like Arduino Nano and others.
This sketch provides a basic functionality for testing and further developments. 
