# CompassBox

This is a box which I made as a wedding present. When functioning it points back to where the couple exchanged their vows. 

I originally got the idea from http://hackaday.com/2016/11/05/personal-compass-points-to-your-spawn-point/. The one aspect I didn't like about the original design was the use of the servo motor and meant it could not turn continually. 

I opted for the use of a stepper motor and integrate a zero location sensor which is a opto sensor. 

4 PCBs in the box under the false bottom:
 - STM32 Nucleo board
 - I2C Magnetometer 
 - GPS
 - Motor driver

There is also a microswitch which controls the power to the entire system. 
