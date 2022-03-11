# HomeOilTankLevel

I protyped this a couple of months ago so this is rough around the edges.

If you are interested in using or doing a project such as this you need to check if your oil tank float is magnetic.  The concept for this occured to me, because the Yellow float indicator, inside the sight glass had some metal filings stuck to it.

I used ethernet jacks and Ethernet cable as my 5VDC wall wart and ESP32 is located near a 120V AC outlet and 20' away is my oil tank where I located the motor driver board.  See the "Wiring Cartoon" I uploaded as reference.

The ESP32 code reports up to mqtt and I use Home assitant to visualize the tank level.

Things to watch out for or I would have done differently:

#1 I would mount the limit switches on the left rather than the right.  If you look at the picture you will see that I had to trim the U-bolt that I use to mount the system to the pipe - the limit switch dog would have hit the stub of the U-bolt.

#2 The Hall effect sensor is polarity sensitive (North - South).  If you are having issue picking up the magnetism of your float - try moving the sensor to the other side of the sight glass.

#3  I used plastic surface mount device boxes to mount the electronics in.  It was tight fitting the componets into the boxes.  I'm sure that a slightly larger 3d printed box would be a better solution.

#4 Don't try to power the ESP32 with 5VDC.  I did this and it worked for about 5 minutes before the magic smoke came out.  I used a 3.3V regulator to interface the 5V to power the ESP32.  You could also try to power the entire system from a 3.3V wall wart, which will probably work

#5 I used a transistor to interface the Hall effect sensor into the ESP32 as an input.  I don't know why but I couldn't get the Hall effect sensor to work as a direct input into the ESP32

Sequence of Operation:

1.) On Power up the Motor strokes the actuator in the up position until the upper limit switch is detected.  Encoder Counts are set to Zero.
2.) Motor direction is switched and the actuator is stroked in the down direction until the hall effect sensor senses the float.
3.) If the actuator senses the lower switch the motor will be turned off.  This can happen if the hall effect sensor misses the float or when the tank is filled.  Cycle power to Re-home the system.

Lines you will have to modify in the Arduino code:

Line 15 your 2G wifi SSID
Line 16 your wifi SSID password

Line 19 your mqtt IP Address

Line 25 your mqtt topic

Line 127 change to reflect your mqtt credentials

Line 139 modify the number 166646 to reflect your distance (counts) from the top home switch to the top of your float
Line 139 modify the number 71283 to reflect the distance (counts) to the bottom of your float
