<img src="docs/front.jpg" height = "700">
<img src="docs/back.jpg" height = "700">


The goal for the flight controller is to

1. Measure acceleration, rotation and altitude
1. Get rocket location
1. Store all measurements on-board
1. Real-time telemetry
1. Ignite explosive charges
1. Record video

So far it has been used on [rocket1](https://github.com/klownfish/rocket1) and my W.I.P guided rocket.

### Sensors

The altitude is calculated from the pressure using a BMP280 pressure sensor. The accuracy is about +-1m which is good enough for determining the apogee but acceleration and velocity is pretty much completely lost in the noise. Acceleration and rotation is gathered from a MPU9250 which sucks hard unfortunately. The gyroscope drifts rapidly and so there is no way to get the an accurate attitude estimate. Both of these sensors are available on a GY-91 breakout board.

A Ublox M8 GPS from Team Blacksheep measured the location. The Ublox chip is easy to configure and use but the module I chose does not have any moutning holes…

### Telemetry
All data is stored on a Winond NOR flash IC and it does what it’s supposed to do. I wanted to use a RF96 LoRa modem but I mixed up the numbers ordered a RF69 generic GFSK modem. It works alright so it didn’t really matter in the end. I have not conducted a real range test but the “GFSK_Rb38_4Fd76_8” profile worked for my 30m launches.

### Pyro channels
The pyro channels consist of half H-bridges connected to GND and 12V. A half bridge is necessary to keep the channel shorted when not in use. A TVS diode is connected in parallel as well to protect from electrostatic discarges. BTN7030-1EPA is an awesome “smart” half bridge IC and powers all the channels. It has built in short circuit and overheat protection. It can source 14 Amps of currents which is on the low side for what I want to do. Running of a 3s LiPo battery at 12.6v it can power a 1.1 Ohm load which is very close to the resistance of an ignitor. From empirical data it has worked four out four times so far.

As an additional safety measure a jumper can overwrite the logic output from the micro controller and force all enable pins low. Two LEDs also show if the channels are armed or not.

### Video
A runcam split 4 recorded the onboard footage and I can not recommend it. The auto record function just straight up does not work so it doesn’t record on start up. As a result you have to push a button to start the recording manually and good luck doing that when it’s mounted inside a nose cone. Good thing the camera only cost 80€ and was destroyed in a crash. I hate that camera so fucking much I am so mad AAAAAAHHH

### Guidance
This flight controller is currently the base for a guided rocket that i am developing. It's possible to connect a BMI088 IMU to the expansion port which is much more accurate than the MPU9250. The UART pins for the GPS can also be used to control 2 servos. The servos are connected to a pyro channel with an external voltage regulator.

### Dashboard
The dashboard is written in python using TK and Matplotlib.