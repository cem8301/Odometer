## Odometer

**Inspiration:** Microavionics class final project

**Purpose:** Code to run a bike odometer from a PIC18F87K22 with a hall effect and pulse sensor

**Background:** Bike odometers are used to track a cyclist's distance and performance. Classic bike odometers work by using a hall effect sensor and a magnet to count wheel rotations. This in turn is used to calculate rider stats (current speed, average speed, ride time, and total distance). To extend the project, a pulse sensor is also used to monitor the user's heart rate. A timer will be used to calculate total ride time and an LCD screen will be used to display the data. Only one piece of data will be displayed at a time. The user can toggle through the information by hitting a push button on the PIC controller board. Finally three LED's on the board will be used to notify the user if their speed is at, above, or below their average speed. If the top LED is lit, the used is above their average speed, if the middle LED is lit, they are at their average speed, and if the lowest LED is lit, they are below their average speed. 

**Sensors:** Sparkfun Pulse Sensor (Sen 11574), Sparkfun Hall Latching Effect Sensor (US1881)
