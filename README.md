# ICM20948
An Arduino library for the ICM-20948 9-axis accelerometer, gyroscope and magnetometer. It contains many example sketches with lots of comments to make it easy to use. It works with I2C.

Fork of: https://github.com/wollewald/ICM20948_WE

DMP features are not implemented. 

If you find bugs please inform me. If you like the library it would be great if you could give it a star.

If you are not familiar with the ICM20948 I recommend to work through the example sketches.

When you wire the ICM-20948 you need to consider that VDD is 3.3 volts, but VDDIO is only 1.71-1.95 volts (see data sheet). For a 5V MCU board, I used a level shifter and additional resistors to GND.

Known issue:
* If you upload sketches, the magnetometer occasionally does not respond. If you disconnect from power and then reconnect it will work. I experienced the issue only after uploads.
