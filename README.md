# MPU9250_WE
An Arduino library for the 9-axis accelerometer, gyroscope and magnetometer MPU9250 and the MPU6500. In essence the MPU6500 is a MPU9250 without the magnetometer.

The library contains many example sketches with lots of comments to make it easy to use. I have written them for the MPU9250 / I2C. You can "translate" them easily for the MPU6500. MPU6500_all_data.ino shows how to do this. The use of SPI is shown in MPU9250_SPI_all_data.ino and MPU6500_SPI_all_data.ino.

For further information visit my blog:

https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1   (German)

https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1    (English) 

If you find bugs please inform me. If you like the library it would be great if you could give it a star. 

If you are not familiar with the MPU9250 I recommend to work through the example sketches in the following order:

1. MPU9250_acceleration_data.ino
2. MPU9250_gyroscope_data.ino
3. MPU9250_calibration.ino
4. MPU9250_reusing_autocalib_data.ino
5. MPU9250_magnetometer_data.ino
6. MPU9250_all_data.ino
7. MPU9250_angles_and_orientation.ino
8. MPU9250_pitch_and_roll.ino
9. MPU9250_data_ready_interrupt_and_cycle.ino
10. MPU9250_wake_on_motion_interrupt.ino
11. MPU9250_FIFO_stop_when_full.ino
12. MPU9250_FIFO_continuous.ino
13. MPU6500_all_data.ino

The sketch MPU9250_blank_all_settings.ino can be used as a basis for your own sketches. It contains all setting options.

<h3>If the MPU9250 / MPU6500 does not respond</h3>

There are various modules with different MPUxxxx ICs out there. Sometimes it's not clearly defined in online-shops what you will really get if you buy an MPU9250/MPU6500 module. It might be an MPU9250, it might be an MPU6500 or it might be something else, although the modules look the same. An indication is the label on the MPUxxxx IC:

![MPU9250](https://user-images.githubusercontent.com/41305162/181456778-d3f69414-2627-445b-82b9-560dbfcbf982.jpg)

The labels I am aware of are:

 - MP92: MPU9250
 - MP65: MPU6500
 - MP651: MPU6515
 
You can also run the example sketch MPU9250_who_am_I.ino to find out which device you have.
 
I am using the "Who I am" registers of the MPU9250, MPU6500 and the magnetometer AK8963 to check if the modules are connected. If you create an MPU9250 object, but, for example, you are actually using an MPU6500, the init functions will return "false". However, the gyroscope and the accelerometer will work, because all related registers are the same. For other variants it might be similar. If the library works although you are using a different MPUxxxx, then just be happy, but you will have to live with the init function returning "false" - or find an alternative library. 
