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
4. MPU9250_magnetometer_data.ino
5. MPU9250_all_data.ino
6. MPU9250_angles_and_orientation.ino
7. MPU9250_pitch_and_roll.ino
8. MPU9250_data_ready_interrupt_and_cycle.ino
9. MPU9250_wake_on_motion_interrupt.ino
10. MPU9250_FIFO_stop_when_full.ino
11. MPU9250_FIFO_continuous.ino
12. MPU6500_all_data.ino

The sketch MPU9250_blank_all_settings.ino can be used as a basis for your own sketches. It contains all setting options.
