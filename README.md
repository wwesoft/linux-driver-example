# Linux kernel driver

> There is a updated driver code that is inherited from Toradex linux kernel.\
> Updated driver code: bmi160.\
> New kernel driver module is qt60248 QMatrix touch sensor.

## BMI160 - Bosch IMU (accel, gyro plus external magnetometer)

### Upgrade from mainline

* insert of_match_table into driver structure. [code](drivers/iio/imu/bmi160/bmi160_i2c.c#L62)
* config irq reading of_node "INT1" or "INT2". [code](drivers/iio/imu/bmi160/bmi160_core.c#L1773)
* set DRDY bit in interrupt enable register.
* set data ready trigger.

### Add new features

* Fast offset compensation
* add new sysfs attributes for fast offset compensation:
  * > gyro_offset_enable:\
    > Enable offset compensation for gyroscope [code](drivers/iio/imu/bmi160/bmi160_core.c#L1623)
  * > accel_offset_enable:\
    > Enable offset compensation for accelerometer [code](drivers/iio/imu/bmi160/bmi160_core.c#L1620)
  * > gyro_calib_config:\
    > Enable fast offset calibration for gyroscope [code](drivers/iio/imu/bmi160/bmi160_core.c#L1607)
  * > accel_x_calib_config, accel_y_calib_config, accel_x_calib_config:\
    > Set compensation target value for x, y ,z axis of the accelerometer:\
    > disabled, +1g, -1g or 0g [code](drivers/iio/imu/bmi160/bmi160_core.c#L1610)
  * > Fast offset compensation start [code](drivers/iio/imu/bmi160/bmi160_core.c#L1626)
* add low pass filter for the accelerometer sensor data
  * > Get available 3dB cutoff frequencies that depend on the sampling rate.
  * > Read and write raw value of IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY [line 893](drivers/iio/imu/bmi160/bmi160_core.c#L893) [line 927](drivers/iio/imu/bmi160/bmi160_core.c#L927) \
    > Set IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY bit in info_mask_shared_by_type of iio_chan_spec structure [code](drivers/iio/imu/bmi160/bmi160_core.c#L203)
* there are 3 kind of iio trigger are registered: [code](drivers/iio/imu/bmi160/bmi160_core.c#L2151)
  * > bmi160-dev trigger based on data ready interrupt
  * > bmi160-any-motion-dev trigger based on any motion interrupt
  * > bmi160-no-motion-dev trigger based on no motion interrupt
