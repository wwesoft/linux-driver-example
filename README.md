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
* Add new sysfs attributes for fast offset compensation:
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
* Add low pass filter for the accelerometer sensor data
  * > Get available 3dB cutoff frequencies that depend on the sampling rate.
  * > Read and write raw value of IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY [line 893](drivers/iio/imu/bmi160/bmi160_core.c#L893) [line 927](drivers/iio/imu/bmi160/bmi160_core.c#L927) \
    > Set IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY bit in info_mask_shared_by_type of iio_chan_spec structure [code](drivers/iio/imu/bmi160/bmi160_core.c#L203)
* There are 3 kind of iio trigger are registered: [code](drivers/iio/imu/bmi160/bmi160_core.c#L2151)\
  And set state of trigger: [code](drivers/iio/imu/bmi160/bmi160_core.c#L1942)
  * > bmi160-dev trigger based on data ready interrupt\
    > Data ready interrupt can be mapped to the appropriate pin (INT1 or INT2).
    > When DRDY interrupt is mapped, other interrupt should not be mapped to the same pin. [code](drivers/iio/imu/bmi160/bmi160_core.c#L1055)
  * > bmi160-any-motion-dev trigger based on any motion interrupt
    > Any motion interrupt can be mapped to the appropriate pin (INT1 or INT2) only when DRDY interrupt is not set and mapped. [code](drivers/iio/imu/bmi160/bmi160_core.c#L1027)
  * > bmi160-no-motion-dev trigger based on no motion interrupt
    > No motion interrupt can be mapped to the appropriate pin (INT1 or INT2) only when DRDY interrupt is not set and mapped. [code](drivers/iio/imu/bmi160/bmi160_core.c#L1027)
* Add IIO Event management.
  * > Any motion event is associated with iio event called in_roc_rising. [code](drivers/iio/imu/bmi160/bmi160_core.c#L514)\
    > There are two attributes: Rate of change threshold, and period.
  * > No motion event is associated with iio event called in_roc_falling. [code](drivers/iio/imu/bmi160/bmi160_core.c#L520)\
    > There are two attributes: Rate of change threshold, and period. [(read event)](drivers/iio/imu/bmi160/bmi160_core.c#L1094) [(write event)](drivers/iio/imu/bmi160/bmi160_core.c#L1141)
* Allocate an interrupt line for iio device [code](drivers/iio/imu/bmi160/bmi160_core.c#L2187)
  * > bmi160_irq_handler: Function to be called when the IRQ occur. [code](drivers/iio/imu/bmi160/bmi160_core.c#L2092)\
    > This funclion calls iio_trigger_poll with all triggers that are enabled.\
    > Function bmi160_trigger_handler is called if there is a enabled trigger. [code](drivers/iio/imu/bmi160/bmi160_core.c#L835)
  * > bmi160_irq_thread_handler: Function to be called in a threaded interrupt context. [code](drivers/iio/imu/bmi160/bmi160_core.c#L2065)

