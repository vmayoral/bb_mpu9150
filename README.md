BB-MPU9150
===========

![Invensense MPU-9150](http://www.drotek.fr/shop/185-431-large_dm/mpu9150-ic.jpg)

This repository creates a ROS package that publishes the Invensense MPU-9150 data into a ROS topic.

----

(The code uses the [linux-mpu9150](https://github.com/Pansenti/linux-mpu9150) driver.)

---


####mpu9150_node
Get samples from the Invensense MPU9150 sensor and output these processed samples. Outputs can be either euler angles, quaternions, calibrated accelerometer or calibrated magnetometer.
Default values (these values should be changed at the local_defaults.h and afterwards the code should be cross-compiled again):
* Default I2C Bus: 1 (i2c-2 at the Beaglebone).
* Default sample rate: 10 Hz
* Default yaw mix factor: 4

#####Subscribed topics
none

#####Published topics
*imu_euler (std_msgs::String)*







