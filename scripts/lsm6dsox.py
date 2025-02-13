'''
This script collects data from the ST LSM6DSOX accelerometer. It exposes an API for collecting 
'''
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
def collect():
    #I2C
    i2c = board.I2C()
    sensor = LSM6DSOX(i2c)
    #SPI
    # spi = board.SPI()
    # sensor = LSM6DSOX(spi)

    sensor.accelerometer_range = AccelRange.RANGE_2G
    sensor.accelerometer_data_rate = Rate.RATE_52_HZ

    # Continuously printing out
    while True:
        accel_x, accel_y, accel_z = sensor.acceleration  # Get accelerometer values
        # Convert acceleration from m/s² to g
        accel_x_g = accel_x / 9.81
        accel_y_g = accel_y / 9.81
        accel_z_g = accel_z / 9.81
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f g" % (accel_x_g, accel_y_g, accel_z_g))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
        print("")
        time.sleep(0.5)

collect()
