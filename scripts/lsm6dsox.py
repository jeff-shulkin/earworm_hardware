'''
This script collects data from the ST LSM6DSOX accelerometer. It exposes an API for collecting 
'''
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

def init():
    #I2C
    i2c = board.I2C()
    sensor = LSM6DSOX(i2c)
    #SPI
    # spi = board.SPI()
    # sensor = LSM6DSOX(spi)

    # Continuously printing out
    while True:
        print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
        print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
        print("")
        time.sleep(0.5)
