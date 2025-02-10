'''
This script collects data from the ST LSM6DSOX accelerometer. It exposes an API for collecting 
'''
import time
import board
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX

def init():
    