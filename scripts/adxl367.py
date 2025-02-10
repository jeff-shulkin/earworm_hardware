'''
This file acts as a test script for the Analog Devices ADXL367 3-axis accelerometer.
It exposes an API which is then utilized for live data plotting by accel_test.py. 
'''

from numpy_ringbuffer import RingBuffer
from matplotlib import pyplot as plt
import os
import threading
from spidev import SpiDev

# SPI COMMANDS
READ_REG = 0x0A
WRITE_REG = 0x0B
READ_FIFO = 0x0D

# REGISTER DEFINITIONS
XDATA_H = 0x0E
XDATA_L = 0x0F
YDATA_H = 0x10
YDATA_L = 0x11
ZDATA_H = 0x12
ZDATA_L = 0x13

FIFO_CTL = 0x28
FIFO_SAMPLES = 0x29
INTMAP1_LOWER = 0x2A
INTMAP2_LOWER = 0x2B
FILTER_CTL = 0x2C
POWER_CTL = 0x2D
STATUS_COPY = 0x44


def init():

def read_vals():

def _read_reg():

def _write_reg():
