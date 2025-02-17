'''
This file acts as a test script for the Analog Devices ADXL367 3-axis accelerometer.
It exposes an API which is then utilized for live data plotting by accel_test.py. 
'''

from datetime import datetime
import numpy as np
import scipy as sp
from matplotlib import pyplot as plt
from spidev import SpiDev
import smbus
from time import sleep
import hdf5storage as h5

# select the correct i2c bus for this revision of Raspberry Pi
revision = ([l[12:-1] for l in open('/proc/cpuinfo','r').readlines() if l[:8]=="Revision"]+['0000'])[0]
bus = smbus.SMBus(1 if int(revision, 16) >= 4 else 0)

# ADXL367 REGISTER MAP

_ADXL367_DEVID_AD                                   = 0x00
_ADXL367_DEVID_MST                                  = 0x01
_ADXL367_PART_ID                                    = 0x02
_ADXL367_REV_ID                                     = 0x03
_ADXL367_RESERVED                                   = 0x04
_ADXL367_SERIAL_NUMBER_2                            = 0x05
_ADXL367_SERIAL_NUMBER_1                            = 0x06
_ADXL367_SERIAL_NUMBER_0                            = 0x07
_ADXL367_XDATA                                      = 0x08
_ADXL367_YDATA                                      = 0x09
_ADXL367_ZDATA                                      = 0x0A
_ADXL367_STATUS                                     = 0x0B
_ADXL367_FIFO_ENTRIES_L                             = 0x0C
_ADXL367_FIFO_ENTRIES_H                             = 0x0D
_ADXL367_XDATA_H                                    = 0x0E
_ADXL367_XDATA_L                                    = 0x0F
_ADXL367_YDATA_H                                    = 0x10
_ADXL367_YDATA_L                                    = 0x11
_ADXL367_ZDATA_H                                    = 0x12
_ADXL367_ZDATA_L                                    = 0x13
_ADXL367_TEMP_H                                     = 0x14
_ADXL367_TEMP_L                                     = 0x15
_ADXL367_EX_ADC_H                                   = 0x16
_ADXL367_EX_ADC_L                                   = 0x17
_ADXL367_I2C_FIFO_DATA                              = 0x18
_ADXL367_SOFT_RESET                                 = 0x1F
_ADXL367_THRESH_ACT_H                               = 0x20
_ADXL367_THRESH_ACT_L                               = 0x21
_ADXL367_TIME_ACT                                   = 0x22
_ADXL367_THRESH_INACT_H                             = 0x23
_ADXL367_THRESH_INACT_L                             = 0x24
_ADXL367_TIME_INACT_H                               = 0x25
_ADXL367_TIME_INACT_L                               = 0x26
_ADXL367_ACT_INACT_CTL                              = 0x27
_ADXL367_FIFO_CONTROL                               = 0x28
_ADXL367_FIFO_SAMPLES                               = 0x29
_ADXL367_INTMAP1_LOWER                              = 0x2A
_ADXL367_INTMAP2_LOWER                              = 0x2B
_ADXL367_FILTER_CTL                                 = 0x2C
_ADXL367_POWER_CTL                                  = 0x2D
_ADXL367_SELF_TEST                                  = 0x2E
_ADXL367_TAP_THRESH                                 = 0x2F
_ADXL367_TAP_DUR                                    = 0x30
_ADXL367_TAP_LATENT                                 = 0x31
_ADXL367_TAP_WINDOW                                 = 0x32
_ADXL367_X_OFFSET                                   = 0x33
_ADXL367_Y_OFFSET                                   = 0x34
_ADXL367_Z_OFFSET                                   = 0x35
_ADXL367_X_SENS                                     = 0x36
_ADXL367_Y_SENS                                     = 0x37
_ADXL367_Z_SENS                                     = 0x38
_ADXL367_TIMER_CTL                                  = 0x39
_ADXL367_INTMAP1_UPPER                              = 0x3A
_ADXL367_INTMAP2_UPPER                              = 0x3B
_ADXL367_ADC_CTL                                    = 0x3C
_ADXL367_TEMP_CTL                                   = 0x3D
_ADXL367_TEMP_ADC_OVER_THRESH_H                     = 0x3E
_ADXL367_TEMP_ADC_OVER_THRESH_L                     = 0x3F
_ADXL367_TEMP_ADC_UNDER_THRESH_H                    = 0x40
_ADXL367_TEMP_ADC_UNDER_THRESH_L                    = 0x41
_ADXL367_TEMP_ADC_TIMER                             = 0x42
_ADXL367_AXIS_MASK                                  = 0x43
_ADXL367_STATUS_COPY                                = 0x44

# _STATUS bitfields
_STATUS_ERR_USER_REGS_SHIFT                         = 7
_STATUS_ERR_USER_REGS_MASK                          = 0b1
_STATUS_SEU_ERROR_DETECTED                          = 0b1

_STATUS_AWAKE_SHIFT                                 = 6
_STATUS_AWAKE_MASK                                  = 0b1
_STATUS_AWAKE_INACTIVE                              = 0b0
_STATUS_AWAKE_ACTIVE                                = 0b1

_STATUS_INACT_SHIFT                                 = 5
_STATUS_INACT_MASK                                  = 0b1
_STATUS_INACT_DETECTED                              = 0b1

_STATUS_ACT_SHIFT                                   = 4
_STATUS_ACT_MASK                                    = 0b1
_STATUS_ACT_DETECTED                                = 0b1

_STATUS_FIFO_OVER_RUN_SHIFT                         = 3
_STATUS_FIFO_OVER_RUN_MASK                          = 0b1
_STATUS_FIFO_OVER_RUN                               = 0b1

_STATUS_FIFO_WATER_MARK_SHIFT                       = 2
_STATUS_FIFO_WATER_MARK_MASK                        = 0b1
_STATUS_FIFO_WATER_MARK_REACHED                     = 0b1

_STATUS_FIFO_READY_SHIFT                            = 1
_STATUS_FIFO_READY_MASK                             = 0b1
_STATUS_FIFO_READY                                  = 0b1

_STATUS_DATA_READY_SHIFT                            = 0
_STATUS_DATA_READY_MASK                             = 0b1
_STATUS_DATA_READY                                  = 0b1

# _XDATA_H bitfields
_XDATA_H_XDATA_SHIFT                                = 0
_XDATA_H_XDATA_MASK                                 = 0b11111111

# _XDATA_L bitfields
_XDATA_L_XDATA_SHIFT                                = 2
_XDATA_L_XDATA_MASK                                 = 0b11111

# _YDATA_H bitfields
_YDATA_H_YDATA_SHIFT                                = 0
_YDATA_H_YDATA_MASK                                 = 0b11111111

# _YDATA_L bitfields
_YDATA_L_YDATA_SHIFT                                = 2
_YDATA_L_YDATA_MASK                                 = 0b11111

# _ZDATA_H bitfields
_ZDATA_H_ZDATA_SHIFT                                = 0
_ZDATA_H_ZDATA_MASK                                 = 0b11111111

# _ZDATA_L bitfields
_ZDATA_L_ZDATA_SHIFT                                = 2
_ZDATA_L_ZDATA_MASK                                 = 0b11111

# _INTMAP1_LOWER bitfields
_INTMAP1_LOWER_INT_LOW_INT1_SHIFT                   = 7
_INTMAP1_LOWER_INT_LOW_INT1_MASK                    = 0b1
_INTMAP1_LOWER_INT_LOW_INT1_ENABLE                  = 0b1
_INTMAP1_LOWER_INT_LOW_INT1_DISABLE                 = 0b0

_INTMAP1_LOWER_AWAKE_INT1_SHIFT                     = 6
_INTMAP1_LOWER_AWAKE_INT1_MASK                      = 0b1
_INTMAP1_LOWER_AWAKE_INT1_ENABLE                    = 0b1
_INTMAP1_LOWER_AWAKE_INT1_DISABLE                   = 0b0

_INTMAP1_LOWER_INACT_INT1_SHIFT                     = 5
_INTMAP1_LOWER_INACT_INT1_MASK                      = 0b1
_INTMAP1_LOWER_INACT_INT1_ENABLE                    = 0b1
_INTMAP1_LOWER_INACT_INT1_DISABLE                   = 0b0

_INTMAP1_LOWER_ACT_INT1_SHIFT                       = 4
_INTMAP1_LOWER_ACT_INT1_MASK                        = 0b1
_INTMAP1_LOWER_ACT_INT1_ENABLE                      = 0b1
_INTMAP1_LOWER_ACT_INT1_DISABLE                     = 0b0

_INTMAP1_LOWER_FIFO_OVERRUN_INT1_SHIFT              = 3
_INTMAP1_LOWER_FIFO_OVERRUN_INT1_MASK               = 0b1
_INTMAP1_LOWER_FIFO_OVERRUN_INT1_ENABLE             = 0b1
_INTMAP1_LOWER_FIFO_OVERRUN_INT1_DISABLE            = 0b0

_INTMAP1_LOWER_FIFO_WATERMARK_INT1_SHIFT            = 2
_INTMAP1_LOWER_FIFO_WATERMARK_INT1_MASK             = 0b1
_INTMAP1_LOWER_FIFO_WATERMARK_INT1_ENABLE           = 0b1
_INTMAP1_LOWER_FIFO_WATERMARK_INT1_DISABLE          = 0b0

_INTMAP1_LOWER_FIFO_RDY_INT1_SHIFT                  = 1
_INTMAP1_LOWER_FIFO_RDY_INT1_MASK                   = 0b1
_INTMAP1_LOWER_FIFO_RDY_INT1_ENABLE                 = 0b1
_INTMAP1_LOWER_FIFO_RDY_INT1_DISABLE                = 0b0

_INTMAP1_LOWER_DATA_RDY_INT1_SHIFT                  = 0
_INTMAP1_LOWER_DATA_RDY_INT1_MASK                   = 0b1
_INTMAP1_LOWER_DATA_RDY_INT1_ENABLE                 = 0b1
_INTMAP1_LOWER_DATA_RDY_INT1_DISABLE                = 0b0

# _FILTER_CTL bitfields
_FILTER_CTL_RANGE_SHIFT                             = 6
_FILTER_CTL_RANGE_MASK                              = 0b11
_FILTER_CTL_RANGE_2G                                = 0b00
_FILTER_CTL_RANGE_4G                                = 0b01
_FILTER_CTL_RANGE_8G                                = 0b10

_FILTER_CTL_I2C_HS_SHIFT                            = 4
_FITLER_CTL_I2C_HS_MASK                             = 0b1
_FILTER_CTL_I2C_HS_ON                               = 0b1
_FILTER_CTL_I2C_HS_OFF                              = 0b0

_FILTER_CTL_EXT_SAMPLE_SHIFT                        = 3
_FILTER_CTL_EXT_SAMPLE_MASK                         = 0b1
_FILTER_CTL_EXT_SAMPLE_ON                           = 0b1
_FILTER_CTL_EXT_SAMPLE_OFF                          = 0b0

_FILTER_CTL_ODR_SHIFT                               = 0
_FILTER_CTL_ODR_MASK                                = 0b111
_FILTER_CTL_ODR_12_5                                = 0b000
_FILTER_CTL_ODR_25                                  = 0b001
_FILTER_CTL_ODR_50                                  = 0b010
_FILTER_CTL_ODR_100                                 = 0b011
_FILTER_CTL_ODR_200                                 = 0b100
_FILTER_CTL_ODR_400                                 = 0b101

# _POWER_CTL bitfields
_POWER_CTL_EXT_CLK_SHIFT                            = 6
_POWER_CTL_EXT_CLK_MASK                             = 0b11
_POWER_CTL_EXT_CLK_ON                               = 0b1
_POWER_CTL_EXT_CLK_OFF                              = 0b0

_POWER_CTL_NOISE_SHIFT                              = 4
_POWER_CTL_NOISE_MASK                               = 0b11
_POWER_CTL_NOISE_LOW_POWER                          = 0b1
_POWER_CTL_NOISE_LOW_NOISE                          = 0b0

_POWER_CTL_WAKEUP_SHIFT                             = 3
_POWER_CTL_WAKEUP_MASK                              = 0b1
_POWER_CTL_WAKEUP_ON                                = 0b1
_POWER_CTL_WAKEUP_OFF                               = 0b0

_POWER_CTL_AUTOSLEEP_SHIFT                          = 2
_POWER_CTL_AUTOSLEEP_MASK                           = 0b1
_POWER_CTL_AUTOSLEEP_ON                             = 0b1
_POWER_CTL_AUTOSLEEP_OFF                            = 0b0

_POWER_CTL_MEASURE_SHIFT                            = 0
_POWER_CTL_MEASURE_MASK                             = 0b11
_POWER_CTL_MEASURE_STANDBY                          = 0b10
_POWER_CTL_MEASURE_MEASUREMENT                      = 0b10

# ADXL367 constants
EARTH_GRAVITY_MS2   = 9.80665
SCALE_MULTIPLIER    = 2.0 / 8192

ADXL367_ODR_FS = {
    _FILTER_CTL_ODR_12_5: 12.5,
    _FILTER_CTL_ODR_25: 25,
    _FILTER_CTL_ODR_50: 50,
    _FILTER_CTL_ODR_100: 100,
    _FILTER_CTL_ODR_200: 200,
    _FILTER_CTL_ODR_400: 400,
}

class ADXL367:

    address = None

    def __init__(self, address = 0x53):
        self.address = address
        self.setNoiseMode(_POWER_CTL_NOISE_LOW_NOISE)
        self.setBandwidthRate(_FILTER_CTL_ODR_50)
        self.setRange(_FILTER_CTL_RANGE_2G)
        self.enableMeasurement()

    # set the accelerometer into standby mode
    def enableStandby(self):
        STANDBY_VAL = np.uint8(_POWER_CTL_MEASURE_STANDBY << _POWER_CTL_MEASURE_STANDBY_SHIFT)
        bus.write_byte_data(self.address, _ADXL367_POWER_CTL, STANDBY_VAL)

    # set the accelerometer into measurement mode
    def enableMeasurement(self):
        MEASUREMENT_VAL = np.uint8(_POWER_CTL_MEASURE_MEASUREMENT << _POWER_CTL_MEASURE_SHIFT)
        bus.write_byte_data(self.address, _ADXL367_POWER_CTL, MEASUREMENT_VAL)

    def setNoiseMode(self, mode):
        MODE_VAL = np.uint8(mode << _POWER_CTL_NOISE_SHIFT)
        bus.write_byte_data(self.address, _ADXL367_POWER_CTL, MODE_VAL)

    # set the accelerometer bandwidth
    def setBandwidthRate(self, bw):
        BW_VAL = np.uint8(bw << _FILTER_CTL_ODR_SHIFT)
        bus.write_byte_data(self.address, _ADXL367_FILTER_CTL, bw)

    # set the measurement range for 10-bit readings
    def setRange(self, range):
        RANGE_VAL = np.uint8(range << _FILTER_CTL_RANGE_SHIFT)
        bus.write_byte_data(self.address, _ADXL367_FILTER_CTL, RANGE_VAL)

    def readDataReady(self):
        status = bus.read_byte_data(self.address, _ADXL367_STATUS)
        #print("DEBUG: STATUS REG: {:08b}".format(status))
        ready = (status >> _STATUS_DATA_READY_SHIFT) & _STATUS_DATA_READY_MASK
        return ready

    def twosComplement(self, num):
    # If the sign bit is set (bit 15), adjust the value to negative using two's complement
        if (num & 0x2000):  # Check if the sign bit is set
            num -= 0x4000   # Convert to negative value in two's complement
        return num

    def convertData(self, raw_data):
        return self.twosComplement(np.int16(raw_data)) * SCALE_MULTIPLIER

    def readX(self):
        # Obtain raw x value
        XDATA_ADJUSTMENT_SHIFT = 6
        xdata_high = bus.read_byte_data(self.address, _ADXL367_XDATA_H)
        xdata_low = bus.read_byte_data(self.address, _ADXL367_XDATA_L)
        xdata_raw = (xdata_high << XDATA_ADJUSTMENT_SHIFT) | (xdata_low >> _XDATA_L_XDATA_SHIFT)
#        print(f"Raw X data before conversion: {xdata_raw}")
#        new = self.convertData(xdata_raw)
#        print(f"X data after twosComplement: {self.twosComplement(xdata_raw)}")
        return self.convertData(xdata_raw)

    def readY(self):
        # Obtain raw y value
        YDATA_ADJUSTMENT_SHIFT = 6
        ydata_high = bus.read_byte_data(self.address, _ADXL367_YDATA_H)
        ydata_low = bus.read_byte_data(self.address, _ADXL367_YDATA_L)
        ydata_raw = (ydata_high << YDATA_ADJUSTMENT_SHIFT) | (ydata_low >> _YDATA_L_YDATA_SHIFT)
#        print(f"Raw Y data before conversion: {ydata_raw}")
#        new = self.convertData(ydata_raw)
#        print(f"Y data after twosComplement: {self.twosComplement(ydata_raw)}")
        return self.convertData(ydata_raw)

    def readZ(self):
        # Obtain raw z value
        ZDATA_ADJUSTMENT_SHIFT = 6
        zdata_high = bus.read_byte_data(self.address, _ADXL367_ZDATA_H)
        zdata_low = bus.read_byte_data(self.address, _ADXL367_ZDATA_L)
        zdata_raw = (zdata_high << ZDATA_ADJUSTMENT_SHIFT) | (zdata_low >> _ZDATA_L_ZDATA_SHIFT)
#        print(f"Raw Z data before conversion: {zdata_raw}")
#        new = self.convertData(zdata_raw)
#        print(f"Z data after twosComplement: {self.twosComplement(zdata_raw)}")
        return self.convertData(zdata_raw)

    def getAxes(self, gforce = False):
        x = self.readX()
        y = self.readY()
        z = self.readZ()

        return {"x": x, "y": y, "z": z}

    def recordData(self, Fs):
        # Number of samples to record based on sampling frequency (Fs)
        num_samples = Fs * 60  # 1 minute of data

        # Create empty lists to store the data for each axis
        data_x = []
        data_y = []
        data_z = []

        print(f"Recording data at {Fs} Hz for 1 minute...")

        # Collect data for 'num_samples' samples
        for i in range(num_samples):
            if (self.readDataReady()):
                axes = self.getAxes(True)  # Get axes data in g-force (True)
                data_x.append(axes['x'])
                data_y.append(axes['y'])
                data_z.append(axes['z'])

            # Sleep for the time interval based on the sampling rate
            sleep(1 / Fs)

        # Store the data in a dictionary
        data = {
            "x": data_x,
            "y": data_y,
            "z": data_z
        }

        return data

    # parameter gforce:
    #    False (default): result is returned in m/s^2
    #    True           : result is returned in gs
    def getAxesDebug(self, gforce = False):
        bytes = bus.read_i2c_block_data(self.address, _ADXL367_XDATA_H, 6)
        # TODO: SEE IF I NEED TO HANDLE THIS
        x = bytes[0] | (bytes[1] << 8)
        if(x & (1 << 16 - 1)):
            x = x - (1<<16)

        y = bytes[2] | (bytes[3] << 8)
        if(y & (1 << 16 - 1)):
            y = y - (1<<16)

        z = bytes[4] | (bytes[5] << 8)
        if(z & (1 << 16 - 1)):
            z = z - (1<<16)

        x = x * SCALE_MULTIPLIER
        y = y * SCALE_MULTIPLIER
        z = z * SCALE_MULTIPLIER

        if gforce == False:
            x = x * EARTH_GRAVITY_MS2
            y = y * EARTH_GRAVITY_MS2
            z = z * EARTH_GRAVITY_MS2

        x = round(x, 4)
        y = round(y, 4)
        z = round(z, 4)

        return {"x": x, "y": y, "z": z}

def plot_timeseries(accel_data, sample_rate):
    # Ensure the data is a dictionary with 'x', 'y', and 'z'
    if not all(axis in accel_data for axis in ['x', 'y', 'z']):
        raise ValueError("Accelerometer data must include 'x', 'y', and 'z' components.")

    # Create time array based on sample_rate
    num_samples = len(accel_data['x'])
    time = np.arange(num_samples) / sample_rate  # Time in seconds

    # Create the plot
    plt.figure(figsize=(10, 6))

    # Plot data for X, Y, and Z axes
    plt.plot(time, accel_data['x'], label='X-Axis', color='r', linewidth=1.5)
    plt.plot(time, accel_data['y'], label='Y-Axis', color='g', linewidth=1.5)
    plt.plot(time, accel_data['z'], label='Z-Axis', color='b', linewidth=1.5)

    # Add labels and title
    plt.xlabel('Time [s]')
    plt.ylabel('Acceleration [m/s² or G]')
    plt.title('Accelerometer Time-Series Data')

    # Add a legend
    plt.legend()

    # Display grid
    plt.grid(True)

    # Show the plot
    plt.tight_layout()
    plt.show()

def timedCap():
    # if run directly we'll just create an instance of the class and output 
    # the current readings
    Fs = 50
    start_time = datetime.now().astimezone()
    filename = "adxl367-" + str(ADXL367_ODR_FS[_FILTER_CTL_ODR_50]) + start_time.strftime('-%Y-%m-%dT%H-%M-%S') + '.mat'
    data = {}

    adxl367 = ADXL367()

    print("Starting Data Capture...")
#    while True:
#        if (adxl367.readDataReady()):
#            axes = adxl367.getAxes(True)
#            print("ADXL367 on address 0x%x:" % (adxl367.address))
#            print("x = %.3fG" % ( axes['x'] ))
#            print("y = %.3fG" % ( axes['y'] ))
#            print("z = %.3fG" % ( axes['z'] ))

    data['Fs'] = ADXL367_ODR_FS[_FILTER_CTL_ODR_50]
    data['sData'] = {}
    data['sData'] = adxl367.recordData(Fs)

    print("Finished Data Capture...")

    plot_timeseries(data['sData'], Fs)
    #print(f"Recorded data: {data['sData']}")

    sp.io.savemat(filename, data)

if __name__ == "__main__":
    timedCap()
