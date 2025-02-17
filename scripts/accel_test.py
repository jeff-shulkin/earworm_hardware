'''
This file takes in an option from the terminal, loads recorded baseline data and blood pressure data from selected accelerometer.
It then calculates and spits out the SNR of each accelerometer.
'''

import os
import numpy as np
import scipy as sp
from matplotlib import pyplot as plt
import pathlib

SENSORS = ["ADXL367", "BMA530", "LIS2DUX12", "LIS2DW12", "LSM6DSOX"]
SAMPLING_RATE = 50 # Each sensor sampled at 50 Hz
SENSOR_AVG_uA = [1, 5, 10, 11, 19] # Average current for each sensor in uA

data_folder = pathlib.Path("data")

#datasets = [
#    Dataset.from_file(path) for path in data_folder.glob("*.mat")
#]

'''
Loads in one set of baseline data from sensor
'''
def load_data(file_path: os.PathLike):
    file = pathlib.Path(file_path)
    data = sp.io.loadmat(file)
    sData = data['sData']

    # Extract X, Y, Z components from the tuple inside sData
    x_data = sData[0, 0][0].flatten()  # Flatten the array to make it 1D
    y_data = sData[0, 0][1].flatten()
    z_data = sData[0, 0][2].flatten()

    # Return the data as a dictionary
    return {'x': x_data, 'y': y_data, 'z': z_data}

def load_datasets(type:str) -> list[np.ndarray]:
    data = []
    for i in range(len(SENSORS)):
        baseline_data[i] = load_data(SENSORS[i] + type + "_latest.mat")
    return baseline_data

def rms(signal):
    return np.sqrt(np.mean(signal**2))

def snr(signal, noise):
    signal_power = rms(np.mean(signal**2))
    noise_power = rms(np.mean(noise**2))
    return 10*np.log10(signal_power / noise_power)

def calculate_snrs(signals: list[np.ndarray], noises: list[np.ndarray]) -> list[float]:
    pass
    #for i in range(len(signals)):

def plot_timeseries(accel_data, sample_rate):
    """
    Plots time-series accelerometer data for X, Y, and Z axes separately.
    Ensures the range is set to ±2g.
    """
    # Ensure the data is a dictionary with 'x', 'y', and 'z'
    if not all(axis in accel_data for axis in ['x', 'y', 'z']):
        raise ValueError("Accelerometer data must include 'x', 'y', and 'z' components.")

    # Create time array based on sample_rate
    num_samples = len(accel_data['x'])
    time = np.arange(num_samples) / sample_rate  # Time in seconds

    # Create a 3x1 grid of subplots for X, Y, and Z
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot X-axis data
    axs[0].plot(time, accel_data['x'], label='X-Axis', color='r', linewidth=1.5)
    axs[0].set_ylabel('Acceleration [g]')
    axs[0].set_title('X-Axis Time-Series Data')
    axs[0].set_ylim([-2, 2])  # Ensure the range is between -2g and +2g
    axs[0].grid(True)
    axs[0].legend()

    # Plot Y-axis data
    axs[1].plot(time, accel_data['y'], label='Y-Axis', color='g', linewidth=1.5)
    axs[1].set_ylabel('Acceleration [g]')
    axs[1].set_title('Y-Axis Time-Series Data')
    axs[1].set_ylim([-2, 2])  # Ensure the range is between -2g and +2g
    axs[1].grid(True)
    axs[1].legend()

    # Plot Z-axis data
    axs[2].plot(time, accel_data['z'], label='Z-Axis', color='b', linewidth=1.5)
    axs[2].set_xlabel('Time [s]')
    axs[2].set_ylabel('Acceleration [g]')
    axs[2].set_title('Z-Axis Time-Series Data')
    axs[2].set_ylim([-2, 2])  # Ensure the range is between -2g and +2g
    axs[2].grid(True)
    axs[2].legend()

    # Adjust layout to prevent overlapping labels
    plt.tight_layout()

    # Show the plots
    plt.show()


import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

def plot_fft_with_filter(accel_data, sample_rate=100, freq_range=(0, 5), cutoff_freq=20):
    """
    Plots the FFT of accelerometer data for X, Y, and Z axes separately,
    with a low-pass filter applied to remove frequencies above cutoff_freq (Hz).
    """
    # Ensure the data is a dictionary with 'x', 'y', and 'z'
    if not all(axis in accel_data for axis in ['x', 'y', 'z']):
        raise ValueError("Accelerometer data must include 'x', 'y', and 'z' components.")
    
    # Create time array based on sample_rate
    num_samples = len(accel_data['x'])
    time = np.arange(num_samples) / sample_rate  # Time in seconds

    # Apply low-pass filter to the data (cutoff frequency = 20 Hz)
    accel_data['x'] = butter_lowpass_filter(accel_data['x'], cutoff_freq, sample_rate)
    accel_data['y'] = butter_lowpass_filter(accel_data['y'], cutoff_freq, sample_rate)
    accel_data['z'] = butter_lowpass_filter(accel_data['z'], cutoff_freq, sample_rate)

    # Create a 3x1 grid of subplots for X, Y, and Z
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Plot FFT of X-axis data
    axs[0].plot(*compute_fft(accel_data['x'], sample_rate, freq_range), label='X-Axis', color='r', linewidth=1.5)
    axs[0].set_ylabel('Magnitude')
    axs[0].set_title('X-Axis FFT')
    axs[0].grid(True)
    axs[0].legend()
    axs[0].set_xlim(freq_range)  # Set frequency range

    # Plot FFT of Y-axis data
    axs[1].plot(*compute_fft(accel_data['y'], sample_rate, freq_range), label='Y-Axis', color='g', linewidth=1.5)
    axs[1].set_ylabel('Magnitude')
    axs[1].set_title('Y-Axis FFT')
    axs[1].grid(True)
    axs[1].legend()
    axs[1].set_xlim(freq_range)  # Set frequency range

    # Plot FFT of Z-axis data
    axs[2].plot(*compute_fft(accel_data['z'], sample_rate, freq_range), label='Z-Axis', color='b', linewidth=1.5)
    axs[2].set_xlabel('Frequency [Hz]')
    axs[2].set_ylabel('Magnitude')
    axs[2].set_title('Z-Axis FFT')
    axs[2].grid(True)
    axs[2].legend()
    axs[2].set_xlim(freq_range)  # Set frequency range

    # Adjust layout to prevent overlapping labels
    plt.tight_layout()

    # Show the plots
    plt.show()

def compute_fft(data, sample_rate, freq_range):
    """
    Helper function to compute the FFT of the given data and return frequency and magnitude
    for a specific frequency range.
    """
    # Convert the data to a numpy array if it's not already
    data = np.array(data)

    # Perform FFT
    N = len(data)
    freqs = np.fft.fftfreq(N, 1 / sample_rate)  # All frequencies, including negative ones
    fft_values = np.fft.fft(data)

    # Only keep the positive frequencies
    freqs = freqs[:N//2]
    fft_values = fft_values[:N//2]

    # Compute magnitude (absolute value of the complex FFT results)
    mag = np.abs(fft_values)

    # Filter frequencies within the specified range
    valid_freqs = (freqs >= freq_range[0]) & (freqs <= freq_range[1])

    # Apply the valid frequency filter
    valid_freqs = freqs[valid_freqs]
    valid_mag = mag[(freqs >= freq_range[0]) & (freqs <= freq_range[1])]  # Filter magnitude values corresponding to valid frequencies

    return valid_freqs, valid_mag

def butter_lowpass(cutoff, sample_rate, order=4):
    """
    Creates a low-pass Butterworth filter.
    """
    nyquist = 0.5 * sample_rate  # Nyquist frequency
    normal_cutoff = cutoff / nyquist  # Normalized cutoff frequency
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, sample_rate, order=4):
    """
    Applies a low-pass Butterworth filter to the data.
    """
    b, a = butter_lowpass(cutoff, sample_rate, order)
    filtered_data = filtfilt(b, a, data)  # Apply filter
    return filtered_data

'''
Compares SNR of all 5 sensors, plots them on bar graph
Intuition is that, the greater the SNR, the greater chance they'll have
of actually picking of heart-rate/blood pressure.
'''
def plot_snr(baseline, test) -> None:
    fig, axs = plt.subplots(2, 1)

    # First plot just the pure SNR of each sensor
    accel_snrs = calculate_snrs(baseline, test)
    axs[0].bar(SENSORS, accel_snrs)

    axs[0].set_xlabel("Accelerometer")
    axs[0].set_ylabel("SNR (dB)")
    axs[0].set_title("SNR Values vs. sensor")

    # Now plot the SNR adjusted for average current
    snrs_per_uA = accel_snrs / SENSOR_AVG_uA
    axs[1].bar(SENSORS, snrs_per_uA)

    axs[1].set_xlabel("Accelerometer")
    axs[1].set_ylabel("SNR/Current (dB/uA)")
    axs[1].set_title("SNR Values adjusted by average current vs. sensor")


if __name__ == "__main__":
    data = load_data('data/adxl367-50-2025-02-13T05-33-14.mat')
    plot_timeseries(data, 50)
    plot_fft_with_filter(data, 50)
    baseline = load_datasets("baseline")
    test = load_datasets("test")
    plot_snr(baseline, test)
