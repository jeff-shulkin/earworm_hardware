'''
This file takes in an option from the terminal, loads recorded baseline data and blood pressure data from selected accelerometer.
It then calculates and spits out the SNR of each accelerometer.
'''

import os
import numpy as np
import scipy as sp
import hdf5storage as h5
from matplotlib import pyplot as plt

SENSORS = ["ADXL367", "BMA530", "LIS2DUX12", "LIS2DW12", "LSM6DSOX"]
SAMPLING_RATE = 50 # Each sensor sampled at 50 Hz
SENSOR_AVG_uA = [1, 5, 10, 11, 19] # Average current for each sensor in uA

data_folder = pathlib.Path("data")

datasets = [
    Dataset.from_file(path) for path in data_folder.glob("*.mat")
]

'''
Loads in one set of baseline data from sensor
'''
def load_data(file_path: PathLike):
    file = pathlib.Path(file_path)
    data = h5.loadmat(str(file), options=STORAGE_OPTIONS)
    data["Fs"] = np.squeeze(data["Fs"]).item()
    return data

def load_datasets(type:str) -> List[np.ndarray]
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

def calculate_snrs(signals: List[np.ndarray, noises: np.ndarray) -> List[np.float]
    for i in range(len(signals)):
        

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
    baseline = load_datasets("baseline")
    test = load_datasets("test")
    plot_snr(baseline, test)
