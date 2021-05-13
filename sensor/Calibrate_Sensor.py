import serial
import time
import math
import numpy as np
import csv
from Quat_Funcs import quat_to_euler
from Csv_Manip import trim_csv

from pyquaternion import Quaternion

# Make sure to change the usb device accordingly
arduinoData = serial.Serial("/dev/ttyUSB0", 115200)

seconds_to_record = 10
in_file_name = "all_data.csv"
out_file_name = "all_data_trimmed.csv"

time.sleep(1)


def get_sensor_data():

    # Sets t_end based on current clock time
    # Adds 1 second to compensate for sensor startup time

    # Redeclaring the file name to clear out previous swing

    # Runs while loop for desired duration
    while time.time() < t_end:

        # Waiting on the sensor to detect new data
        while arduinoData.inWaiting() == 0:
            pass

        # Reading serial data from the sensor and parsing it
        # Channels 0-3:  Calibration accel, gyro, magnetometer, system
        # Channels 4-7:  Quaternion w, x, y, z
        # Channels 8-10: linear acceleration
        # Channel  11:   time since program start in ms
        dataPacket = arduinoData.readline()
        dataPacket = str(dataPacket, "utf-8")
        splitPacket = dataPacket.split(",")
        splitPacketFloat = [float(item) for item in splitPacket]
        calibration_data = splitPacketFloat[0:4]

        # Printing pitch, roll, and yaw for troubleshooting purposes
        print(f"pitch = {pitch}, roll = {roll}, yaw = {yaw}")

        # Writing each line of data to the csv
