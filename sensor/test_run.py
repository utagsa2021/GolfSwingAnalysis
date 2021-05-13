# Script to test the Sensor_Driver function and associated csv operations

from Sensor_Driver import get_sensor_data

seconds_to_record = 10
in_file_name = "all_data.csv"
out_file_name = "all_data_trimmed.csv"

get_sensor_data(seconds_to_record, in_file_name, out_file_name)