import csv
import numpy as np
import pandas as pd


# Removes first few rows of data that don't have actual sensor readings
def trim_csv(in_file_name, out_file_name):

    with open(in_file_name, "r") as inp, open(out_file_name, "w") as out:
        writer = csv.writer(out)
        for row in csv.reader(inp):
            if float(row[14]) != 0:
                writer.writerow(row)


# Reads csv file into numpy array
def read_csv_data(file_name):

    file_1 = pd.read_csv(file_name)

    return file_1.values