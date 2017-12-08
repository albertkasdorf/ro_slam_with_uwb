#!/usr/bin/python3
# https://docs.python.org/3.6/library/struct.html
# http://pyserial.readthedocs.io/en/latest/shortintro.html
# https://stackoverflow.com/questions/3191528/csv-in-python-adding-an-extra-carriage-return
# https://arduino.stackexchange.com/questions/9899/serial-structure-data-transfer-between-an-arduino-and-a-linux-pc

# TODO:
# [3.4] => New ground truth, flush input buffer, make n mesarements. Ask again.
# [k] => autoincrement by 10 cm
# [q] => Quit
# reset_input_buffer()
# Flush input buffer, discarding all its contents.
# Changed in version 3.0: renamed from flushInput()
# http://pyserial.readthedocs.io/en/latest/pyserial_api.html


import struct
import csv
import serial

from struct import *

serial_port = "COM4"
csv_file_name = "data.csv"
ground_truth = 0.2
measurements = 100

with serial.Serial(serial_port, 115200, timeout=1) as ser:
    with open(csv_file_name, "a") as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')

        for i in range(measurements):
            while True:
                line = ser.readline()
                print(line)
                if len(line) != 31:
                    continue
                try:
                    data = unpack('=B2h6f', line[0:29])
                except struct.error:
                    continue
                break

            # ground_truth, range, receive_power, first_path_power, receive_quality, temperature, voltage
            csv_writer.writerow([ground_truth, data[3], data[4], data[5], data[6], data[7], data[8]])
