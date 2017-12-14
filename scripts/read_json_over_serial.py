#!/usr/bin/python3
# https://docs.python.org/3.6/library/struct.html
# http://pyserial.readthedocs.io/en/latest/shortintro.html
# https://stackoverflow.com/questions/3191528/csv-in-python-adding-an-extra-carriage-return
# https://arduino.stackexchange.com/questions/9899/serial-structure-data-transfer-between-an-arduino-and-a-linux-pc

import json
import csv
import serial

serial_port = "/dev/CP2104_Friend"
csv_file_name = "file_name.csv"
ground_truth = 1.0
measurements = 250
antenna_delay = b'16440'

with serial.Serial(serial_port, 115200, timeout=2) as ser:
    with open(csv_file_name, "a") as csv_file:
        csv_writer = csv.writer(csv_file, delimiter=';', lineterminator='\n')
        csv_writer.writerow([
            "type",
            "tag_address", "anchor_address",
            "range_ground_truth", "range",
            "receive_power", "first_path_power",
            "receive_quality",
            "temperature", "voltage",
            "antenna_delay"])

        while True:
            line = ser.readline()
            line = line.decode("utf-8")
            print(line)
            if line.startswith("### TAG ###"):
                ser.write(antenna_delay)
                break

        for i in range(0, measurements):
            print("{0:03d} of {1:03d}".format(i+1, measurements))
            while True:
                line = ser.readline()
                line = line.decode("utf-8")
                try:
                    print(line)
                    obj = json.loads(line)
                except ValueError:
                    continue
                except TypeError:
                    continue
                break

            # type, tag_address, anchor_address,
            # ground_truth, range,
            # receive_power, first_path_power,
            # receive_quality,
            # temperature, voltage
            # antenna_delay
            csv_writer.writerow([
                obj["type"],
                obj["ta"], obj["aa"],
                ground_truth, obj["r"],
                obj["rxp"], obj["fpp"],
                obj["q"],
                obj["t"], obj["v"],
                obj["ad"]])
