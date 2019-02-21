#!/usr/bin/env python

import serial
import time
import binascii
import codecs
import struct

BAUDRATE = 500000 #(int) Baud rate such as 9600 or 115200 etc.

def parse_serial_message():
    # write
    ser = serial.Serial('COM3')  # open serial port
    print("Port name: %s" % ser.name)  # check which port was really used
    # ser.write(b'hello')  # write a string
    ser.close()  # close port
    #read
    with serial.Serial('COM3', BAUDRATE, timeout=1) as ser:
        # read a simple line from serial port
        # x = ser.read()  # read one byte
        # s = ser.read(10)  # read up to ten bytes (timeout)


        line = ser.readline() # read a '\n' terminated line
        print(line)
        """for c in line:
            print(hex(c))"""
        # print(line[:92])

        # parse each message
        sats = line[17:18]  # active satellites
        print(sats)
        sats = int.from_bytes(sats, byteorder='big')
        print("Sats: " + str(sats))
        # n = int(word,2)
        # binascii.unhexlify('%x' % n)

        time = line[18:21]  # active satellites
        print(time)
        print("time: ",int.from_bytes(time, byteorder='big'))

        latitude = line[21:25]  # active satellites
        print(latitude)
        print("latitude: ",int.from_bytes(latitude, byteorder='big', signed=True))

        longitude = line[25:29]  # active satellites
        print(longitude)
        print("longitude: ",int.from_bytes(longitude, byteorder='big', signed=True))

        velocity = line[29:31]  # active satellites
        print(velocity)
        print("velocity: ",int.from_bytes(velocity, byteorder='big'))

        heading = line[31:33]  # active satellites
        print(heading)
        print("heading: ",int.from_bytes(heading, byteorder='big'))

        height = line[33:36]  # active satellites
        print(height)
        print("height: ",int.from_bytes(height, byteorder='big'))

        vertical_velocity = line[36:38]  # active satellites
        print(vertical_velocity)
        print("vertical_velocity: ",int.from_bytes(vertical_velocity, byteorder='big', signed=True))

        long_acc = line[38:40]  # active satellites
        print(long_acc)
        print("long_acc: ",int.from_bytes(long_acc, byteorder='big', signed=True))

        lat_acc = line[40:42]  # active satellites
        print(lat_acc)
        print("lat_acc: ",int.from_bytes(lat_acc, byteorder='big', signed=True))

        glonass_sats = line[42:43]  # active satellites
        print(glonass_sats)
        print("glonass_sats: ",int.from_bytes(glonass_sats, byteorder='big'))

        gps_sats = line[43:44]
        print(gps_sats)
        print("gps_sats: ",int.from_bytes(gps_sats, byteorder='big'))

        serial_number = line[44:46]
        print(serial_number)
        print("serial_number: ",int.from_bytes(serial_number, byteorder='big'))

        kalman_filter_status = line[46:48]
        print(kalman_filter_status)
        print("kalman_filter_status: ", int.from_bytes(kalman_filter_status, byteorder='big'))

        solution_type = line[48:50]
        print(solution_type)
        print("solution_type: ", int.from_bytes(solution_type, byteorder='big'))

        velocity_quality = line[50:54]
        print(velocity_quality)
        print("velocity_quality: ", int.from_bytes(velocity_quality, byteorder='big'))

        ram_address = line[54:57]
        print(ram_address)
        print("ram_address: ", int.from_bytes(ram_address, byteorder='big'))

        event_time1 = line[57:61]
        print(event_time1)
        print("event_time1: ", struct.unpack('f', event_time1))



def get_port_list():
    com_ports = list(serial.tools.list_ports.comports())  # get list of all devices connected through serial port
    print("List of devices connected on com ports: %s", com_ports)
    num_ports_used = len(com_ports)  # get number of com ports used   # get number of com ports used in system
    print("Number of com ports used in system: %s", num_ports_used)


def read_serial_port():
    """i = 0
    for modem in PortList:
        for port in modem:
            try:
                ser = serial.Serial(port, 9600, timeout=1)
                ser.close()
                ser.open()
                ser.write("ati")
                time.sleep(3)
                read_val = ser.read(size=64)
                print
                read_val
                if read_val is not '':
                    print
                    port
            except serial.SerialException:
                continue
            i += 1"""


if __name__ == '__main__':
    print("Serial Port Reader")
    parse_serial_message()
