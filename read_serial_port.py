#!/usr/bin/env python

import serial
import time
import binascii
import codecs
import struct


def parse_serial_message():
    # write
    ser = serial.Serial('COM3')  # open serial port
    print("Port name: %s" % ser.name)  # check which port was really used
    # ser.write(b'hello')  # write a string
    ser.close()  # close port
    #read
    with serial.Serial('COM3', 19200, timeout=1) as ser:
        # read a simple line from serial port
        # x = ser.read()  # read one byte
        # s = ser.read(10)  # read up to ten bytes (timeout)
        line = ser.readline() # read a '\n' terminated line
        print(line)
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
        print("latitude: ",struct.unpack('f', latitude))

        longitude = line[25:29]  # active satellites
        print(longitude)
        print("longitude: ",struct.unpack('f', longitude))

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

        status1 = line[40:42]  # active satellites
        print(status1)
        print("status1: ",int.from_bytes(status1, byteorder='big', signed=True))

        status2 = line[42:44]  # active satellites
        print(status2)
        print("status2: ",int.from_bytes(status2, byteorder='big'))

        trigger_distance = line[44:48]  # active satellites
        print(trigger_distance)
        print("Trigger_Distance: ",int.from_bytes(trigger_distance, byteorder='big'))

        longitudinal_acceleration_g	 = line[48:50]
        print(longitudinal_acceleration_g)
        print("longitudinal_acceleration_g: ",int.from_bytes(longitudinal_acceleration_g, byteorder='big'))

        lateral_acceleration_g = line[50:52]
        print(lateral_acceleration_g)
        print("lateral_acceleration_g: ",int.from_bytes(lateral_acceleration_g, byteorder='big'))

        distance = line[52:56]
        print(distance)
        print("distance: ",struct.unpack('f', distance))

        trigger_time = line[56:58]
        print(trigger_time)
        print("trigger_time: ", int.from_bytes(trigger_time, byteorder='big'))

        trigger_speed_knots = line[58:60]
        print(trigger_speed_knots)
        print("trigger_speed_knots: ", int.from_bytes(trigger_speed_knots, byteorder='big'))

        Speed_Quality_kmh = line[60:62]
        print(Speed_Quality_kmh)
        print("Speed_Quality_kmh: ", int.from_bytes(Speed_Quality_kmh, byteorder='big'))

        # vbox_serial = line[44:46] # active satellites
        # print(vbox_serial)
        # print("vbox_serial: ",int.from_bytes(vbox_serial, byteorder='big'))



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
