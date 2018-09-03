#! /usr/bin/python

import serial, time
import subprocess
from subprocess import call, Popen
from argparse import ArgumentParser
import re
import sys
import unittest

def do_test(test_cmd, port, baudrate=57600):

    databits = serial.EIGHTBITS
    stopbits = serial.STOPBITS_ONE
    parity = serial.PARITY_NONE
    ser = serial.Serial(port, baudrate, databits, parity, stopbits, 100)
    ser.write('\n\n')
    
    finished = False
    while not finished:
        serial_line = ser.readline()
        print(serial_line.replace('\n','')) 

        if "nsh>" in serial_line:
            finished = True
        time.sleep(0.05)

    ser.write('tests %s\n' % test_cmd)

    finished = False
    while not finished:
        serial_line = ser.readline()
        print(serial_line.replace('\n','')) 

        if "PASSED" in serial_line:
            finished = True
            ser.close()

        time.sleep(0.05)

    ser.close()

    return finished


class PX4Tests(unittest.TestCase):
    DEVICE = "/dev/ttyUSB0"
    BAUDRATE = 57600

    def test_perf(self):
        self.assertTrue(do_test("perf", self.DEVICE, self.BAUDRATE))

    def test_adc(self):
        self.assertTrue(do_test("adc", self.DEVICE, self.BAUDRATE))
 
if __name__ == '__main__':
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--device', "-d", nargs='?', default="/dev/ttyUSB0", help='')
    parser.add_argument("--baudrate", "-b", dest="baudrate", type=int, help="Mavlink port baud rate (default=57600)", default=57600)
    args = parser.parse_args()

    print args

    sys.argv = sys.argv[:1]  

    PX4Tests.DEVICE = args.device
    PX4Tests.BAUDRATE = args.baudrate

    unittest.main()
