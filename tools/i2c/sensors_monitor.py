#!/usr/bin/env python3

from time import sleep
from zakhar_pycore.i2c import read
from zakhar_pycore.dev import sensor_platform

if __name__ == "__main__":
    while (1):
        print("[ ", end="")
        for i in range(7):
            print("0x%x\t" % read(sensor_platform.addr, i), end="")
        print("]")
        sleep(.1)
