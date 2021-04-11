#!/usr/bin/env python3

from time import sleep
from zakhar_pycore.i2c import read
from zakhar_pycore.dev import moving_platform


if __name__ == "__main__":
    while (1):
        print("[ ", end="")
        for i in range(7):
            print("0x%x\t" % read(moving_platform.addr, i), end="")
        print("]")
        sleep(.3)
