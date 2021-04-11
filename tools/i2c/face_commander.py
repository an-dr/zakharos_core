#!/usr/bin/env python3

from zakhar_pycore.i2c import write
from zakhar_pycore.constants import ADDR
from zakhar_pycore.dev import face_module
from zakhar_pycore.helpers.os import wait_key

if __name__ == "__main__":
    while (1):
        print("Press the key (Esc to exit): ")
        key = wait_key()
        if key == 0x1b:
            print("Exit")
            exit()
        write(face_module.addr, 0, key)  # write
