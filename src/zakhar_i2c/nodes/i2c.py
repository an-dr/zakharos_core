#!/usr/bin/env python3
from zakhar_pycore import zakhar__log as log
from zakhar_i2c.node import node


if __name__ == '__main__':
    node.start(log_level=log.INFO)
