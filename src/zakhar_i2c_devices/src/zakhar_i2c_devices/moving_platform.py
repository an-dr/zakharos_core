from os.path import basename, splitext
from zakhar_pycore import zakhar__log as log
from zakhar_i2c import ZkI2cDevice

ADDR_MOTORS = 0x2a  # bus address
CMD_FORWARD = 119
CMD_BACKWARD = 115
CMD_LEFT = 97
CMD_RIGHT = 0x64
CMD_STOP = 32
CMD_SHIVER = 0x71

DEV_NAME = splitext(basename(__file__))[0]  # filename without extension
moving_platform = ZkI2cDevice(dev_name=DEV_NAME, address=ADDR_MOTORS, log_level=log.INFO)
