from os.path import basename, splitext
from zakhar_i2c import ZkI2cDevice
from zakhar_pycore.constants import ADDR

CMD_FORWARD = 119
CMD_BACKWARD = 115
CMD_LEFT = 97
CMD_RIGHT = 0x64
CMD_STOP = 32
CMD_SHIVER = 0x71

DEV_NAME = splitext(basename(__file__))[0]  # filename without extension
moving_platform = ZkI2cDevice(dev_name=DEV_NAME, address=ADDR.I2C.MOTORS)
