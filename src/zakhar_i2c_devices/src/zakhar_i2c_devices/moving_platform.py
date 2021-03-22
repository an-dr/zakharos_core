from zakhar_i2c import ZkI2cDevice
from zakhar_pycore import dev as zdev

moving_platform = ZkI2cDevice(dev_name=zdev.moving_platform.name, address=zdev.moving_platform.addr)
