import rospy
from os.path import basename, splitext
from zakhar_i2c import ZkI2cDevice
from zakhar_pycore import zakhar__log as log
from time import sleep
from .sensor_photo_resistor import SensorPhotoResistor

ADDR_EYE = 0x2b
REG_VAL_LO = 0x02
REG_VAL_HI = 0x03

class I2cSensorPlatform(ZkI2cDevice):
    def __init__(self, dev_name, address, poll_dict={},
                log_level=log.INFO):
        ZkI2cDevice.__init__(self,dev_name,address,poll_dict,log_level)
        self.sensor_photo_res = SensorPhotoResistor(self, log_level)

    def start(self, log_level):
        super(I2cSensorPlatform, self).run(log_level=log_level)
        self.sensor_photo_res.start_polling(freq=10)
        sleep(2)
        self.sensor_photo_res.start_corr_measurements(500, [0, 0, 255, 0, 0], 0.7)
        rospy.spin()



DEV_NAME = splitext(basename(__file__))[0]  # filename without extension
sensor_platform = I2cSensorPlatform(dev_name=DEV_NAME,
                                    address=ADDR_EYE,
                                    #   poll_dict={0x02:.2, 0x03:.2},
                                    log_level=log.INFO)
