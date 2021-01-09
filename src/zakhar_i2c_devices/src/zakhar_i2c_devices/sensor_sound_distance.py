import rospy
from .sensor_abstract import ZkSensor
from zakhar_pycore.constants import REGS

CONFIG_PRINT_VAL = True
CONFIG_TRIG_DISTANCE = 0x05

MSG_SENSOR_VALUE = "distances"
MSG_TRIG = "obstacle"


class ZkSensorSoundDistance(ZkSensor):
    def __init__(self, sensor_platform):
        ZkSensor.__init__(self, name="sound_dist", sensor_platform=sensor_platform)
        self.last_val = [0xffff, 0xffff, 0xffff]

    def read_distance(self):
        try:
            val_l = self.sensor_platform.read(REGS.SENSOR_PLATFORM.DIST_L)
            val_c = self.sensor_platform.read(REGS.SENSOR_PLATFORM.DIST_C)
            val_r = self.sensor_platform.read(REGS.SENSOR_PLATFORM.DIST_R)
        except rospy.ServiceException:
            return [0xffff, 0xffff, 0xffff]
        return [val_l, val_c, val_r]

    def start_setup(self):
        pass

    def poll_once(self):
        # read
        self.last_val = self.read_distance()
        if CONFIG_PRINT_VAL:
            rospy.loginfo("Distances : " + str(self.last_val))
        # publishing
        self.publish(dtype=MSG_SENSOR_VALUE, valA=self.last_val[0], valB=self.last_val[1], valC=self.last_val[2])
        # data handling
        trigs_bin = 0b000
        if self.last_val[0] <= CONFIG_TRIG_DISTANCE:
            trigs_bin |= (1 << 0)
        if self.last_val[1] <= CONFIG_TRIG_DISTANCE:
            trigs_bin |= (1 << 1)
        if self.last_val[2] <= CONFIG_TRIG_DISTANCE:
            trigs_bin |= (1 << 2)

        if trigs_bin:
            self.publish(dtype=MSG_TRIG, valA=trigs_bin)
