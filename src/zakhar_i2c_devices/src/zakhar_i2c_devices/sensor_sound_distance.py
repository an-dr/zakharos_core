import rospy
from .sensor_abstract import ZkSensor, ZkSensorParamWeight
from zakhar_pycore import dev as zdev

CONFIG_PRINT_VAL = True
CONFIG_TRIG_DISTANCE = 0x05

MSG_TRIG = "obstacle"


class ZkSensorSoundDistance(ZkSensor):
    def __init__(self, sensor_platform):
        ZkSensor.__init__(self,
                          name="sound_distance",
                          sensor_platform=sensor_platform,
                          val_min=0,
                          val_max=0xffff,
                          weights=[
                              ZkSensorParamWeight("adrenaline", -1),
                              ZkSensorParamWeight("cortisol", -1),
                          ])
        self.last_val = [0xffff, 0xffff, 0xffff]

    def read_distance(self):
        try:
            val_l = self.sensor_platform.read(zdev.sensor_platform.reg.DIST_L)
            val_c = self.sensor_platform.read(zdev.sensor_platform.reg.DIST_C)
            val_r = self.sensor_platform.read(zdev.sensor_platform.reg.DIST_R)
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
        self.publish(dtype=self.name, valA=self.last_val[0], valB=self.last_val[1], valC=self.last_val[2])
        # data handling
        trigs = ""
        if self.last_val[0] <= CONFIG_TRIG_DISTANCE:
            trigs = trigs + "l"
        if self.last_val[1] <= CONFIG_TRIG_DISTANCE:
            trigs = trigs + "c"
        if self.last_val[2] <= CONFIG_TRIG_DISTANCE:
            trigs = trigs + "r"

        if len(trigs):  # if any trigger
            rospy.loginfo("Trigger: %s", trigs)
            self.publish(dtype=self.name, valString=trigs)
