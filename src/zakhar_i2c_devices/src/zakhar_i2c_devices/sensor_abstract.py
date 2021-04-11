import rospy
from time import sleep
import threading
from zakhar_i2c import ZkI2cDevice
from zakhar_pycore import ros as zr
from typing import Any, List


class ZkSensorParamWeight(object):
    def __init__(self, param_name: str, weight: float):
        self.name = param_name
        self.weight = weight

    def __str__(self):
        return "\"" + self.name + "\":" + str(self.weight)

    def __repr__(self):
        return self.__str__()


class ZkSensor():
    def __init__(self,
                 name: str,
                 sensor_platform: ZkI2cDevice,
                 val_min: int = None,
                 val_max: int = None,
                 weights: List[ZkSensorParamWeight] = None):
        self.name = name
        self.sensor_platform = sensor_platform
        self.termination = False
        self.poll_thread = None  # type: Any[threading.Thread, None]
        self.poll_freq = 0
        self.emotion_core_enabled = False

        if val_min:
            self.val_min = val_min
        else:
            self.val_min = 0

        if val_max:
            self.val_max = val_max
        else:
            self.val_max = 0

        self.weights = weights

    def publish(self, dtype="", valA=0, valB=0, valC=0, valD=0, valString="", msg_note=""):
        self.sensor_platform.publish(dtype=dtype,
                                     valA=valA,
                                     valB=valB,
                                     valC=valC,
                                     valD=valD,
                                     valString=valString,
                                     msg_note=msg_note)

    def __poll_loop(self):
        rospy.loginfo("Polling start")
        while 1:
            if self.termination:
                break
            self.poll_once()
            sleep(1 / self.poll_freq)
        self.poll_freq = 0
        rospy.loginfo("Polling end")

    def start(self, freq_hz=10):
        """
        Parameters
        ----------
        freq : int, optional
            Hz, by default 10
        """
        self.termination = False
        self.poll_freq = freq_hz
        self.emotion_core_enabled = rospy.get_param('/emotion_core_enable', default=False)
        if self.emotion_core_enabled and self.weights:
            rospy.logwarn("Emotion Core enabled!")
            rospy.wait_for_service(zr.services.EMOTIONCORE_DATADSC, timeout=10)
            self.sensor_platform.send_sensorDataDescriptor(sensor_name=self.name,
                                                           val_max=self.val_max,
                                                           val_min=self.val_min,
                                                           weights_json=self.get_weight_json_string())
        self.start_setup()
        self.poll_thread = threading.Thread(name='[%s]polling' % self.name, target=self.__poll_loop)
        self.poll_thread.setDaemon(True)
        self.poll_thread.start()

    def get_weight_json_string(self) -> str:
        res = "{"
        for i in self.weights:
            res += str(i) + ","
        return res[:-1] + "}"

    def stop(self):
        self.termination = True

    def poll_once(self):
        raise NotImplementedError()

    def start_setup(self):
        raise NotImplementedError()
