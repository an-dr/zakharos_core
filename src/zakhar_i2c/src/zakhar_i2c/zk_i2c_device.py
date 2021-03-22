import threading
import time
import rospy
from typing import Union
import json
import zakhar_pycore as z
from zakhar_msgs import srv, msg

REG_CMD = 0
REG_MODE = 1

CMD_NONE = 0xFF
CMD_DONE = 0x00
CMD_STOP = 0xA0

CONFIG_MAX_RETRY = 16


class ZkI2cDevice(object):
    def __init__(self, dev_name, address, poll_dict={}):
        """
        [summary]

        Parameters
        ----------
        dev_name : str
        address : int
        req_poll_list : dict
            {
                (register) : (poll freq, hz),
                (register) : (poll freq, hz),
                ...
            }
            e.g.:
            {
                0x1: 10,
                0x2: 1
            }
        log_level : int, optional
        """

        self._name = dev_name
        self._address = address
        self.subscriber_to_cmd = None
        self.client_i2c = None
        self.client_i2c_lock = threading.Lock()
        self.client_EmotionCoreDataDescriptor = None
        self.publisher_of_sensor_data = None
        self._started = False
        self.poll_dict = poll_dict
        self.poll_threads = []

    def subscriber_to_cmd_callback(self, data: msg.DeviceCmd):
        if ((data.target == self._name) or (data.target == 'all')):
            rospy.loginfo("Send to %s: 0x%x(0x%x). [%s]" % (data.target, data.cmd, data.arg, data.message))
            if data.arg != 0:
                self.write(reg=1, val=data.arg)
            self.write(reg=0, val=data.cmd)

    def publish(self, dtype="", valA=0, valB=0, valC=0, valD=0, valString="", msg_note=""):
        to_send = msg.SensorData()
        to_send.sensor_type = dtype
        to_send.valA = valA
        to_send.valB = valB
        to_send.valC = valC
        to_send.valD = valD
        to_send.valString = valString
        to_send.message = msg_note
        rospy.loginfo("%s published: %s, %d, %d, %d, %d, %s (%s)" %
                      (self._name, to_send.sensor_type, to_send.valA, to_send.valB, to_send.valC, to_send.valD,
                       to_send.valString, to_send.message))
        self.publisher_of_sensor_data.publish(to_send)

    def send_sensorDataDescriptor(self, sensor_name: str, val_max: int, val_min: int, weights_json: str):
        m = srv.EmotionCoreDataDescriptorRequest()
        m.sensor_name = sensor_name
        m.val_max = val_max
        m.val_min = val_min
        if isinstance(weights_json, str):
            m.weights_json = weights_json
        else:
            m.weights_json = weights_json.dumps()
        return self.client_EmotionCoreDataDescriptor(m)

    def write(self, reg, val):
        rospy.logdebug("write(0x%x, 0x%x) to 0x%x" % (reg, val, self._address))
        if not self.client_i2c:
            raise rospy.ServiceException
        try:
            self.client_i2c_lock.acquire(blocking=True)
            resp = self.client_i2c("w", self._address, reg, val)
            self.client_i2c_lock.release()
            return resp.reg_value
        except rospy.ServiceException as e:
            if self.client_i2c_lock.locked:
                self.client_i2c_lock.release()
            rospy.logerr("Service call failed: %s" % e)

    def read(self, reg):
        rospy.logdebug("read(%x)" % (reg))
        if not self.client_i2c:
            raise rospy.ServiceException
        try:
            self.client_i2c_lock.acquire(blocking=True)
            resp = self.client_i2c("r", self._address, reg, 0)
            self.client_i2c_lock.release()
            return resp.reg_value
        except rospy.ServiceException as e:
            if self.client_i2c_lock.locked:
                self.client_i2c_lock.release()
            rospy.logerr("Service call failed: %s" % e)

    def mode(self):
        s = self.read(REG_MODE)
        return s

    def cmd(self, val):
        rospy.logdebug("Send a command :%s" % hex(val))
        self.write(REG_CMD, val)

    def cmd_stop(self):
        self.cmd(0xA0)

    def poll_process(self, register, freq):
        while (1):
            try:
                d = self.read(reg=register)
            except Exception as e:
                rospy.logerr(str(e))
                d = None
            if d is None:
                d = -1
                continue
            self.publish(dtype=self._name, valA=register, valB=d, valC=0, valD=0, valString="", msg_note="A:reg, B:val")
            time.sleep(1 / freq)

    def _start_poll(self):
        for poll in self.poll_dict:
            t = threading.Thread(target=self.poll_process, args=(poll, self.poll_dict[poll]))
            t.daemon = True
            t.start()
            self.poll_threads.append(t)

    def run(self):
        rospy.logdebug("Node \'%s\' is starting..." % self._name)
        rospy.init_node(self._name, anonymous=False)
        ###
        rospy.loginfo("  Waiting for the service %s", z.ros.services.I2C)

        rospy.wait_for_service(z.ros.services.I2C, timeout=10)
        ###
        self.client_i2c = z.ros.get.client(name=z.ros.services.I2C, service=srv.I2c)
        self.client_EmotionCoreDataDescriptor = z.ros.get.client(name=z.ros.services.EMOTIONCORE_DATADSC,
                                                                 service=srv.EmotionCoreDataDescriptor)
        self.subscriber_to_cmd = z.ros.get.subscriber(topic_name=z.ros.topics.DEVICECMD,
                                                      data_class=msg.DeviceCmd,
                                                      callback=self.subscriber_to_cmd_callback)
        self.publisher_of_sensor_data = z.ros.get.publisher(topic_name=z.ros.topics.SENSOR_DATA,
                                                            data_class=msg.SensorData)
        self._start_poll()
        ###
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self._name)

    def start(self):
        self.run()
        rospy.spin()
