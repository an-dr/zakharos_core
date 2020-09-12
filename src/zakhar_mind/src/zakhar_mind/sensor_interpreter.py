import rospy
from os.path import basename, splitext
import zakhar_common as com
from zakhar_pycore import zakhar__log as log
import zakhar_i2c_devices as dev
from zakhar_msgs import msg, srv

class SensorInterpreter:
    def __init__(self, name="SensorInterpreter", log_level=log.INFO):
        self.name = name
        self.l = log.get_logger(name=self.name, log_level=log_level)
        self.publisher = None
        self.subscriber = None

    def publish(self, perception_summary, argument):
        to_send = msg.SensorConcept()
        to_send.perception_summary = perception_summary
        to_send.argument = argument
        self.publisher.publish(to_send)
        self.l.debug("Sent [%s, 0x%x]" % (perception_summary, argument))

    def interpretation(self, data):
        self.l.debug(
            "Got SensorData [type: %s, valA: 0x%x, valB: 0x%x, valC: 0x%x, valD: 0x%x, valString: %s, msg: \'%s\'"
            % (data.type, data.valA, data.valB, data.valC, data.valD,
               data.valString, data.message))
        self.publish(data.type, data.valA)

    def start(self):
        self.l.debug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher = com.get.publisher(
            topic_name=com.names.TOPIC_MAIN_SENSOR_INTERPRETER,
            data_class=msg.SensorConcept,
            logger=self.l)
        self.subscriber = com.get.subscriber(topic_name=com.names.TOPIC_SENSORDATA,
                                             data_class=msg.SensorData,
                                             callback=self.interpretation)

        self.l.info("[  DONE  ] Node \'%s\' is ready..." % self.name)


FNAME = splitext(basename(__file__))[0]  # filename without extension
sensor_interpreter = SensorInterpreter(name=FNAME)