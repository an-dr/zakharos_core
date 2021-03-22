import rospy
from os.path import basename, splitext
from zakhar_msgs import msg, srv
from zakhar_pycore import ros as zros

# TODO: move data interpretation from sensors to here


class Data2ConceptInterpreter:
    def __init__(self):
        self.name = splitext(basename(__file__))[0]
        self.publisher_to_egos = None
        self.subscriber_to_devices = None
        self.client_of_emotion_core_write = None
        self.client_of_emotion_data_descriptor = None

    def publish(self, symbol, modifier):
        to_send = msg.PerceptionConcept()
        to_send.symbol = symbol
        to_send.modifier = modifier
        self.publisher_to_egos.publish(to_send)
        rospy.loginfo("Sent [%s, %s]" % (symbol, modifier))

    def sensor_data_handler_photoresistor(self, sensor_data: msg.SensorData):
        if sensor_data.sensor_type == "photoresistor":
            if sensor_data.valString == "bird":
                self.publish("bird", "above")
            if sensor_data.valA != 0:  # TODO: replace value to symbol like (dark, light, darker, lighter, etc.)
                self.publish("light", str(sensor_data.valA))

    def sensor_data_handler_sound_distance(self, sensor_data: msg.SensorData):
        if sensor_data.sensor_type == "sound_distance":
            if len(sensor_data.valString):
                self.publish("obstacle", str(sensor_data.valString))

    def sensor_data_handler(self, data):
        rospy.loginfo(
            "Got SensorData [sensor_type: %s, valA: 0x%x, valB: 0x%x, valC: 0x%x, valD: 0x%x, valString: %s, msg: \'%s\'"
            % (data.sensor_type, data.valA, data.valB, data.valC, data.valD, data.valString, data.message))
        write_msg = srv.EmotionCoreWriteRequest()
        write_msg.sensor_name = data.sensor_type
        write_msg.value = data.valA
        try:
            rospy.logwarn(f"Data2ConceptInterpreter: Emotion core write: {data.sensor_type}:{data.valA}")
            self.client_of_emotion_core_write(write_msg)
        except rospy.ServiceException:
            rospy.logwarn("Service %s did not process request. Is it running?" % zros.services.EMOTIONCORE_WRITE)
        self.sensor_data_handler_photoresistor(data)
        self.sensor_data_handler_sound_distance(data)

    def start(self):
        rospy.logdebug("Node \'%s\' is starting..." % self.name)
        rospy.init_node(self.name, anonymous=False)

        self.publisher_to_egos = zros.get.publisher(topic_name=zros.topics.MAIN_SENSOR_INTERPRETER,
                                                    data_class=msg.PerceptionConcept)
        self.subscriber_to_devices = zros.get.subscriber(topic_name=zros.topics.SENSOR_DATA,
                                                         data_class=msg.SensorData,
                                                         callback=self.sensor_data_handler)
        self.client_of_emotion_core_write = zros.get.client(name=zros.services.EMOTIONCORE_WRITE,
                                                            service=srv.EmotionCoreWrite)
        self.client_of_emotion_data_descriptor = zros.get.client(name=zros.services.EMOTIONCORE_DATADSC,
                                                                 service=srv.EmotionCoreDataDescriptor)
        rospy.loginfo("[  DONE  ] Node \'%s\' is ready..." % self.name)


def start():
    obj = Data2ConceptInterpreter()
    obj.start()
    rospy.spin()
