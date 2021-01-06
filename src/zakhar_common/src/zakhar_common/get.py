import rospy
from rospy.client import _unspecified
from time import time, sleep
from typing import Any


def server(name, service, handle):
    """
    Parameters
    ----------
    name : str
    service : Any
    handle : fn(req)->resp
    """
    rospy.logdebug("Service \'%s\' is loading..." % name)
    s = rospy.Service(name, service, handle)
    rospy.logdebug("Service \'%s\' is ready" % name)
    return s


def client(name, service):
    """
    Parameters
    ----------
    name : str
    service : Any
    """
    rospy.logdebug("Client \'%s\' is loading..." % name)
    c = rospy.ServiceProxy(name, service)
    rospy.logdebug("Client \'%s\' is ready" % name)
    return c


def publisher(topic_name, data_class):
    """
    Parameters
    ----------
    topic_name : str
    data_class : L{Message} class
    """
    rospy.logdebug("Publisher \'%s\' is loading..." % topic_name)
    p = rospy.Publisher(topic_name, data_class, queue_size=10)
    rospy.logdebug("Publisher \'%s\' is ready" % topic_name)
    return p


def subscriber(topic_name, data_class, callback, callback_args=None):
    """
    Parameters
    ----------
    topic_name : str
    data_class : L{Message} class
    callback : fn(msg, cb_args)
    callback_args : any, optional
    """
    rospy.logdebug("Subscriber \'%s\' is loading..." % topic_name)
    s = rospy.Subscriber(topic_name, data_class, callback, callback_args)
    rospy.logdebug("Subscriber \'%s\' is ready" % topic_name)
    return s


def param(name: str, default: Any = _unspecified, timeout_s: int = 0) -> Any:
    """
    Parameters
    ----------
    name : str
    default : Any
    timeout_s : int
        timeout_s > 0  - try to read the parameter during the timeout
        timeout_s == 0 - try to read the parameter one time without a timeout
        timeout < 0 - try to read the parameter until success
    """
    if timeout_s > 0:
        start = time()
        while ((time() - start) < timeout_s):
            try:
                return rospy.get_param(name, default)
            except (rospy.ROSException, KeyError):
                sleep(.1)
    elif timeout_s < 0:
        while 1:
            try:
                return rospy.get_param(name, default)
            except (rospy.ROSException, KeyError):
                sleep(.1)
    else:
        return rospy.get_param(name, default)
