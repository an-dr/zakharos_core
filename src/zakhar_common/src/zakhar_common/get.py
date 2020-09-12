import rospy
from zakhar_pycore import zakhar__log as log


def server(name, service, handle, logger=None):
    """
    Parameters
    ----------
    name : str
    service : Any
    handle : fn(req)->resp
    logger : log.Logger or None
    """
    if logger:
        logger.debug("Service \'%s\' is loading..." % name)
    s = rospy.Service(name, service, handle)
    if logger:
        logger.debug("Service \'%s\' is ready" % name)
    return s


def client(name, service, logger=None):
    """
    Parameters
    ----------
    name : str
    service : Any
    logger : log.Logger, optional
    """
    if logger:
        logger.debug("Client \'%s\' is loading..." % name)
    c = rospy.ServiceProxy(name, service)
    if logger:
        logger.debug("Client \'%s\' is ready" % name)
    return c


def publisher(topic_name, data_class, logger=None):
    """
    Parameters
    ----------
    topic_name : str
    data_class : L{Message} class
    logger : log.Logger, optional
    """
    if logger:
        logger.debug("Publisher \'%s\' is loading..." % topic_name)
    p = rospy.Publisher(topic_name,data_class, queue_size=10)
    if logger:
        logger.debug("Publisher \'%s\' is ready" % topic_name)
    return p

def subscriber(topic_name, data_class, callback, callback_args=None, logger=None):
    """
    Parameters
    ----------
    topic_name : str
    data_class : L{Message} class
    callback : fn(msg, cb_args)
    callback_args : any, optional
    logger : log.Logger, optional
    """
    if logger:
        logger.debug("Subscriber of \'%s\' is loading..." % topic_name)
    s = rospy.Subscriber(topic_name, data_class, callback, callback_args)
    if logger:
        logger.debug("Subscriber of \'%s\' is ready" % topic_name)
    return s

