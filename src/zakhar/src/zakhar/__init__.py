import rosnode
import roslaunch
from time import sleep
from zakhar_common import names

node = roslaunch.core.Node('zakhar_i2c', 'i2c.py', name=names.NODE_I2C_SERVER)
launch = roslaunch.scriptapi.ROSLaunch()
launch.start()

process = launch.launch(node)
print (process.is_alive())

sleep(5)
print(rosnode.get_node_names())
process.stop()

import os
print(os.path.basename(__file__))

