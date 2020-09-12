#!/usr/bin/python3
import click
import sys
import rosnode
import roslaunch
import atexit
import rospy
from time import sleep
from zakhar_common import names

cmds = ("all", "ll", "hl")

nodes_ll = {
    "i2c": ("zakhar_i2c", "i2c.py"),
    "face_platform": ("zakhar_i2c_devices", "face_platform.py"),
    "sensor_platform": ("zakhar_i2c_devices", "sensor_platform.py"),
    "moving_platform": ("zakhar_i2c_devices", "moving_platform.py"),
    "concept_translator" : ("zakhar_mind","concept_translator.py"),
    "sensor_interpreter" : ("zakhar_mind","sensor_interpreter.py"),
}

nodes_hi = {
    "instinct_bird_panic" : ("zakhar_mind","instinct_bird_panic.py"),
    "ego_small_researcher" : ("zakhar_mind","ego_small_researcher.py"),
}

nodes_all = {}
nodes_all.update(nodes_ll)
nodes_all.update(nodes_hi)


nodes_active = []


def close_all():
    print("Closing nodes...")
    for n in nodes_active:
        n.stop()
    sleep(5)


@click.option('--nodes',
              '-n',
              help="Run Zakhar nodes",
              multiple=True,
              type=click.Choice(cmds + tuple(nodes_all.keys())),
              default="all",
              show_default=True)
# @click.argument('extra_nodes', nargs=-1)
@click.command()
def cli(nodes):
    atexit.register(close_all)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()

    if "all" in nodes:
        nodes = nodes_all.keys()
    elif "ll" in nodes:
        nodes = nodes_ll.keys()


    for n in nodes:
        print(f"Starting {n} : {nodes_all[n][0]},{nodes_all[n][1]}")
        new_node = roslaunch.core.Node(nodes_all[n][0], nodes_all[n][1], n)
        process = launch.launch(new_node)
        nodes_active.append(process)
    print(rosnode.get_node_names())
    input("Press Enter to close...")
    close_all()

    print("[   DONE   ]")


if __name__ == '__main__':
    cli(sys.argv[1:])