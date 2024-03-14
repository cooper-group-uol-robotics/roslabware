# external
from typing import Optional
from miscware import BaseValve
import rospy
import serial

# Core
from roslabware_msgs.msg import (
    BaseValveCmd,
    BaseValveStatus,
    BaseValveTask
)
from std_msgs.msg import Bool


class BaseValveRos:
    """
    ROS Wrapper for driver for OptiMax base valve.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        simulation: bool,
    ):

        self.base_valve  = serial.Serial(port=port, baudrate=9600, timeout=None)

        self._prev_msg = None
        self._prev_id = -1

        if simulation == "True":
            self.base_valve.simulation = True

        # Initialize ROS subscriber.
        self.subs = rospy.Subscriber(
            name="/optimax_basevalve_command",
            data_class=BaseValveCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status.
        self.pub = rospy.Publisher(
            name="/optimax_basevalve_status",
            data_class=BaseValveStatus,
            queue_size=10
        )

        rospy.loginfo("Base Valve Driver Started")

        self._task_complete_pub = rospy.Publisher(
            name="/base_valve/task_complete",
            data_class=BaseValveTask,
            queue_size=10
        )

        # Sleeping rate.
        self.rate = rospy.Rate(1)

    def _open_close_long(self, id):
        self.base_valve.write((bytes("vopen", 'utf-8')))
        rospy.loginfo("Open valve message sent.")
        rospy.sleep(60) # Open valve for a long time, so all liquid will drain.
        self.base_valve.write((bytes("vclose", 'utf-8')))
        rospy.loginfo("Close valve message sent.")
        rospy.sleep(15)
        for i in range(10):
            self._task_complete_pub.publish(seq=id, complete=True)

    def _open_close_steps(self, id, num_steps):
        self.base_valve.write((bytes("update_steps", 'utf-8')))
        rospy.sleep(3)
        self.base_valve.write((bytes(f"{num_steps}", 'utf-8')))
        rospy.loginfo("Update steps messages sent.")
        rospy.sleep(3) 
        self.base_valve.write((bytes("vopenclose", 'utf-8')))
        rospy.loginfo("Open valve message sent.")
        rospy.sleep(20) # TODO need a more robust method to know when valve has been opened rather than time.
        for i in range(10):
            self._task_complete_pub.publish(seq=id, complete=True)
    
    # Callback for subscriber.
    def callback_commands(self, msg):

        command = msg.valve_command
        id = msg.seq
        if id > self._prev_id:
            if command == msg.OPEN_CLOSE_LONG:
                rospy.loginfo("Open message received.")
                self._open_close_long(id)
            elif command == msg.OPEN_CLOSE_STEPS:
                rospy.loginfo("Custom open close message received.")
                self._open_close_steps(id, msg.num_steps)
            else:
                rospy.loginfo("Invalid command.")
            self._prev_id = id
            self._prev_msg = command
