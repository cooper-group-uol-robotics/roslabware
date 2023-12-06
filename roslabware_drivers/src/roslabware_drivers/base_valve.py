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

    def _open_valve(self, id):
        self.base_valve.write((bytes("vopen", 'utf-8')))
        rospy.loginfo("Open valve message sent.")
        rospy.sleep(9) # TODO need a more robust method to know when valve has been opened rather than time.
        for i in range(10):
            self._task_complete_pub(seq=id, complete=True)

    def _close_valve(self, id):
        self.base_valve.write((bytes("vclose", 'utf-8')))
        rospy.loginfo("Close valve message sent.")
        rospy.sleep(9) # TODO need a more robust method to know when valve has been opened rather than time.
        for i in range(10):
            self._task_complete_pub(seq=id, complete=True)
    
    # Callback for subscriber.
    def callback_commands(self, msg):

        command = msg.valve_command
        id = msg.seq
        if command != self._prev_msg:
            if command == msg.OPEN:
                rospy.loginfo("Open message received.")
                self._open_valve(id)
            elif command == msg.CLOSE:
                rospy.loginfo("Close message received.")
                self._close_valve(id)
            else:
                rospy.loginfo("Invalid command.")
            self._prev_msg = command
