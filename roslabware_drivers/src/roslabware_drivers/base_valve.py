# external
from typing import Optional
from miscware import BaseValve
import rospy
import serial

# Core
from roslabware_msgs.msg import (
    BaseValveCmd,
    BaseValveStatus)
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

        self.process_complete = False
        self.previous_command = None

        if simulation == "True":
            self.base_valve.simulation = True

        # Initialize ROS subscriber.
        self.subs = rospy.Subscriber(
            name="/Optimax_BaseValve_Commands",
            data_class=BaseValveCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status.
        self.pub = rospy.Publisher(
            name="/Optimax_BaseValve_Status",
            data_class=BaseValveStatus,
            queue_size=10
        )

        rospy.loginfo("Base Valve Driver Started")

        self._task_complete_pub = rospy.Publisher(
            '/base_valve/task_complete',
            Bool,
            queue_size=1)

        # Sleeping rate.
        self.rate = rospy.Rate(1)

        # Publish status of the valve.
        while not rospy.is_shutdown():
            self._task_complete_pub.publish(True)
            rospy.sleep(5)

    def _open_valve(self):
        self.base_valve.write((bytes("vopen", 'utf-8')))
        rospy.loginfo("open_valve_message_sent_to_miscware")
        rospy.sleep(8) # TODO need a more robust method to know when valve has been opened rather than time.
        self.process_complete = True

    def _close_valve(self):
        self.base_valve.write((bytes("vclose", 'utf-8')))
        rospy.loginfo("close_valve_message_sent_to_miscware")
        rospy.sleep(8) # TODO need a more robust method to know when valve has been opened rather than time.
        self.process_complete = True
    

    # Callback for subscriber.
    def callback_commands(self, msg):

        command = msg.valve_command
        if not command == self._prev_msg:
            if command == msg.OPEN:
                rospy.loginfo("Open message received.")
                self.process_complete = False
                self._open_valve()
            elif command == msg.CLOSE:
                rospy.loginfo("Close message received.")
                self.process_complete = False
                self._close_valve()
            else:
                rospy.loginfo("Invalid command.")
            self._prev_msg = command
