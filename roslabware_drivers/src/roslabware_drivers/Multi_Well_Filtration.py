# external
# written by satheesh

from typing import Optional, Union

import rospy
import serial

# Core
from roslabware_msgs.msg import (FiltrationCmd, FiltrationStatus
)
from std_msgs.msg import Bool

class FiltrationRos:
    """
    ROS wrapper and python driver class for controlling Fumehood sash door
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False,
    ):

        # Instantiate driver
        self.valve = serial.Serial(port=port, baudrate=9600, timeout=None)

        self._filtration_prev_msg = None
        
        ##### Publishers and subcribers for sash door #####

        # Initialize ros subscriber of topic to which commands are published
        self.filtration_subs = rospy.Subscriber(
            name="/filtration_command",
            data_class=FiltrationCmd,
            callback=self.filtration_callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.filtration_pub = rospy.Publisher(
            name="/filtration_status",
            data_class=FiltrationStatus,
            queue_size=10
        )

        self.filtration_task_complete_pub = rospy.Publisher(
            name='/filtration/task_complete',
            data_class=Bool,
            queue_size=10
        )

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(1)

        rospy.loginfo("filtration valve driver started.")

        rospy.sleep(2)

        for i in range(10):
            self.filtration_task_complete_pub.publish(bool(False))

    ###### Sash door methods #####

    def open_valve(self, id):
        self.valve.write((bytes("O", "utf-8")))
        rospy.sleep(0.2)
        response = self.valve.readline().decode('utf-8').strip()
        rospy.loginfo(f"Response from controller: {response}.")
        self.filtration_pub.publish(seq=id, status=str(response))
        for i in range(10):
            self.filtration_task_complete_pub.publish(bool(True))

    def close_valve(self, id):
        self.valve.write((bytes("C", 'utf-8')))
        rospy.sleep(0.2)
        response = self.valve.readline().decode('utf-8').strip()
        rospy.loginfo(f"Response from controller: {response}.")
        self.filtration_pub.publish(seq=id, status=str(response))
        for i in range(10):
            self.filtration_task_complete_pub.publish(bool(True))

    def filtration_callback_commands(self, msg:FiltrationCmd):
        self.filtration_task_complete_pub.publish(bool(False))
        message = msg.filtration_command
        id = msg.seq
        rospy.loginfo("Filtration message received.")
        if message != self._filtration_prev_msg:
            if message == msg.OPEN_VALVE:
                rospy.loginfo("Open valve message.")
                self.open_valve(id)
            elif message == msg.CLOSE_VALVE:
                rospy.loginfo("Close valve message.")
                self.close_valve(id)
            else:
                rospy.loginfo("Invalid message.")
            self._filtration_prev_msg = message

