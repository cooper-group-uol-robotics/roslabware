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
            callback=self.sash_door_callback_commands,
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

    ###### Sash door methods #####

    def open_valve(self, id):
        self.valve.write((bytes("O", "utf-8")))
        rospy.loginfo("Open valve message sent to device controller.")
        rospy.sleep(70)
        self.filtration_pub.publish(status="valve_opened")
        for i in range(10):
            self.filtration_task_complete_pub.publish(bool(True))

    def close_valve(self, id):
        self.valve.write((bytes("C", 'utf-8')))
        rospy.loginfo("Close valve message sent to device controller.")
        rospy.sleep(70)
        self.filtration_pub.publish(status="valve_closed")
        for i in range(10):
            self.filtration_task_complete_pub.publish(bool(True))

    def sash_door_callback_commands(self, msg:FiltrationCmd):
        self.filtration_task_complete_pub.publish(bool(False))
        message = msg.filtration_command
        id = msg.seq
        rospy.loginfo("Sash door message received.")
        if message != self._filtration_prev_msg:
            if message == msg.OPEN_VALVE:
                rospy.loginfo("Open sash message received.")
                self.open_valve(id)
            elif message == msg.CLOSE_VALVE:
                rospy.loginfo("Close sash message received.")
                self.close_valve(id)
            else:
                rospy.loginfo("Invalid command.")
            self._filtration_prev_msg = message

