# external
# written by satheesh

from typing import Optional, Union

import rospy
import serial

# Core
from roslabware_msgs.msg import (
    sashDoorCmd,
    sashDoorStatus
)
from std_msgs.msg import Bool

class SashDoorRos:
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

        # Instantiate IKA driver
        self.door = serial.Serial(port=port, baudrate=9600, timeout=None)

        self.process_complete = False
        self._prev_msg = None
        
        # Initialize ros subscriber of topic to which commands are published
        self.subs = rospy.Subscriber(
            name="/sash_door_command",
            data_class=sashDoorCmd,
            callback=self.callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher(
            name="/sash_door_status",
            data_class=sashDoorStatus,
            queue_size=10
        )

        self._task_complete_pub = rospy.Publisher(
            '/sash_door/task_complete',
            Bool,
            queue_size=1)

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Sash door driver started")
        while not rospy.is_shutdown():
            self._task_complete_pub.publish(True)
            rospy.sleep(5)
        #initialize device

    def open_door(self):
        self.door.write((bytes("sopen", 'utf-8')))
        rospy.loginfo("open_door_message_sent_to_device_controller")
        self.pub.publish(status = 'door_open')
        rospy.sleep(5)
        self.process_complete = True

    def close_door(self):
        self.door.write((bytes("sclose", 'utf-8')))
        rospy.loginfo("close_door_message_sent_to_device_controller")
        #if serial msg received:
        self.pub.publish(status = 'door_closed')
        rospy.sleep(5)
        self.process_complete = True

    def callback_commands(self, msg):
        message = msg.sash_door_command
        rospy.loginfo("message_received")
        if message != self._prev_msg:
            if message == msg.OPEN_DOOR:
                self.process_complete = False
                rospy.loginfo("open_door_message received")
                self.open_door()
            elif message == msg.CLOSE_DOOR:
                self.process_complete = False
                rospy.loginfo("close_door_message received")
                self.close_door()
            else:
                rospy.loginfo("Invalid command")
            self._prev_msg = message