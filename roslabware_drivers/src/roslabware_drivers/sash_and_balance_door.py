# external
# written by satheesh

from typing import Optional, Union

import rospy
import serial

# Core
from roslabware_msgs.msg import (
    sashDoorCmd,
    sashDoorStatus,
    KernDoorCmd,
    KernDoorStatus
)
from std_msgs.msg import Bool

class SashBalanceDoorRos:
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
        self.door = serial.Serial(port=port, baudrate=9600, timeout=None)

        self.sash_door_process_complete = False
        self.balance_door_process_complete = False
        self._sash_door_prev_msg = None
        self._balance_door_prev_msg = None
        
        ##### Publishers and subcribers for sash door #####

        # Initialize ros subscriber of topic to which commands are published
        self.sash_door_subs = rospy.Subscriber(
            name="/sash_door_command",
            data_class=sashDoorCmd,
            callback=self.sash_door_callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.sash_door_pub = rospy.Publisher(
            name="/sash_door_status",
            data_class=sashDoorStatus,
            queue_size=10
        )

        self._sash_door_task_complete_pub = rospy.Publisher(
            '/sash_door/task_complete',
            Bool,
            queue_size=1)
        
        ##### Publishers and subcribers for balance door #####

        # Initialize ros subscriber of topic to which commands are published
        self.balance_door_subs = rospy.Subscriber(
            name="/kern_door_command",
            data_class=KernDoorCmd,
            callback=self.balance_door_callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.balance_door_pub = rospy.Publisher(
            name="/kern_door_status",
            data_class=KernDoorStatus,
            queue_size=10
        )

        self._balance_door_task_complete_pub = rospy.Publisher(
            '/kern_door/task_complete',
            Bool,
            queue_size=1)

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Sash and balance door driver started.")
        while not rospy.is_shutdown():
            self._sash_door_task_complete_pub.publish(self.process_complete)
            self._balance_door_task_complete_pub.publish(self.process_complete)
            rospy.sleep(5)

    ###### Sash door methods #####

    def open_sash_door(self):
        self.door.write((bytes("sopen", 'utf-8')))
        rospy.loginfo("Open sash door message sent to device controller.")
        rospy.sleep(64)
        self.sash_door_pub.publish(status = 'door_open')
        self.sash_door_process_complete = True

    def close_sash_door(self):
        self.door.write((bytes("sclose", 'utf-8')))
        rospy.loginfo("Close sash door message sent to device controller.")
        rospy.sleep(66)
        self.sash_door_pub.publish(status = 'door_closed')
        self.sash_door_process_complete = True

    def sash_door_callback_commands(self, msg):
        message = msg.sash_door_command
        rospy.loginfo("Sash door message received.")
        if message != self._prev_msg:
            if message == msg.OPEN_DOOR:
                self.sash_door_process_complete = False
                rospy.loginfo("Open sash message received.")
                self.open_sash_door()
            elif message == msg.CLOSE_DOOR:
                self.sash_door_process_complete = False
                rospy.loginfo("Close sash message received.")
                self.close_sash_door()
            else:
                rospy.loginfo("Invalid command.")
            self._sash_door_prev_msg = message

    ##### Balance door methods ######

    def open_balance_door(self):
        self.door.write((bytes("bopen", 'utf-8')))
        rospy.loginfo("Open balance door message sent to device controller.")
        rospy.sleep(7)
        self.balance_door_pub.publish(status = 'door_open')
        self.balance_door_process_complete = True

    def close_balance_door(self):
        self.door.write((bytes("bclose", 'utf-8')))
        rospy.loginfo("Close balance door message sent to device controller.")
        rospy.sleep(7)
        self.balance_door_pub.publish(status = 'door_closed')
        self.balance_door_process_complete = True

    def balance_door_callback_commands(self, msg):
        message = msg.kern_door_command
        rospy.loginfo("Balance door message received.")
        if message != self._balance_door_prev_msg:
            if message == msg.OPEN_DOOR:
                self.balance_door_process_complete = False
                rospy.loginfo("Open balance door message received.")
                self.open_balance_door()
            elif message == msg.CLOSE_DOOR:
                self.balance_door_process_complete = False
                rospy.loginfo("Close balance door message received.")
                self.close_balance_door()
            else:
                rospy.loginfo("Invalid command.")
            self._balance_door_prev_msg = message