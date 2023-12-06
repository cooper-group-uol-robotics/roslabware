# external
# written by satheesh

from typing import Optional, Union

import rospy
import serial

# Core
from roslabware_msgs.msg import (
    SashDoorCmd,
    SashDoorStatus,
    SashDoorTask,
    KernDoorCmd,
    KernDoorStatus,
    KernDoorTask
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

        self._sash_door_prev_msg = None
        self._balance_door_prev_msg = None
        
        ##### Publishers and subcribers for sash door #####

        # Initialize ros subscriber of topic to which commands are published
        self.sash_door_subs = rospy.Subscriber(
            name="/sash_door_command",
            data_class=SashDoorCmd,
            callback=self.sash_door_callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.sash_door_pub = rospy.Publisher(
            name="/sash_door_status",
            data_class=SashDoorStatus,
            queue_size=10
        )

        self._sash_door_task_complete_pub = rospy.Publisher(
            name='/sash_door/task_complete',
            data_class=SashDoorTask,
            queue_size=10
        )
        
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
            name="/kern_door/task_complete",
            data_class=KernDoorTask,
            queue_size=10
        )

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Sash and balance door driver started.")

    ###### Sash door methods #####

    def open_sash_door(self, id):
        self.door.write((bytes("sopen", "utf-8")))
        rospy.loginfo("Open sash door message sent to device controller.")
        rospy.sleep(64)
        self.sash_door_pub.publish(status="door_open")
        for i in range(10):
            self._sash_door_task_complete_pub(seq=id, complete=True)

    def close_sash_door(self, id):
        self.door.write((bytes("sclose", 'utf-8')))
        rospy.loginfo("Close sash door message sent to device controller.")
        rospy.sleep(66)
        self.sash_door_pub.publish(status="door_closed")
        for i in range(10):
            self._sash_door_task_complete_pub(seq=id, complete=True)

    def sash_door_callback_commands(self, msg):
        message = msg.sash_door_command
        id = msg.seq
        rospy.loginfo("Sash door message received.")
        if message != self._sash_door_prev_msg:
            if message == msg.OPEN_DOOR:
                self.sash_door_process_complete = False
                rospy.loginfo("Open sash message received.")
                self.open_sash_door(id)
            elif message == msg.CLOSE_DOOR:
                self.sash_door_process_complete = False
                rospy.loginfo("Close sash message received.")
                self.close_sash_door(id)
            else:
                rospy.loginfo("Invalid command.")
            self._sash_door_prev_msg = message

    ##### Balance door methods ######

    def open_balance_door(self, id):
        self.door.write((bytes("bopen", "utf-8")))
        rospy.loginfo("Open balance door message sent to device controller.")
        rospy.sleep(8)
        self.balance_door_pub.publish(status="door_open")
        for i in range(10):
            self._balance_door_task_complete_pub(seq=id, complete=True)

    def close_balance_door(self, id):
        self.door.write((bytes("bclose", 'utf-8')))
        rospy.loginfo("Close balance door message sent to device controller.")
        rospy.sleep(8)
        self.balance_door_pub.publish(status="door_closed")
        for i in range(10):
            self._balance_door_task_complete_pub(seq=id, complete=True)

    def balance_door_callback_commands(self, msg):
        message = msg.kern_door_command
        id = msg.seq
        rospy.loginfo("Balance door message received.")
        if message != self._balance_door_prev_msg:
            if message == msg.OPEN_DOOR:
                self.balance_door_process_complete = False
                rospy.loginfo("Open balance door message received.")
                self.open_balance_door(id)
            elif message == msg.CLOSE_DOOR:
                self.balance_door_process_complete = False
                rospy.loginfo("Close balance door message received.")
                self.close_balance_door(id)
            else:
                rospy.loginfo("Invalid command.")
            self._balance_door_prev_msg = message