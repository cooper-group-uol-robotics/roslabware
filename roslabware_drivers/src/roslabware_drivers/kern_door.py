# external
from typing import Optional, Union

import rospy
#from pylabware import Kern_Door
from miscware import BalanceDoor

# Core
from roslabware_msgs.msg import (
    KernDoorCmd,
    KernDoorStatus
)


class KernDoorRos:
    """
    ROS wrapper class for controlling Kern PCB Top Balance
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False,
    ):

        self._prev_msg = None

        # Instantiate IKA driver
        self.door = BalanceDoor(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port,
        )

        if simulation == "True":
            self.door.simulation = True

        # Connect to balance
        self.door.connect()

        # Initialize ros subscriber of topic to which commands are published
        self.subs = rospy.Subscriber(
            name="kern_door_Commands",
            data_class=KernDoorCmd,
            callback=self.callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher(
            name="kern_Door_Status",
            data_class=KernDoorStatus,
            queue_size=10
        )

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Kern door Miscware driver started")
        self.door.initialize_device()
        #initialize device
        #self.close_door()

    def open_door(self):
        self.door.connect()
        self.door.open_door()

        #if serial msg received:
        rospy.loginfo("open_door_message_sent_to_miscware")
        self.pub.publish( status = 'Door_Opened')
        rospy.sleep(5)

    def close_door(self):
        self.door.connect()
        self.door.close_door()
        rospy.loginfo("close_door_message_sent_to_miscware")
        #if serial msg received:
        self.pub.publish(status = 'Door_Closed')
        rospy.sleep(5)

    def callback_commands(self, msg):
        message = msg.kern_door_command
        rospy.loginfo("message_received")
        if not message == self._prev_msg:
            if message == msg.OPEN_DOOR:
                rospy.loginfo("open_door_message")
                self.open_door()
                self._prev_msg = message
            elif message == msg.CLOSE_DOOR:
                rospy.loginfo("close_door_message")
                self.close_door()
                self._prev_msg = message
            else:
                rospy.loginfo("Invalid command")
