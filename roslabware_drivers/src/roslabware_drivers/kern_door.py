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

        self.pub = None
        self.tared = False

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

    def open_door(self):
        self.door.open_door()
        #if serial msg received:
        self.pub.publish('Door_Opened')

    def close_door(self):
        self.door.close_door()
        #if serial msg received:
        self.pub.publish('Door_Closed')

    def callback_commands(self, msg):
        message = msg.kern_door_command

        if message == msg.OPEN_DOOR:
            self.open_door()
        elif message == msg.CLOSE_DOOR:
            self.close_door()
        else:
            rospy.loginfo("Invalid command")
