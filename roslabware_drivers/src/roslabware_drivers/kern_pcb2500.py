# external
from typing import Optional, Union

import rospy
from pylabware import PCB2500

# Core
from roslabware_msgs.msg import (
    KernPCB2500Cmd,
    KernPCB2500Reading
)


class PCB2500Ros:
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
        self.balance = PCB2500(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port,
        )

        if simulation == "True":
            self.balance.simulation = True

        # Connect to balance
        self.balance.connect()
        self.balance.initialize_device()

        # Initialize ros subscriber of topic to which commands are published
        self.subs = rospy.Subscriber(
            name="kern_PCB2500_Commands",
            data_class=KernPCB2500Cmd,
            callback=self.callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher(
            name="kern_PCB2500_Readings",
            data_class=KernPCB2500Reading,
            queue_size=10
        )

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Kern PCB2500 pylabware driver started")

    def tare_balance(self):
        self.tared = True
        rospy.sleep(2)

        self.balance.tare_balance()
        rospy.loginfo("Zeroing Balance")

        rospy.sleep(2)
        self.tared = False

    def get_stable_mass(self):
        self.pub.publish(float(self.balance.get_stable_mass()[0]))

    def get_mass(self):
        self.pub.publish(float(self.balance.get_mass()[0]))

    def callback_commands(self, msg):
        message = msg.kern_command

        if message == msg.TARE_BALANCE:
            self.tare_balance()
        elif message == msg.GET_MASS:
            self.get_mass()
        elif message == msg.GET_MASS_STABLE:
            self.get_stable_mass()
        else:
            rospy.loginfo("Invalid command")
