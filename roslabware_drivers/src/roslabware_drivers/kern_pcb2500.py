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
        
        self.tared = False
        self._prev_msg = None

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
        rospy.sleep(2)

        # Initialize ros subscriber of topic to which commands are published
        self.subs = rospy.Subscriber(
            name="/kern_pcb2500_command",
            data_class=KernPCB2500Cmd,
            callback=self.callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher(
            name="/kern_pcb2500_reading",
            data_class=KernPCB2500Reading,
            queue_size=10
        )

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Kern PCB2500 pylabware driver started")

    def tare_balance(self):
        self.tared = False
        rospy.loginfo("Zeroing balance.")
        self.balance.tare_balance()
        rospy.sleep(2)
        self.tared = True

    def get_stable_mass(self):
        rospy.loginfo("Getting stable mass.")
        stable_mass = self.balance.get_stable_mass()
        rospy.sleep(2)
        rospy.loginfo("Stable mass is: %s", stable_mass)
        self.pub.publish(stable_mass)

    def get_mass(self):
        rospy.loginfo("Getting mass.")
        mass = self.balance.get_mass()
        rospy.sleep(2)
        rospy.loginfo("Stable mass is: %s", mass)
        self.pub.publish(mass)

    def callback_commands(self, msg):
        message = msg.kern_command
        rospy.loginfo("Message received.")
        if message != self._prev_msg: # TODO what if we do want to send the same msg twice? Use a time elapsed check (>15 secs)
            if message == msg.TARE_BALANCE:
                self.tare_balance()
            elif message == msg.GET_MASS:
                self.get_mass()
            elif message == msg.GET_MASS_STABLE:
                self.get_stable_mass()
            else:
                rospy.loginfo("Invalid command")
            self._prev_msg = message