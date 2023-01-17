# external
from typing import Optional, Union

import rospy
from pylabware import XPR226DRQ

# Core
from roslabware_msgs.msg import (
    MettlerXPR226Cmd,
    MettlerXPR226Reading,
)


class XPR226DRQRos:
    """
    ROS Wrapper for Serial Driver for Mettler Toledo XPR226
    DRQ dispensing system.
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False
    ):

        # Create device object
        self.balance = XPR226DRQ()

        # TODO after SOAP implementation
        # if simulation == "True":
        #     self.balance.simulation = True

        # self.balance.connect()
        # self.balance.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_xpr226_drq",
            data_class=MettlerXPR226Cmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="mettler_xpr226_drq_info",
            data_class=MettlerXPR226Reading,
            queue_size=10,
        )

        rospy.loginfo("Mettler XPR226 DRQ Driver Started")

    def start_dosing(self):
        self.balance.start_dosing()
        rospy.loginfo("Starting Dosing")

    def open_door(self):
        self.balance.open_door()
        rospy.loginfo("Opening Front Door")

    def close_door(self):
        self.balance.close_door()
        rospy.loginfo("Closing Front Door")

    def tare(self):
        self.balance.tare()
        rospy.loginfo("Taring balance")

    def get_mass(self):
        self.balance.get_mass()
        rospy.loginfo("Get balance")

    # Callback for subscriber.
    def callback_commands(self, msg):
        """Callback commands for susbcriber"""
        message = msg.xpr_command

        if message == msg.START_DOSE:
            self.start_dosing()
        elif message == msg.OPEN_DOOR:
            self.open_door()
        elif message == msg.CLOSE_DOOR:
            self.close_door()
        elif message == msg.TARE:
            self.tare()
        elif message == msg.GET_MASS:
            self.get_mass()
        else:
            rospy.loginfo("invalid command")

        self.pub_done.publish("Done")
