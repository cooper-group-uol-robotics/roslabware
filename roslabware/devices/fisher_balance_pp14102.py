# external
from typing import Optional, Union

import rospy
from pylabware import BalancePPS4102

# core
from ..msgs.fisher_balance_pp14102 import (
    fisher_balance_pp14102_command,
    fisher_balance_pp14102_reading,
)


class BalancePPS4102Ros:
    """ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial Driver
    Utilises ROS Topics to facilitate communication with balance using a
    serial driver."""

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
    ):

        # Instantiate balance driver
        self.IKA = BalancePPS4102(
            device_name=device_name,
            connectionmode=connection_mode,
            address=address,
            port=port,
        )

        # Initialize ros subscriber of topic to which commands are published
        self.sub = rospy.Subscriber(
            name="fisher_balance_pp14102_commands",
            data_class=fisher_balance_pp14102_command,
            callback=self.callback_commands,
        )

        # Initialize ros published for Balance responses (weights)
        self.pub = rospy.Publisher(
            name="fisher_balance_pp14102_readings",
            data_class=fisher_balance_pp14102_reading,
            queue_size=10,
        )

    def get_stable_mass(self):
        self.pub.publish(float(self.balance.get_stable_mass()))

    def get_mass(self):
        self.pub.publish(float(self.balance.get_mass()))

    def tare_balance(self):
        self.balance.tare_balance()
        rospy.loginfo("Zeroing Balance")

    def turn_off(self):
        self.balance.turn_off()
        rospy.loginfo("Balance going into standby")

    def turn_on(self):
        self.balance.turn_on()
        rospy.loginfo("Balance turning on")

    def callback_commands(self, msg):

        message = msg.balance_command

        if message == msg.ZERO:
            self.tare_balance()
        elif message == msg.BALANCE_ON:
            self.turn_on()
        elif message == msg.BALANCE_OFF:
            self.turn_off()
        elif message == msg.MASS_STABLE:
            self.get_stable_mass()
        elif message == msg.MASS:
            self.get_mass()
        else:
            rospy.loginfo("invalid command")
