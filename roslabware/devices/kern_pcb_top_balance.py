# external
from typing import Optional, Union

import rospy
from pylabware import TopBalance

# Core
from ..msgs.kern_pcb_top_balance import kern_command, kern_reading


class TopBalanceRos:
    """
    ROS wrapper class for controlling Kern PCB Top Balance
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
    ):

        self.pub = None
        self.tared = False

        # Instantiate IKA driver
        self.balance = TopBalance(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port,
        )

        # Initialize ros subscriber of topic to which commands are published
        self.subs = rospy.Subscriber(
            name="Kern_Commands",
            data_class=kern_command,
            callback=self.callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher(
            name="Kern_Weights", data_class=kern_reading, queue_size=10
        )

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Kern driver started")

    def tare_balance(self):
        self.tared = True
        rospy.sleep(2)

        self.kern.tare_balance()
        rospy.loginfo("Zeroing Balance")

        rospy.sleep(2)
        self.tared = False

    def get_stable_mass(self):
        self.pub.publish(float(self.balance.get_stable_mass()))

    def get_mass(self):
        self.pub.publish(float(self.balance.get_mass()))

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
