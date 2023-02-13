# external
from typing import Optional, Union

import rospy
from pylabware import Optimax

# Core
from roslabware_msgs.msg import (
    MettlerOptimaxCmd,
    MettlerOptimaxReading,
)


class OptimaxRos:
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
        simulation: bool = False,
        experiment_name: str = "test"
    ):

        # Create device object
        self.optimax = Optimax(experiment_name=experiment_name)

        # TODO after (IF) API implementation
        # if simulation == "True":
        #     self.optimax.simulation = True
        # self.optimax.connect()

        self.optimax.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_optimax",
            data_class=MettlerOptimaxCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="mettler_optimax_info",
            data_class=MettlerOptimaxReading,
            queue_size=10,
        )

        rospy.loginfo("Mettler Optimax Driver Started")

    def start_heating(self):
        self.optimax.start_temperature_regulation()
        rospy.loginfo("Turning on Heating")

    def stop_heating(self):
        self.optimax.stop_temperature_regulation()
        rospy.loginfo("Turning off Heating")

    def start_stirring(self):
        self.optimax.start_stirring()
        rospy.loginfo("Turning on Stirring")

    def stop_stirring(self):
        self.optimax.stop_stirring()
        rospy.loginfo("Turning off Stirring")

    def set_speed(self, speed: int):
        self.optimax.set_speed(speed)
        rospy.loginfo("Setting Stirring To: " + str(speed) + "RPM")

    def set_temperature(self, temperature: int):
        self.optimax.set_temperature(temperature)
        rospy.loginfo("Setting Heating To: " + str(temperature) + "ºC")

    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.optimax_command

        if message == msg.HEAT_ON:
            self.start_heating()
        elif message == msg.HEAT_OFF:
            self.stop_heating()
        elif message == msg.STIR_ON:
            self.start_stirring()
        elif message == msg.STIR_OFF:
            self.stop_stirring()
        elif message == msg.SET_STIR:
            self.set_speed(msg.optimax_param)
        elif message == msg.SET_HEAT:
            self.set_temperature(msg.optimax_param)
        elif message == msg.STIR_AT:
            self.set_speed(msg.optimax_param)
            self.start_stirring()
        elif message == msg.HEAT_AT:
            self.set_temperature(msg.optimax_param)
            self.start_heating()
        elif message == msg.ALL_OFF:
            self.stop_heating()
            self.stop_stirring()
            self.set_speed(0)
            self.set_temperature(21)
        else:
            rospy.loginfo("invalid command")


rospy.loginfo("working")
