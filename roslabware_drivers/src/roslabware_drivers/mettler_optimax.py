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
    """ROS Wrapper for Serial Driver for Mettler Toledo XPR226 DRQ
    dispensing system."""

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

    def add_temp_step(self, temperature):
        self.optimax._add_heating_step(temperature)
        rospy.loginfo(f"Added temperature step with temperature {temperature} ºC")

    def add_stir_step(self, speed):
        self.optimax._add_stirring_step(speed)
        rospy.loginfo(f"Added stirring step with speed {speed} RPM")

    def add_wait_step(self, time):
        self.optimax._add_waiting_step(time)
        rospy.loginfo(f"Added waiting step with duration {time} Minutes")

    def start_experiment(self):
        self.optimax.start()
        rospy.loginfo("Experiment Started")

    def stop_experiment(self):
        self.optimax.stop()
        rospy.loginfo("Experiment Stopped")

    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.optimax_command
        param = msg.optimax_param

        if message == msg.ADD_TEMP:
            self.add_temp_step(param)
        elif message == msg.ADD_STIR:
            self.add_stir_step(param)
        elif message == msg.ADD_WAIT:
            self.add_wait_step(param)
        elif message == msg.START:
            self.start_experiment()
        elif message == msg.STOP:
            self.stop_experiment()
        else:
            rospy.loginfo("invalid command")


rospy.loginfo("working")
