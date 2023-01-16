# external
from typing import Optional

import rospy
from pylabware import XLP6000

# Core
from roslabware_msgs.msg import (
    tecan_xlp6000_command,
    tecan_xlp6000_reading,
)

# Constants
DEFAULT_SPEED = 40  # ml/min
DEFAULT_RESOLUTION = "N2"


class XLP6000Ros:
    """
    ROS Wrapper for Driver for Tecan XLP6000 syringe pump.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        switch_address: str,
        syringe_size: float,
        simulation: bool
    ):

        # Create device object
        self.tecan = XLP6000(
            device_name=device_name,
            connection_mode=connection_mode,
            switch_address=switch_address,
            address=address,
            port=port
        )

        if simulation == "True":
            self.tecan.simulation = True

        # Add syringe size attribute
        self.tecan._syringe_size = syringe_size
        self.tecan.connect()
        self.tecan.set_resolution_mode(resolution_mode=DEFAULT_RESOLUTION)
        self.tecan.set_speed(DEFAULT_SPEED)
        self.tecan.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="Tecan XLP6000 Commands",
            data_class=tecan_xlp6000_command,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="Tecan XLP6000 Readings",
            data_class=tecan_xlp6000_reading,
            queue_size=10
        )

        rospy.loginfo("XLP6000 Driver Started")

        # Sleeping rate
        self.rate = rospy.Rate(1)

        # Get data
        while not rospy.is_shutdown():
            plunger, valve = self.get_positions()
            self.pub.publish(plunger, valve)
            rospy.loginfo(
                " Plunger position: "
                + str(plunger)
                + "| Valve position: "
                + str(valve))

            self.rate.sleep()

    def stop(self):
        """Stops executing any program/action immediately"""
        self.tecan.stop()

    def _volume_to_step(self, volume: float):
        """Converts volume in mL to number of increments based
        on the resolution set in the syringe pump

        Args:
            volume(float): volume in mL
        Returns:
            increments (float): steps increments
        """
        self.steps_per_ml = int(self.tecan.number_of_steps / self.tecan._syringe_size)
        increments = int(round(volume * self.steps_per_ml))

        return increments

    def _convert_velocity(self, speed: float):
        """Converts ml/min to increments/s

        Args:
            speed(float): speed in mL/min
        Returns:
            speed(float): speed in increments/s
        """
        speed = int(speed * self.steps_per_ml / 60)

        return speed

    def dispense(
        self,
        port: int,
        volume: float,
        speed: Optional[float] = DEFAULT_SPEED
    ):
        """Dispense the specified volume with the defined speed
        at the specified port
            Args:
                port(int): port number
                volume (float): volume to dispense in mL
                spped (float): speed in mL/min"""
        # Convert to increments and increments/s
        increments = self._volume_to_step(volume)
        velocity = self._convert_velocity(speed)
        # Actions
        self.tecan.set_valve_position(port)
        self.tecan.set_speed(speed=velocity)
        self.tecan.dispense(increments, velocity)

    def withdraw(
        self,
        port: int,
        volume: float,
        speed: Optional[float] = DEFAULT_SPEED
    ):
        """Withdraw the specified volume with the defined speed
        at the specified port
            Args:
                port(int): port number
                volume (float): volume to dispense in mL
                spped (float): speed in mL/min"""
        # Convert to increments and increments/s
        increments = self._volume_to_step(volume)
        velocity = self._convert_velocity(speed)
        # Actions
        self.tecan.set_valve_position(port)
        self.tecan.set_speed(speed=velocity)
        self.tecan.withdraw(increments, velocity)

    def move_plunger_relative(self, position: int, set_busy: bool = True):
        """Makes relative plunger move. This is a wrapper for
        dispense()/withdraw().
        """
        position = int(position)
        if position > 0:
            return self.withdraw(position, set_busy)
        return self.dispense(abs(position), set_busy)

    def get_positions(self):
        """Gets plunger and valve positions"""
        plunger = self.tecan.get_plunger_position()
        valve = self.tecan.get_valve_position()

        return plunger, valve

    def callback_commands(self, msg):
        """Callback commands for susbcriber"""
        message = msg.tecan_command

        if message == msg.DISPENSE:
            self.dispense(msg.tecan_param)
        elif message == msg.WITHDRAW:
            self.withdraw(msg.tecan_param)
        else:
            rospy.loginfo("invalid command")
