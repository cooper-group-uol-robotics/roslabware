# external
from typing import Optional

import rospy
from pylabware import XCalibur

# Core
from roslabware_msgs.msg import (
    TecanXCaliburCmd,
    TecanXCaliburReading)
from std_msgs.msg import Bool

# Constants
DEFAULT_SPEED = 40  # ml/min
DEFAULT_RESOLUTION = "N1"


class XCaliburRos:
    """
    ROS Wrapper for Driver for Tecan Xcalibur syringe pump.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        switch_address: str,
        syringe_size: float,
        simulation: bool,
    ):

        # Create device object
        self.tecan = XCalibur(
            device_name=device_name,
            connection_mode=connection_mode,
            switch_address=switch_address,
            address=address,
            port=port
        )

        if simulation == "True":
            self.tecan.simulation = True

        # Add syringe size attribute
        self._syringe_size = float(syringe_size)
        self.tecan.connect()
        self.tecan.set_resolution_mode(resolution_mode=DEFAULT_RESOLUTION)
        self.tecan.set_speed(DEFAULT_SPEED)
        self.tecan.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="Tecan_XCalibur_Commands",
            data_class=TecanXCaliburCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="Tecan_XCalibur_Readings",
            data_class=TecanXCaliburReading,
            queue_size=10
        )

        rospy.loginfo("XCalibur Driver Started")

        # Sleeping rate
        self.rate = rospy.Rate(1)

        self._task_complete_pub = rospy.Publisher(
            '/tecan_xcalibur/task_complete',
            Bool,
            queue_size=1)

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
        self.steps_per_ml = float(self.tecan.number_of_steps / self._syringe_size)
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

    def _convert_port(self, port: int):
        """Converts port for I-O notation for pumps"""

        if self.clockwise:
            position = "I" + str(port)
        else:
            position = "O" + str(port)

        return position

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

        # Add Input for ports
        _port = self._convert_port(port)
        # Convert to increments and increments/s
        increments = self._volume_to_step(volume)
        velocity = self._convert_velocity(speed)
        # Actions
        self.tecan.set_valve_position(_port)
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

        # Add inputs for ports
        _port = self._convert_port(port)
        # Convert to increments and increments/s
        increments = self._volume_to_step(volume)
        velocity = self._convert_velocity(speed)
        # Actions
        self.tecan.set_valve_position(_port)
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
        message = msg.tecan_xcalibur_command

        if message == msg.DISPENSE:
            self.dispense(
                msg.xcalibur_port,
                msg.xcalibur_volume,
                msg.xcalibur_speed)
        elif message == msg.WITHDRAW:
            self.withdraw(
                msg.xcalibur_port,
                msg.xcalibur_volume,
                msg.xcalibur_speed)
        else:
            rospy.loginfo("invalid command")

        self._task_complete_pub.publish(bool(True))
