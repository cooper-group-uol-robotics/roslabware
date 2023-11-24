# external
from typing import Optional
from pylabware import XLP6000
import rospy
import time


# Core
from roslabware_msgs.msg import TecanXlp6000Cmd, TecanXlp6000Reading
from std_msgs.msg import Bool

# Constants
DEFAULT_SPEED = 140  # ml/min
DEFAULT_RESOLUTION = "N2"


class XLP6000Ros:
    """ROS Wrapper for Driver for Tecan XLP6000 syringe pump."""

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
        self.tecan = XLP6000(
            device_name=device_name,
            connection_mode=connection_mode,
            switch_address=switch_address,
            address=address,
            port=port,
        )

        self._prev_msg = None

        if simulation == "True":
            self.tecan.simulation = True

        # Add syringe size attribute
        self._syringe_size = float(syringe_size)
        self.tecan.connect()
        self.tecan.set_resolution_mode(resolution_mode=DEFAULT_RESOLUTION)
        self.tecan.set_speed(150)
        self.tecan.initialize_device()
        self._prev_msg = None
        self.operation_complete = False

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="/tecan_xlp6000_command",
            data_class=TecanXlp6000Cmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="/tecan_xlp6000_reading", 
            data_class=TecanXlp6000Reading, 
            queue_size=10
        )

        rospy.loginfo("XLP6000 Driver Started")

        self._task_complete_pub = rospy.Publisher(
            "/tecan_xlp/task_complete", Bool, queue_size=1
        )

        # Sleeping rate
        self.rate = rospy.Rate(1)

        # Get data
        while not rospy.is_shutdown():

            #plunger, valve = self.get_positions()
            self._task_complete_pub.publish(self.operation_complete)
            # self.pub.publish(plunger, valve)
            # rospy.loginfo(
            #     " Plunger position: "
            #     + str(plunger)
            #     + "| Valve position: "
            #     + str(valve))
            rospy.sleep(5)

    def stop(self):
        """Stops executing any program/action immediately."""
        self.tecan.stop()

    def _volume_to_step(self, volume: float):
        """Converts volume in mL to number of increments based on the
        resolution set in the syringe pump.

        Args:
            volume(float): volume in mL
        Returns:
            increments (float): steps increments
        """
        self.steps_per_ml = float(self.tecan.number_of_steps / self._syringe_size)
        increments = int(round(volume * self.steps_per_ml))

        return increments

    def _convert_velocity(self, speed: float):
        """Converts ml/min to increments/s.

        Args:
            speed(float): speed in mL/min
        Returns:
            speed(float): speed in increments/s
        """
        speed = int(speed * self.steps_per_ml / 60)

        return speed

    def _convert_port(self, port: int):
        """Converts port for I-O notation for pumps."""

        if self.clockwise:
            position = "I" + str(port)
        else:
            position = "O" + str(port)

        return position

    def dispense(self, split_volume):
        """Dispense the specified volume with the defined speed
        at the specified port
            Args:
                port(int): port number
                volume (float): volume to dispense in mL
                speed (float): speed in mL/min"""
        # Add inputs for ports
        _port = "I" + str(self._dispense_port)

        # Convert to increments and increments/s
        increments = self._volume_to_step(split_volume)
        velocity = self._convert_velocity(self._speed)
        # Actions
        rospy.loginfo(f"dispense command received for volume:{split_volume} ml")
        self.tecan.set_valve_position(_port)
        self.tecan.set_speed(speed=velocity)
        self.tecan.dispense(increments)
        rospy.sleep(3)

      
    def withdraw(self, split_volume):
        """Withdraw the specified volume with the defined speed
        at the specified port
            Args:
                port(int): port number
                volume (float): volume to dispense in mL
                speed (float): speed in mL/min"""
        # Add inputs for ports
        _port = "I" + str(self._withdraw_port)

        # Convert to increments and increments/s
        increments = self._volume_to_step(split_volume)
        velocity = self._convert_velocity(DEFAULT_SPEED)
        # Actions
        rospy.loginfo(f"withdraw command received for volume:{split_volume} ml")
        self.tecan.set_valve_position(_port)
        self.tecan.set_speed(speed=velocity)
        self.tecan.withdraw(increments)
        rospy.sleep(3)


    def move_plunger_relative(self, position: int, set_busy: bool = True):
        """Makes relative plunger move.

        This is a wrapper for dispense()/withdraw().
        """
        position = int(position)
        if position > 0:
            return self.withdraw(position, set_busy)
        return self.dispense(abs(position), set_busy)

    def get_positions(self):
        """Gets plunger and valve positions."""
        plunger = self.tecan.get_plunger_position()
        valve = self.tecan.get_valve_position()

        return plunger, valve
    
    def request_pumping(self,
        withdraw_port: int,
        dispense_port: int,
        volume: float,
        speed: Optional[float] = DEFAULT_SPEED):

        split_volume = []
        self._withdraw_port = withdraw_port
        self._dispense_port = dispense_port
        self._speed = speed
        self._volume = volume
        iterations, last_iteration_volume = divmod(self._volume, int(self._syringe_size))
        for i in range(iterations):
            split_volume.append(self._syringe_size)
        if last_iteration_volume != 0:
            split_volume.append(last_iteration_volume)
        for iter in range(len(split_volume)):            
            self.withdraw(split_volume[iter])
            while not self.tecan.is_idle():
                time.sleep(3)
            self.dispense(split_volume[iter])
            while not self.tecan.is_idle():
                time.sleep(3)
        self.operation_complete = True


    def callback_commands(self, msg):
        """Callback commands for susbcriber."""
        message = msg.tecan_xlp_command
        time_now = time.time()
        if (time_now-self.time_before) > 10:
            self.operation_complete = False
            if message == msg.DISPENSE:
                self.request_pumping(
                    msg.xlp_withdraw_port,
                    msg.xlp_dispense_port,
                    msg.xlp_volume,
                    msg.xlp_speed)
                self.time_before = time_now
            else:
                rospy.loginfo("invalid command")
  
