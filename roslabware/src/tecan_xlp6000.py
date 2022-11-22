# external
from typing import Optional, Union

import rospy
from pylabware import XLP6000

# Core
from ..msg.tecan_xlp6000 import (
    tecan_xlp6000_command,
    tecan_xlp6000_reading,
)


class XLP6000Ros:
    """
    ROS Wrapper for Serial Driver for Mettler Toledo Quantos
    QB1 dispensing system.
    """

    def __init__(
        self,
        device_name: str = None,
        syringe_size: float = None,
        resolution: str = "N2",
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: Optional[bool] = False,
    ):

        # Create device object
        self.tecan = XLP6000(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port
        )

        # Add syringe size attribute
        self.tecan._syringe_size = syringe_size

        if simulation:
            self.tecan.simulation = True

        else:
            self.tecan.connect()
            self.tecan.set_resolution_mode(resolution_mode=resolution)
            self.tecan.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="Tecan XLP6000 Commands",
            data_class=tecan_xlp6000_command,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub_done = rospy.Publisher(
            name="Tecan XLP6000 Readings",
            data_class=tecan_xlp6000_reading,
            queue_size=10
        )

        rospy.loginfo("XLP6000 Driver Started")

    def volume_to_step(self, volume_in_ml):
        """Converts volume specified to number of steps 

        Returns:
            float -- steps
        """
        self.steps_per_ml = int(self.tecan.number_of_steps / self.tecan._syringe_size)

        return int(round(volume_in_ml * self.steps_per_ml))

    def convert_velocity(self, speed):
        """Converts ml/min to increments/s

        Returns:
            velocity in increments/s float
        """
        velocity = int(speed * self.steps_per_ml / 60)

        return velocity

    def start_dosing(self):
        self.quantos.start_dosing()
        rospy.loginfo("Starting Dosing")

    def stop_dosing(self):
        self.quantos.stop_dosing
        rospy.loginfo("Stopping Dosing")

    def get_door_position(self):
        position = self.quantos.get_door_position()
        self.pub.publish(position, self._doorPos)
        rospy.loginfo("Getting Door Position")

    def get_sample_position(self):
        position = self.quantos.get_sample_position()
        self.pub.publish(position, self._samplerPos)
        rospy.loginfo("Getting Sampler Position")

    def move_dosing_pin(self, locked):
        self.quantos.move_dosing_head_pin(locked=locked)
        rospy.loginfo("Moving Dosing Head Pin")

    def open_door(self):
        self.quantos.open_door()
        rospy.loginfo("Opening Front Door")

    def close_door(self):
        self.quantos.close_door()
        rospy.loginfo("Closing Front Door")

    def move_sampler(self, position):
        self.quantos.move_sampler(position)
        rospy.loginfo("Moving Sampler")

    def set_tapping_before(self, activated: bool):
        self.quantos.set_tapping_before_dosing(activated)
        rospy.loginfo("Setting Status of Tapping Before Dosing Setting")

    def set_tapping_while_dosing(self, activated: bool):
        self.quantos.set_tapping_while_dosing(activated)
        rospy.loginfo("Setting Status of Tapping While Dosing Setting")

    def set_tapper_intensity(self, intensity: int):
        self.quantos.set_tapper_intensity(intensity)
        rospy.loginfo("Setting Intensity of Tapper")

    def set_tapper_duration(self, duration: int):
        self.quantos.set_tapper_duration(duration)
        rospy.loginfo("Setting Duration of Tapper")

    def set_target_mass(self, mass: float):
        self.quantos.set_target_mass(mass)
        rospy.loginfo("Setting Target Mass Value")

    def set_tolerance(self, percentage: float):
        self.quantos.set_tolerance()
        rospy.loginfo("Setting Percentage Tolerance")

    def set_tolerance_mode(self, overdose: bool):
        self.quantos.set_tolerance_mode(overdose)
        if overdose:
            rospy.loginfo("Setting Tolerance Overdose Mode")
        else:
            rospy.loginfo("Setting Tolerance Normal Mode")

    def set_sample_id(self, sample_id: str):
        self.quantos.set_sample_id(sample_id)
        rospy.loginfo("Setting Sample ID")

    def set_value_pan(self):
        self.quantos.set_value_pan()
        rospy.loginfo("Setting weighing pan as empty")

    def set_algorithm(self, advanced: bool):
        self.quantos.set_algorithm(advanced)
        if advanced:
            rospy.loginfo("Setting dispensing advanced algorithm")
        else:
            rospy.loginfo("Setting dispensing normal algorithm?")

    def set_antistatic(self, activated: bool):
        self.quantos.set_antistatic(activated)
        if activated:
            rospy.loginfo("Setting antistatic system activation")
        else:
            rospy.loginfo("No antistatic system")

    def dispense_solid(self, position: int, amount: float):

        # Open door
        self.open_door()
        # Move dosing pin
        self.move_dosing_pin(True)
        # Set target mass
        self.set_target_mass(amount)
        # Move sampler
        self.move_sampler(position)
        # Dose
        self.start_dosing()

        self.pubDone.publish("Done")

    # Callback for subscriber.
    def callback_commands(self, msg):
        """Callback commands for susbcriber"""
        message = msg.quantos_command

        if message == msg.DISPENSE_SOLID:
            self.dispense_solid(msg.quantos_int, msg.quantos_float)
        elif message == msg.START_DOSE:
            self.start_dosing()
        elif message == msg.STOP_DOSE:
            self.stop_dosing()
        elif message == msg.GET_DOOR_POS:
            self.get_door_position()
        elif message == msg.GET_SAMPLE_POS:
            self.get_sample_position()
        elif message == msg.GET_HEAD_DATA:
            self.get_head_data()
        elif message == msg.GET_SAMPLE_DATA:
            self.get_sample_data()
        elif message == msg.MOVE_PIN:
            self.move_dosing_pin(msg.quantos_bool)
        elif message == msg.OPEN_DOOR:
            self.open_door()
        elif message == msg.CLOSE_DOOR:
            self.close_door()
        elif message == msg.SET_SAMPLE_POS:
            self.move_sampler(msg.quantos_int)
        elif message == msg.SET_TAP_BEFORE:
            self.set_tapping_before(msg.quantos_bool)
        elif message == msg.SET_TAP_DURING:
            self.set_tapping_while_dosing(msg.quantos_bool)
        elif message == msg.SET_TAP_INT:
            self.set_tapper_intensity(msg.quantos_int)
        elif message == msg.SET_TAP_DURATION:
            self.set_tapper_duration(msg.quantos_int)
        elif message == msg.SET_TARGET:
            self.set_target_mass(msg.quantos_float)
        elif message == msg.SET_TOL:
            self.set_tolerance(msg.quantos_float)
        elif message == msg.SET_TOLMODE:
            self.set_tolerance_mode(msg.quantos_bool)
        elif message == msg.SET_SAMPLE_ID:
            self.set_sample_id(msg.quantos_ID)
        elif message == msg.SET_PAN_OFF:
            self.set_value_pan()
        elif message == msg.SET_ALG:
            self.set_algorithm(msg.quantos_bool)
        elif message == msg.SET_AS:
            self.set_antistatic(msg.quantos_bool)
        else:
            rospy.loginfo("invalid command")

        self.pub_done.publish("Done")
