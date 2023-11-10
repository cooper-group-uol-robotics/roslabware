# external
from typing import Optional, Union

import rospy
from pylabware import QuantosQB1
from std_msgs.msg import String

# Core
from roslabware_msgs.msg import (
    MettlerQuantosQB1Cmd,
    MettlerQuantosQB1Reading,
    MettlerQuantosQB1Sample,
)


class QuantosQB1Ros:
    """ROS Wrapper for Serial Driver for Mettler Toledo Quantos QB1
    dispensing system."""

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False,
    ):
        self._doorPos = 0
        self._samplerPos = 0

        # Create device object
        self.quantos = QuantosQB1(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port,
        )

        if simulation == "True":
            self.quantos.simulation = True

        self.quantos.connect()
        rospy.loginfo("connected")
        self.quantos.initialize_device()
        rospy.loginfo("initialized")

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_quantos_qB1_commands",
            data_class=MettlerQuantosQB1Cmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub_done = rospy.Publisher(
            name="mettler_quantos_qB1_status", data_class=String, queue_size=1
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="mettler_quantos_qB1_info",
            data_class=MettlerQuantosQB1Reading,
            queue_size=10,
        )

        # Initialize ROS publisher for samples info
        self.pubSample = rospy.Publisher(
            name="mettler_quantos_qB1_sample",
            data_class=MettlerQuantosQB1Sample,
            queue_size=10,
        )

        rospy.loginfo("Mettler Quantos QB1 Driver Started")
        self.quantos.open_door()

    def tare(self):
        reply = self.quantos.tare()
        rospy.loginfo(reply)
        rospy.loginfo("Tared the balance")

    def get_stable_weight(self):
        reply = self.quantos.get_stable_weight()
        rospy.loginfo(reply)
        rospy.loginfo("Got stable weight")

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

    def get_head_data(self):
        # TODO check if this is the correct strcuture SMZ
        # self.quantos.connection.connection_parameters["timeout"] = 30
        head_data = self.quantos.get_head_data()

        try:
            # Publish all info
            rospy.loginfo(head_data)
            rospy.loginfo("Published Head Data")
        except Exception:
            rospy.loginfo("Parsing XML Data Failed")

        # TODO Check this structure
        self.quantos.connection.connection_parameters["timeout"] = 120

    def lock_dosing_head_pin(self):
        reply = self.quantos.lock_dosing_head_pin()
        rospy.loginfo(reply)

    def unlock_dosing_head_pin(self):
        reply = self.quantos.unlock_dosing_head_pin()
        rospy.loginfo(reply)

    def open_front_door(self):
        rospy.loginfo("here-------")
        reply = self.quantos.open_door()
        rospy.loginfo(reply)
        rospy.loginfo("Opening Front Door")

    def close_front_door(self):
        reply = self.quantos.close_door()
        rospy.loginfo(reply)
        rospy.loginfo("Closing Front Door")

    def set_target_mass(self, mass: float):
        reply = self.quantos.set_target_mass(mass)
        rospy.loginfo(reply)
        rospy.loginfo("Setting Target Mass Value")

    def set_tolerance(self, percentage: float):
        reply = self.quantos.set_tolerance(percentage)
        rospy.loginfo(reply)
        rospy.loginfo("Setting Percentage Tolerance")

    def set_tolerance_mode(self, overdose: bool):
        reply = self.quantos.set_tolerance_mode(overdose)
        rospy.loginfo(reply)
        if overdose:
            rospy.loginfo("Setting Tolerance Overdose Mode")
        else:
            rospy.loginfo("Setting Tolerance Normal Mode")

    def set_value_pan(self):
        reply = self.quantos.set_value_pan()
        rospy.loginfo(reply)
        rospy.loginfo("Setting weighing pan as empty")

    def set_antistatic_on(self):
        reply = self.quantos.set_antistatic_on()
        rospy.loginfo(reply)
        rospy.loginfo("Setting antistatic system on")

    def set_antistatinc_off(self):
        reply = self.quantos.set_antistatic_off()
        rospy.loginfo(reply)
        rospy.loginfo("Setting antistatic system off")

    def dispense_solid(self, percentage: float, amount: float):
        self.set_target_mass(amount)

        self.set_tolerance(percentage)

        self.start_dosing()

        self.pub_done.publish("Done")

    # Callback for subscriber.
    def callback_commands(self, msg):
        """Callback commands for susbcriber."""
        message = msg.quantos_command

        if message == msg.DISPENSE_SOLID:
            self.dispense_solid(msg.quantos_float, msg.quantos_float)
        elif message == msg.START_DOSE:
            self.start_dosing()
        elif message == msg.STOP_DOSE:
            self.stop_dosing()
        elif message == msg.GET_DOOR_POS:
            self.get_door_position()
        elif message == msg.GET_HEAD_DATA:
            self.get_head_data()
        elif message == msg.LOCK_PIN:
            self.lock_dosing_head_pin()
        elif message == msg.UNLOCK_PIN:
            self.unlock_dosing_head_pin()
        elif message == msg.OPEN_DOOR:
            self.open_front_door()
        elif message == msg.CLOSE_DOOR:
            self.close_front_door()
        elif message == msg.SET_TARGET:
            self.set_target_mass(msg.quantos_float)
        elif message == msg.SET_TOL:
            self.set_tolerance(msg.quantos_float)
        elif message == msg.SET_TOLMODE:
            self.set_tolerance_mode(msg.quantos_bool)
        elif message == msg.SET_PAN_OFF:
            self.set_value_pan()
        elif message == msg.SET_AS_ON:
            self.set_antistatic_on()
        elif message == msg.SET_AS_OFF:
            self.set_antistatinc_off()
        elif message == msg.TARE:
            self.tare()
        elif message == msg.GET_STABLE_WEIGHT:
            self.get_stable_weight()
        else:
            rospy.loginfo("invalid command")

        self.pub_done.publish("Done")
