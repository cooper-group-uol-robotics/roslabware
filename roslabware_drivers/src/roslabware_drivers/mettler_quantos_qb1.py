# external
from typing import Optional, Union
from xml.dom import minidom  # noqa: DUO107

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
    """
    ROS Wrapper for Serial Driver for Mettler Toledo Quantos
    QB1 dispensing system.
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
    ):
        self._doorPos = 0
        self._samplerPos = 0

        # Create device object
        self.quantos = QuantosQB1(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port
        )

        self.quantos.connect()
        self.quantos.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_quantos_qB1_commands",
            data_class=MettlerQuantosQB1Cmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub_done = rospy.Publisher(
            name="mettler_quantos_qB1_status",
            data_class=String,
            queue_size=1
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

    def get_head_data(self):
        # TODO check if this is the correct strcuture SMZ
        self.quantos.connection.connection_parameters["timeout"] = 30
        head_data = self.quantos.get_head_data()
        print(head_data)
        try:
            xmldoc = minidom.parseString(head_data)
            substance = (
                xmldoc.getElementsByTagName("Substance")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Substance")) > 0
                    and len(xmldoc.getElementsByTagName("Substance")[0].childNodes) > 0
                )
                else ""
            )
            lot_id = (
                xmldoc.getElementsByTagName("Lot_ID")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Lot_ID")) > 0
                    and len(xmldoc.getElementsByTagName("Lot_ID")[0].childNodes) > 0
                )
                else ""
            )
            user_id = (
                xmldoc.getElementsByTagName("User_ID")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("User_ID")) > 0
                    and len(xmldoc.getElementsByTagName("User_ID")[0].childNodes) > 0
                )
                else ""
            )
            filling_date = (
                xmldoc.getElementsByTagName("Dispense_date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dispense_date")) > 0
                    and len(xmldoc.getElementsByTagName("Dispense_date")[0].childNodes)
                    > 0
                )
                else ""
            )
            expiry_date = (
                xmldoc.getElementsByTagName("Exp._date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Exp._date")) > 0
                    and len(xmldoc.getElementsByTagName("Exp._date")[0].childNodes) > 0
                )
                else ""
            )
            retest_date = (
                xmldoc.getElementsByTagName("Retest_date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Retest_date")) > 0
                    and len(xmldoc.getElementsByTagName("Retest_date")[0].childNodes)
                    > 0
                )
                else ""
            )
            mass_in_mg = (
                xmldoc.getElementsByTagName("Content")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Content")) > 0
                    and len(xmldoc.getElementsByTagName("Content")[0].childNodes) > 0
                )
                else ""
            )
            remaining_doses = (
                xmldoc.getElementsByTagName("Rem._dosages")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Rem._dosages")) > 0
                    and len(xmldoc.getElementsByTagName("Rem._dosages")[0].childNodes)
                    > 0
                )
                else ""
            )
            terminal_snr = (
                xmldoc.getElementsByTagName("Terminal_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Terminal_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Terminal_SNR")[0].childNodes)
                    > 0
                )
                else ""
            )
            bridge_snr = (
                xmldoc.getElementsByTagName("Bridge_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Bridge_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Bridge_SNR")[0].childNodes) > 0
                )
                else ""
            )
            balance_type = (
                xmldoc.getElementsByTagName("Balance_Type")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Balance_Type")) > 0
                    and len(xmldoc.getElementsByTagName("Balance_Type")[0].childNodes)
                    > 0
                )
                else ""
            )
            balance_id = (
                xmldoc.getElementsByTagName("Balance_ID")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Balance_ID")) > 0
                    and len(xmldoc.getElementsByTagName("Balance_ID")[0].childNodes) > 0
                )
                else ""
            )
            last_calibration = (
                xmldoc.getElementsByTagName("Last_cal.")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Last_cal.")) > 0
                    and len(xmldoc.getElementsByTagName("Last_cal.")[0].childNodes) > 0
                )
                else ""
            )
            option_snr = (
                xmldoc.getElementsByTagName("Option_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Option_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Option_SNR")[0].childNodes) > 0
                )
                else ""
            )
            dose_unit_snr = (
                xmldoc.getElementsByTagName("Dose_unit_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dose_unit_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Dose_unit_SNR")[0].childNodes)
                    > 0
                )
                else ""
            )
            application_name = (
                xmldoc.getElementsByTagName("Appl._Name")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Appl._Name")) > 0
                    and len(xmldoc.getElementsByTagName("Appl._Name")[0].childNodes) > 0
                )
                else ""
            )
            date_time = (
                xmldoc.getElementsByTagName("Date_Time")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Date_Time")) > 0
                    and len(xmldoc.getElementsByTagName("Date_Time")[0].childNodes) > 0
                )
                else ""
            )
            level_control = (
                xmldoc.getElementsByTagName("Levelcontrol")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Levelcontrol")) > 0
                    and len(xmldoc.getElementsByTagName("Levelcontrol")[0].childNodes)
                    > 0
                )
                else ""
            )
            production_head_date = (
                xmldoc.getElementsByTagName("Head_prod._date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Head_prod._date")) > 0
                    and len(
                        xmldoc.getElementsByTagName("Head_prod._date")[0].childNodes
                    )
                    > 0
                )
                else ""
            )
            head_type = (
                xmldoc.getElementsByTagName("Head_type")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Head_type")) > 0
                    and len(xmldoc.getElementsByTagName("Head_type")[0].childNodes) > 0
                )
                else ""
            )
            dose_limit = (
                xmldoc.getElementsByTagName("Dose_limit")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dose_limit")) > 0
                    and len(xmldoc.getElementsByTagName("Dose_limit")[0].childNodes) > 0
                )
                else ""
            )
            dosing_counter = (
                xmldoc.getElementsByTagName("Dosing_counter")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dosing_counter")) > 0
                    and len(xmldoc.getElementsByTagName("Dosing_counter")[0].childNodes)
                    > 0
                )
                else ""
            )
            remaining_quantity = (
                xmldoc.getElementsByTagName("Rem._quantity")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Rem._quantity")) > 0
                    and len(xmldoc.getElementsByTagName("Rem._quantity")[0].childNodes)
                    > 0
                )
                else ""
            )

            # Publish all info
            self.pubSample.publish(
                substance,
                lot_id,
                user_id,
                filling_date,
                expiry_date,
                retest_date,
                mass_in_mg,
                remaining_doses,
                terminal_snr,
                bridge_snr,
                balance_type,
                balance_id,
                last_calibration,
                option_snr,
                dose_unit_snr,
                application_name,
                date_time,
                level_control,
                production_head_date,
                head_type,
                dose_limit,
                dosing_counter,
                remaining_quantity,
            )

            rospy.loginfo("Published Head Data")
        except Exception:
            rospy.loginfo("Parsing XML Data Failed")

        # TODO Check this structure
        self.quantos.connection.connection_parameters["timeout"] = 120

    def get_sample_data(self):

        self.quantos.connection.connection_parameters["timeout"] = 30
        sample_data = self.quantos.get_sample_data()
        print(sample_data)

        # Parse the data
        try:
            xmldoc = minidom.parseString(sample_data)
            substance = (
                xmldoc.getElementsByTagName("Substance")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Substance")) > 0
                    and len(xmldoc.getElementsByTagName("Substance")[0].childNodes) > 0
                )
                else ""
            )
            lot_id = (
                xmldoc.getElementsByTagName("Lot_ID")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Lot_ID")) > 0
                    and len(xmldoc.getElementsByTagName("Lot_ID")[0].childNodes) > 0
                )
                else ""
            )
            user_id = (
                xmldoc.getElementsByTagName("User_ID")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("User_ID")) > 0
                    and len(xmldoc.getElementsByTagName("User_ID")[0].childNodes) > 0
                )
                else ""
            )
            filling_date = (
                xmldoc.getElementsByTagName("Dispense_date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dispense_date")) > 0
                    and len(xmldoc.getElementsByTagName("Dispense_date")[0].childNodes)
                    > 0
                )
                else ""
            )
            expiry_date = (
                xmldoc.getElementsByTagName("Exp._date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Exp._date")) > 0
                    and len(xmldoc.getElementsByTagName("Exp._date")[0].childNodes) > 0
                )
                else ""
            )
            retest_date = (
                xmldoc.getElementsByTagName("Retest_date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Retest_date")) > 0
                    and len(xmldoc.getElementsByTagName("Retest_date")[0].childNodes)
                    > 0
                )
                else ""
            )
            mass_in_mg = (
                xmldoc.getElementsByTagName("Content")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Content")) > 0
                    and len(xmldoc.getElementsByTagName("Content")[0].childNodes) > 0
                )
                else ""
            )
            remaining_doses = (
                xmldoc.getElementsByTagName("Rem._dosages")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Rem._dosages")) > 0
                    and len(xmldoc.getElementsByTagName("Rem._dosages")[0].childNodes)
                    > 0
                )
                else ""
            )
            terminal_snr = (
                xmldoc.getElementsByTagName("Terminal_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Terminal_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Terminal_SNR")[0].childNodes)
                    > 0
                )
                else ""
            )
            bridge_snr = (
                xmldoc.getElementsByTagName("Bridge_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Bridge_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Bridge_SNR")[0].childNodes) > 0
                )
                else ""
            )
            balance_type = (
                xmldoc.getElementsByTagName("Balance_Type")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Balance_Type")) > 0
                    and len(xmldoc.getElementsByTagName("Balance_Type")[0].childNodes)
                    > 0
                )
                else ""
            )
            balance_id = (
                xmldoc.getElementsByTagName("Balance_ID")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Balance_ID")) > 0
                    and len(xmldoc.getElementsByTagName("Balance_ID")[0].childNodes) > 0
                )
                else ""
            )
            last_calibration = (
                xmldoc.getElementsByTagName("Last_cal.")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Last_cal.")) > 0
                    and len(xmldoc.getElementsByTagName("Last_cal.")[0].childNodes) > 0
                )
                else ""
            )
            option_snr = (
                xmldoc.getElementsByTagName("Option_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Option_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Option_SNR")[0].childNodes) > 0
                )
                else ""
            )
            dose_unit_snr = (
                xmldoc.getElementsByTagName("Dose_unit_SNR")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dose_unit_SNR")) > 0
                    and len(xmldoc.getElementsByTagName("Dose_unit_SNR")[0].childNodes)
                    > 0
                )
                else ""
            )
            application_name = (
                xmldoc.getElementsByTagName("Appl._Name")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Appl._Name")) > 0
                    and len(xmldoc.getElementsByTagName("Appl._Name")[0].childNodes) > 0
                )
                else ""
            )
            date_time = (
                xmldoc.getElementsByTagName("Date_Time")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Date_Time")) > 0
                    and len(xmldoc.getElementsByTagName("Date_Time")[0].childNodes) > 0
                )
                else ""
            )
            level_control = (
                xmldoc.getElementsByTagName("Levelcontrol")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Levelcontrol")) > 0
                    and len(xmldoc.getElementsByTagName("Levelcontrol")[0].childNodes)
                    > 0
                )
                else ""
            )
            production_head_date = (
                xmldoc.getElementsByTagName("Head_prod._date")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Head_prod._date")) > 0
                    and len(
                        xmldoc.getElementsByTagName("Head_prod._date")[0].childNodes
                    )
                    > 0
                )
                else ""
            )
            head_type = (
                xmldoc.getElementsByTagName("Head_type")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Head_type")) > 0
                    and len(xmldoc.getElementsByTagName("Head_type")[0].childNodes) > 0
                )
                else ""
            )
            dose_limit = (
                xmldoc.getElementsByTagName("Dose_limit")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dose_limit")) > 0
                    and len(xmldoc.getElementsByTagName("Dose_limit")[0].childNodes) > 0
                )
                else ""
            )
            dosing_counter = (
                xmldoc.getElementsByTagName("Dosing_counter")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Dosing_counter")) > 0
                    and len(xmldoc.getElementsByTagName("Dosing_counter")[0].childNodes)
                    > 0
                )
                else ""
            )
            remaining_quantity = (
                xmldoc.getElementsByTagName("Rem._quantity")[0].childNodes[0].data
                if (
                    len(xmldoc.getElementsByTagName("Rem._quantity")) > 0
                    and len(xmldoc.getElementsByTagName("Rem._quantity")[0].childNodes)
                    > 0
                )
                else ""
            )

            # Publish all info
            self.pubSample.publish(
                substance,
                lot_id,
                user_id,
                filling_date,
                expiry_date,
                retest_date,
                mass_in_mg,
                remaining_doses,
                terminal_snr,
                bridge_snr,
                balance_type,
                balance_id,
                last_calibration,
                option_snr,
                dose_unit_snr,
                application_name,
                date_time,
                level_control,
                production_head_date,
                head_type,
                dose_limit,
                dosing_counter,
                remaining_quantity,
            )

            rospy.loginfo("Published Sample Data")
        except Exception:
            rospy.loginfo("Parsing XML Data Failed")

        self.quantos.connection.connection_parameters["timeout"] = 120

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
