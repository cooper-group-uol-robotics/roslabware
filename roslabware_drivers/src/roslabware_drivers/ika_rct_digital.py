# external
import rospy
from pylabware import RCTDigitalHotplate

# Core
from roslabware_msgs.msg import (
    IKARctDigitalCmd,
    IKARctDigitalReading,
)
from std_msgs.msg import Bool


class RCTDigitalHotplateRos:
    """
    Ros wrapper for the IKA RCT hotplate driver
    """

    def __init__(
        self,
        device_name: str,
        connection_mode: str,
        address: str,
        port: str,
        sensor: int,
        simulation: bool
    ):

        # Instantiate IKA driver
        self.hotplate = RCTDigitalHotplate(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port,
        )

        if simulation == "True":
            self.hotplate.simulation = True

        # connect
        self.hotplate.connect()
        self.hotplate.initialize_device()

        # Initialize ROS subscriber
        self.sub = rospy.Subscriber(
            name="ika_rct_digital_commands",
            data_class=IKARctDigitalCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher
        self.pub = rospy.Publisher(
            name="ika_rct_digital_readings",
            data_class=IKARctDigitalReading,
            queue_size=10,
        )

        # Sleeping rate
        self.rate = rospy.Rate(1)

        self._task_complete_pub = rospy.Publisher(
            '/tecan_xcalibur/task_complete',
            Bool,
            queue_size=1)

        rospy.loginfo("IKA RCT digital hotplate driver started")

        # Get data
        while not rospy.is_shutdown():

            # Get temperature of external sensor as default
            temperature = self.hotplate.get_temperature(sensor=int(sensor))
            stir_speed = self.hotplate.get_speed()
            viscosity_trend = self.hotplate.get_viscosity_trend()

            self.pub.publish(
                float(temperature), float(stir_speed), float(viscosity_trend)
            )

            rospy.loginfo(
                " Hotplate Temperature: "
                + str(temperature)
                + "| Stirring Speed: "
                + str(stir_speed)
                + "| Viscosity Trend: "
                + str(viscosity_trend)
            )

            self.rate.sleep()

    def start_heating(self):
        self.hotplate.start_temperature_regulation()
        rospy.loginfo("Turning on Heating")

    def stop_heating(self):
        self.hotplate.stop_temperature_regulation()
        rospy.loginfo("Turning off Heating")

    def start_stirring(self):
        self.hotplate.start_stirring()
        rospy.loginfo("Turning on Stirring")

    def stop_stirring(self):
        self.hotplate.stop_stirring()
        rospy.loginfo("Turning off Stirring")

    def set_speed(self, speed: int):
        self.hotplate.set_speed(speed)
        rospy.loginfo("Setting Stirring To: " + str(speed) + "RPM")

    def set_temperature(self, temperature: int):
        self.hotplate.set_temperature(temperature)
        rospy.loginfo("Setting Heating To: " + str(temperature) + "ºC")

    def callback_commands(self, msg):

        message = msg.ika_command

        if message == msg.HEAT_ON:
            self.start_heating()
        elif message == msg.HEAT_OFF:
            self.stop_heating()
        elif message == msg.STIR_ON:
            self.start_stirring()
        elif message == msg.STIR_OFF:
            self.stop_stirring()
        elif message == msg.SET_STIR:
            self.set_speed(msg.ika_param)
        elif message == msg.SET_HEAT:
            self.set_temperature(msg.ika_param)
        elif message == msg.STIR_AT:
            self.set_speed(msg.ika_param)
            self.start_stirring()
        elif message == msg.HEAT_AT:
            self.set_temperature(msg.ika_param)
            self.start_heating()
        elif message == msg.ALL_OFF:
            self.stop_heating()
            self.stop_stirring()
            self.set_speed(0)
            self.set_temperature(21)
        else:
            rospy.loginfo("invalid command")
        
        self._task_complete_pub.publish(bool(True))




