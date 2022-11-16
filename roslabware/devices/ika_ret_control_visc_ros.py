# external
import rospy
from pylabware import RETControlViscHotplate

# local
from ..msgs.ika_ret_control_visc import (
    IKA_RET_CONTROL_VISC_Command,
    IKA_RET_CONTROL_VISC_Reading,
)


class RETControlViscHotplateRos:
    """
    Ros wrapper for the IKA RCT hotplate driver
    """

    def __init__(self, serial_port):

        # Instantiate IKA driver
        self.IKA = RETControlViscHotplate(
            device_name = None,
            connection_mode = "serial",
            address = None,
            port = serial_port)

        # Initialize ROS subscriber
        rospy.Subscriber(
            "IKA_Commands",
            IKA_RET_CONTROL_VISC_Command,
            self.callback_commands)

        # Initialize ROS publisher
        self.pub = rospy.Publisher(
            "IKA_Readings",
            IKA_RET_CONTROL_VISC_Reading,
            queue_size = 10)
        
        # Sleeping rate
        self.rate = rospy.Rate(1)
        
        rospy.loginfo("IKA driver started")

        # Get data
        while not rospy.is_shutdown():
            
            temperature = self.IKA.get_temperature()
            stir_speed = self.IKA.get_speed()
            viscosity_trend = self.IKA.get_viscosity_trend()

            self.pub.publish(
                float(temperature),
                float(stir_speed),
                float(viscosity_trend))
            
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
        self.IKA.start_temperature_regulation()
        rospy.loginfo("Turning on Heating")

    def stop_heating(self):
        self.IKA.stop_temperature_regulation()
        rospy.loginfo("Turning off Heating")

    def start_stirring(self):
        self.IKA.start_stirring()
        rospy.loginfo("Turning on Stirring")

    def stop_stirring(self):
        self.IKA.stop_stirring()
        rospy.loginfo("Turning off Stirring")

    def set_speed(self, speed: int):
        self.IKA.set_speed(speed)
        rospy.loginfo("Setting Stirring To: " + str(speed) + "RPM")

    def set_temperature(self, temperature: int):
        self.IKA.set_temperature(temperature)
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
            # Not sure if this makes sense, maybe to 30ºC?
            self.set_temperature(0)
        else:
            rospy.loginfo("invalid command")

print("working")
