# external
import rospy
from pylabware import PPS4102

# Core
from roslabware_msgs.msg import FisherPP14102Cmd, FisherPP14102Reading


class PP14102Ros:
    """ROS Wrapper for Fisher Scientific PPS4102 Top Pan Balance Serial
    Driver Utilises ROS Topics to facilitate communication with balance
    using a serial driver."""

    def __init__(
        self,
        device_name: str,
        connection_mode: str,
        address: str,
        port: str,
        simulation: bool,
    ):
        # Instantiate balance driver
        self.balance = PPS4102(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port,
        )

        if simulation == "True":
            self.balance.simulation = True

        # connect
        self.balance.connect()
        self.balance.initialize_device()

        # Initialize ros subscriber of topic to which commands are published
        self.sub = rospy.Subscriber(
            name="fisher_pp14102_commands",
            data_class=FisherPP14102Cmd,
            callback=self.callback_commands,
        )

        # Initialize ros published for Balance responses (weights)
        self.pub = rospy.Publisher(
            name="fisher_pp14102_readings",
            data_class=FisherPP14102Reading,
            queue_size=10,
        )

        # Sleeping rate
        self.rate = rospy.Rate(1)

        rospy.loginfo("Fisher PP14102 balance driver started")

    def get_stable_mass(self):
        self.pub.publish(float(self.balance.get_stable_mass()))

    def get_mass(self):
        self.pub.publish(float(self.balance.get_mass()))

    def tare_balance(self):
        self.balance.tare_balance()
        rospy.loginfo("Zeroing Balance")

    def turn_off(self):
        self.balance.turn_off()
        rospy.loginfo("Balance going into standby")

    def turn_on(self):
        self.balance.turn_on()
        rospy.loginfo("Balance turning on")

    def callback_commands(self, msg):
        message = msg.balance_command

        if message == msg.ZERO:
            self.tare_balance()
        elif message == msg.BALANCE_ON:
            self.turn_on()
        elif message == msg.BALANCE_OFF:
            self.turn_off()
        elif message == msg.MASS_STABLE:
            self.get_stable_mass()
        elif message == msg.MASS:
            self.get_mass()
        else:
            rospy.loginfo("invalid command")
