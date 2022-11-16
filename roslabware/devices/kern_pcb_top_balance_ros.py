# external
import rospy

from ...pylabware.devices.kern_pcb_top_balance import KernTopBalance

# Core
from ..msgs.kern_pcb_top_balance import kern_command, kern_reading


class KernTopBalanceRos:
    """
    ROS wrapper class for controlling Kern PCB Top Balance
    """

    def __init__(self, port):

        self.pub = None
        self.tared = False

        # Create object
        self.kern = KernTopBalance(
            device_name=None, connection_mode="serial", address=None, port=port
        )

        # Initialize ros subscriber of topic to which commands are published
        rospy.Subscriber("Kern_Commands", kern_command, self.callback_commands)

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher("Kern_Weights", kern_reading, queue_size=10)

        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Kern driver started")

        # Publish values every second TODO SMZ Check the mass publishing
        while not rospy.is_shutdown():
            if self.tared:
                self.pub.publish(float(self.kern.get_mass()))
            else:
                self.pub.publish(float(-1))

            self.rate.sleep()

    def tare_balance(self):
        self.tared = True
        rospy.sleep(2)

        self.kern.tare_balance()
        rospy.loginfo("Zeroing Balance")

        rospy.sleep(2)
        self.tared = False

    def callback_commands(self, msg):
        if msg.kern_command == msg.TARE_BALANCE:
            self.tare_balance()
        else:
            rospy.loginfo("invalid command")
