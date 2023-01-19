# external
import rospy
from pylabware import RCPlus

# Core
from roslabware_msgs.msg import (
    KNFSimdos10RCPlusCmd,
    KNFSimdos10RCPlusReading)


class RCPlusRos:
    """
    ROS Wrapper for Driver for KNF Simdos10 RC plus syringe pump.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        switch_address: str,
        simulation: bool
    ):

        # Create device object
        self.pump = RCPlus(
            device_name=device_name,
            connection_mode=connection_mode,
            switch_address=switch_address,
            address=address,
            port=port
        )

        if simulation == "True":
            self.tecan.simulation = True

        # Add syringe size attribute
        self.pump.connect()
        self.pump.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="KNF Simdos10 RCPlus Commands",
            data_class=KNFSimdos10RCPlusCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="KNF Simdos10 RCPlus Commands",
            data_class=KNFSimdos10RCPlusReading,
            queue_size=10
        )

        rospy.loginfo("KNF Simdos RCplus Driver Started")

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

    def dispense(
            self,
            volume: int,
            speed: int,
            time: float = None,
            mode: str = "run"
    ):
        """Starts dispensing a specifiec volume with a specified
        speed or time based
        TODO use mode here"""

        if not time:
            self.pump.set_run_mode()
            self.pump.set_dispense_volume(volume=volume)
            self.pump.set_speed(speed=speed)
            self.pump.start()
        else:
            self.pump.set_run_mode()
            self.pump.set_time(time=time)
            self.pump.start()

    def get_counters(self):
        """Gets volume counter"""
        volume_counter = self.pump.get_volume_counter()
        time_counter = self.pump.get_time_counter()

        return volume_counter, time_counter

    def callback_commands(self, msg):
        """Callback commands for susbcriber"""
        message = msg.knf_command

        if message == msg.DISPENSE:
            self.dispense(
                msg.knf_volume,
                msg.knf_speed,
                msg.knf_time)
        elif message == msg.GET_COUNTERS:
            self.get_counters()
        else:
            rospy.loginfo("invalid command")
