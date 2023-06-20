# external
from typing import Optional
from miscware import BaseValve
import rospy


# Core
from roslabware_msgs.msg import (
    BaseValveCmd,
    BaseValveStatus)
from std_msgs.msg import Bool



class BaseValveRos:
    """
    ROS Wrapper for Driver for Tecan XLP6000 syringe pump.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        simulation: bool,
    ):

        # Create device object
        self.base_valve = BaseValve(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port
        )

        self.process_complete = False

        if simulation == "True":
            self.base_valve.simulation = True

        # initialize base valve
        self.base_valve.connect()
        self.base_valve.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="Optimax_BaseValve_Commands",
            data_class=BaseValveCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="Optimax_BaseValve_Status",
            data_class=BaseValveStatus,
            queue_size=10
        )

        rospy.loginfo("Base Valve Driver Started")

        self._task_complete_pub = rospy.Publisher(
            '/base_valve/task_complete',
            Bool,
            queue_size=1)

        # Sleeping rate
        self.rate = rospy.Rate(1)

        # publish status of the valve
        while not rospy.is_shutdown():
            self._task_complete_pub.publish(self.process_complete)
            rospy.sleep(5)

    def open_valve(self):
        self.base_valve.open_valve()
        rospy.loginfo("Valve Opening")
        rospy.sleep(5)
        self.process_complete = True

    def close_valve(self):
        self.base_valve.close_valve()
        rospy.loginfo("Valve Closing")
        rospy.sleep(5)
        self.process_complete = True
    

    # Callback for subscriber.
    def callback_commands(self, msg):
        message = msg.base_valve_command
        self.process_complete = False

        if message == msg.OPEN:
            self.open_valve()
        elif message == msg.CLOSE:
            self.close_valve()
        else:
            rospy.loginfo("invalid command")

rospy.loginfo("working")