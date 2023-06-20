# external
from typing import Optional
from miscware import BaseValve
import rospy
import serial


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
        # self.base_valve = BaseValve(
        #     device_name=device_name,
        #     connection_mode=connection_mode,
        #     address=address,
        #     port=port
        # )

        #self.process_complete = False
        self.base_valve  = serial.Serial(port=port, baudrate=9600, timeout=None)

        self.process_complete = False

        if simulation == "True":
            self.base_valve.simulation = True



        # initialize base valve
        # self.base_valve.connect()
        # self.base_valve.initialize_device()

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="/Optimax_BaseValve_Commands",
            data_class=BaseValveCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="/Optimax_BaseValve_Status",
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
            self._task_complete_pub.publish(True)
            rospy.sleep(5)

    def _open_valve(self):
        self.base_valve.write((bytes("<o>", 'utf-8')))

        # self.base_valve.connect()
        # self.base_valve.open_valve()
        #if serial msg received:
        rospy.loginfo("open_valve_message_sent_to_miscware")
        # self.base_valve.disconnect()
        rospy.sleep(5)
        self.process_complete = True

    def _close_valve(self):
        self.base_valve.write((bytes("<c>", 'utf-8')))
        # self.base_valve.connect()
        # self.base_valve.close_valve()
        #if serial msg received:
        rospy.loginfo("close_valve_message_sent_to_miscware")
        # self.base_valve.disconnect()
        rospy.sleep(5)
        self.process_complete = True
    

    # Callback for subscriber.
    def callback_commands(self, msg):
        command = msg.valve_command
        ref_open = msg.OPEN
        ref_close = msg.CLOSE
        if command == ref_open:
            self.process_complete = False
            self._open_valve()
            rospy.loginfo("open_message_received")
        elif command == ref_close:
            self.process_complete = False
            self._close_valve()
            rospy.loginfo("close_message_received")
        else:
            rospy.loginfo("invalid command")

#rospy.loginfo("working")