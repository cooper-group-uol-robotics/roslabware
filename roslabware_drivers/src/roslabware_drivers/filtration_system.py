# external
from typing import Optional
from miscware import FiltrationSystem
import rospy
import serial

# Core
from roslabware_msgs.msg import (
    FiltrationCmd,
    FiltrationStatus)
from std_msgs.msg import Bool


class FiltrationRos:
    """
    ROS Wrapper for Driver for filtration Station.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        simulation: bool,
    ):

        self.filtration_system  = FiltrationSystem()

        self.filtration_system.connect()

        self.process_complete = False
        self.previous_command = None

        if simulation == "True":
            self.filtration_system.simulation = True

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="Filtration_Commands",
            data_class=FiltrationCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="Filtration_Status",
            data_class=FiltrationStatus,
            queue_size=10
        )

        rospy.loginfo("Filtration station Driver Started")

        self._task_complete_pub = rospy.Publisher(
            '/filtration/task_complete',
            Bool,
            queue_size=1)

        # Sleeping rate
        self.rate = rospy.Rate(1)

        # publish status of the valve
        while not rospy.is_shutdown():
            self._task_complete_pub.publish(self.process_complete)
            rospy.sleep(5)

    def main_filtration(self):
        self.process_complete = False
        rospy.loginfo("Running main filtration method.")
        self.filtration_system.main_filtration()
        rospy.sleep(3)
        self.process_complete = True

    def dry(self):
        self.process_complete = False
        rospy.loginfo("Drying.")
        self.filtration_system.dry()
        rospy.sleep(3)
        self.process_complete = True
    
    def timed_drain(self):
        self.process_complete = False
        rospy.loginfo("Running timed drain.")
        self.filtration_system.timed_drain()
        rospy.sleep(3)
        self.process_complete = True

    def drain(self):
        self.process_complete = False
        rospy.loginfo("Draining.")
        self.filtration_system.drain_on()
        rospy.sleep(3)
        self.process_complete = True

    def vacuum(self):
        self.process_complete = False
        rospy.loginfo("Vacuuming.")
        self.filtration_system.vac_pump_on()
        self.filtration_system.vac_valve_open()
        rospy.sleep(3)
        self.process_complete = True
    
    def stop(self):
        self.process_complete = False
        rospy.loginfo("Stopping all process.")
        self.filtration_system.stop()
        rospy.sleep(3)
        self.process_complete = True
    
    # Callback for subscriber.
    def callback_commands(self, msg):
        message = msg.filtration_system_command
        if not message == self.previous_command:
            if message == msg.MAIN_FILTRAION:
                self.process_complete = False
                self.main_filtration()
            elif message == msg.DRY:
                self.process_complete = False
                self.dry()
            elif message == msg.TIMED_DRAIN:
                self.process_complete = False
                self.timed_drain()
            elif message == msg.DRAIN:
                self.process_complete = False
                self.drain()
            elif message == msg.VACUUM:
                self.process_complete = False
                self.vacuum()
            elif message == msg.STOP:
                self.process_complete = False
                self.stop()
            else:
                rospy.loginfo("invalid command")
            self.previous_command = message

rospy.loginfo("working")