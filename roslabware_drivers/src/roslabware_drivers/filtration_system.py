# external
from typing import Optional
from miscware import FiltrationSystem
import rospy
import serial
import time

# Core
from roslabware_msgs.msg import (
    FiltrationCmd,
    FiltrationStatus,
    FiltrationTask
)
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

        self.filtration_system = FiltrationSystem(device_name="filtration_system", port="COM6", connection_mode="serial")

        self.filtration_system.connect()
        rospy.sleep(2)

        self.complete = False
        self._prev_id = -1

        if simulation == "True":
            self.filtration_system.simulation = True

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="/filtration_command",
            data_class=FiltrationCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="/filtration_status",
            data_class=FiltrationStatus,
            queue_size=10
        )

        rospy.loginfo("Filtration station driver Started")

        self._task_complete_pub = rospy.Publisher(
            name="/filtration/task_complete",
            data_class=FiltrationTask,
            queue_size=10)

        # Sleeping rate
        self.rate = rospy.Rate(1)

        # publish status of the valve
        while not rospy.is_shutdown():
            self.complete = self.filtration_system.check_status()
            self._task_complete_pub.publish(self.complete)
            rospy.sleep(5)

    def main_filtration(self):
        rospy.loginfo("Running main filtration method.")
        self.complete = False
        self.filtration_system.main_filtration()
        rospy.sleep(3)

    def dry(self):
        rospy.loginfo("Drying.")
        self.complete = False
        self.filtration_system.dry()
        rospy.sleep(3)
    
    def timed_drain(self):
        rospy.loginfo("Running timed drain.")
        self.complete = False
        self.filtration_system.timed_drain()
        rospy.sleep(3)

    def drain(self):
        rospy.loginfo("Draining.")
        self.complete = False
        self.filtration_system.drain_on()
        rospy.sleep(3)

    def vacuum(self):
        rospy.loginfo("Vacuuming.")
        self.complete = False
        self.filtration_system.vac_pump_on()
        self.filtration_system.vac_valve_open()
        rospy.sleep(3)
    
    def stop(self):
        rospy.loginfo("Stopping all process.")
        self.complete = False
        self.filtration_system.stop()
        rospy.sleep(3)
    
    # Callback for subscriber.
    def callback_commands(self, msg):
        message = msg.filtration_system_command
        id = msg.seq
        if id > self._prev_id:
            if message == msg.MAIN_FILTRATION:
                self.main_filtration()
            elif message == msg.DRY:
                self.dry()
            elif message == msg.TIMED_DRAIN:
                self.timed_drain()
            elif message == msg.DRAIN:
                self.drain()
            elif message == msg.VACUUM:
                self.vacuum()
            elif message == msg.STOP:
                self.stop()
            else:
                rospy.loginfo("invalid command")
            self._prev_id = id

rospy.loginfo("working")