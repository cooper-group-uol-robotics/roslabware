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

        self.filtration_system = FiltrationSystem(device_name="filtration_system", port="COM14", connection_mode="serial")

        self.filtration_system.connect()
        rospy.sleep(2)

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

    def main_filtration(self, id):
        rospy.loginfo("Running main filtration method.")
        self.filtration_system.main_filtration()
        rospy.sleep(3)
        complete = False
        while not complete:
            complete = self.filtration_system.check_status()
            time.sleep(1)
        self._task_complete_pub.publish(seq=id, complete=True)

    def dry(self, id):
        rospy.loginfo("Drying.")
        self.filtration_system.dry()
        rospy.sleep(3)
        complete = False
        while not complete:
            complete = self.filtration_system.check_status()
            time.sleep(1)
        self._task_complete_pub.publish(seq=id, complete=True)
    
    def timed_drain(self, id):
        rospy.loginfo("Running timed drain.")
        self.filtration_system.timed_drain()
        rospy.sleep(3)
        complete = False
        while not complete:
            complete = self.filtration_system.check_status()
            time.sleep(1)
        self._task_complete_pub.publish(seq=id, complete=True)

    def drain(self, id):
        rospy.loginfo("Draining.")
        self.filtration_system.drain_on()
        rospy.sleep(3)
        complete = False
        while not complete:
            complete = self.filtration_system.check_status()
            time.sleep(1)
        self._task_complete_pub.publish(seq=id, complete=True)

    def vacuum(self, id):
        rospy.loginfo("Vacuuming.")
        self.filtration_system.vac_pump_on()
        self.filtration_system.vac_valve_open()
        rospy.sleep(3)
        complete = False
        while not complete:
            complete = self.filtration_system.check_status()
            time.sleep(1)
        self._task_complete_pub.publish(seq=id, complete=True)
    
    def stop(self, id):
        rospy.loginfo("Stopping all process.")
        self.filtration_system.stop()
        rospy.sleep(3)
        complete = False
        while not complete:
            complete = self.filtration_system.check_status()
            time.sleep(1)
        self._task_complete_pub.publish(seq=id, complete=True)
    
    # Callback for subscriber.
    def callback_commands(self, msg):
        message = msg.filtration_system_command
        id = msg.seq
        if id > self._prev_id:
            if message == msg.MAIN_FILTRATION:
                self.main_filtration(id)
            elif message == msg.DRY:
                self.dry(id)
            elif message == msg.TIMED_DRAIN:
                self.timed_drain(id)
            elif message == msg.DRAIN:
                self.drain(id)
            elif message == msg.VACUUM:
                self.vacuum(id)
            elif message == msg.STOP:
                self.stop(id)
            else:
                rospy.loginfo("invalid command")
            self._prev_id = id

rospy.loginfo("working")