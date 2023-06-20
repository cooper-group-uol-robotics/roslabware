# external
from typing import Optional
from miscware import FiltrationSystem
import rospy


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

        # Create device object
        self.filtration_system = FiltrationSystem(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port
        )

        self.process_complete = False

        if simulation == "True":
            self.filtration_system.simulation = True

        # initialize base valve
        self.filtration_system.connect()
        self.filtration_system.initialize_device()

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

    def drain(self):
        self.filtration_system.start_drain_system()
        rospy.loginfo("Draining")
        rospy.sleep(5)
        self.process_complete = True

    def vacuum(self):
        self.filtration_system.start_vacuum_system()
        rospy.loginfo("Vacuuming")
        rospy.sleep(5)
        self.process_complete = True
    
    def stop(self):
        self.filtration_system.stop_all()
        rospy.loginfo("Stopping all process")
        rospy.sleep(5)
        self.process_complete = True
    

    # Callback for subscriber.
    def callback_commands(self, msg):
        message = msg.filtration_system_command
        self.process_complete = False

        if message == msg.DRAIN:
            self.open_valve()
        elif message == msg.VACUUM:
            self.close_valve()
        elif message == msg.STOP:
            self.close_valve()
        else:
            rospy.loginfo("invalid command")

rospy.loginfo("working")