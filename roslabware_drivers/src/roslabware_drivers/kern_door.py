# external
# written by satheesh
from typing import Optional, Union

import rospy

import serial

# Core
from roslabware_msgs.msg import (
    KernDoorCmd,
    KernDoorStatus
)
from std_msgs.msg import Bool

class KernDoorRos:
    """
    ROS wrapper and python driver class for controlling weighing station door
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False,
    ):

        # Instantiate IKA driver
        self.door = serial.Serial(port=port, baudrate=9600, timeout=None)

        self.process_complete = False
        self._prev_msg = None
        

        # Initialize ros subscriber of topic to which commands are published
        self.subs = rospy.Subscriber(
            name="kern_door_Commands",
            data_class=KernDoorCmd,
            callback=self.callback_commands,
        )

        # Initialize ros published for balance responses (weights)
        self.pub = rospy.Publisher(
            name="kern_Door_Status",
            data_class=KernDoorStatus,
            queue_size=10
        )

        self._task_complete_pub = rospy.Publisher(
            '/kern_door/task_complete',
            Bool,
            queue_size=1)


        # Initialize rate object for consistent timed looping
        self.rate = rospy.Rate(10)

        rospy.loginfo("Kern door driver started")
        while not rospy.is_shutdown():
            self._task_complete_pub.publish(True)
            rospy.sleep(5)
        #initialize device

    def open_door(self):
        self.door.write((bytes("bopen", 'utf-8')))
        rospy.loginfo("open_door_message_sent_to_miscware")
        self.pub.publish( status = 'Door_Opened')
        rospy.sleep(5)
        self.process_complete = True

    def close_door(self):
        self.door.write((bytes("bclose", 'utf-8')))
        rospy.loginfo("close_door_message_sent_to_miscware")
        #if serial msg received:
        self.pub.publish(status = 'Door_Closed')
        rospy.sleep(5)
        self.process_complete = True

    def callback_commands(self, msg):
        message = msg.kern_door_command
        rospy.loginfo("message_received")
        if not message == self._prev_msg:
            if message == msg.OPEN_DOOR:
                self.process_complete = False
                rospy.loginfo("open_door_message received")
                self.open_door()
            elif message == msg.CLOSE_DOOR:
                self.process_complete = False
                rospy.loginfo("close_door_message received")
                self.close_door()
            else:
                rospy.loginfo("Invalid command")
            self._prev_msg = message