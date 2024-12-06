# external
import datetime
from typing import Optional, Union

import rospy
from OT2_client import OT2Client

# Core
from roslabware_msgs.msg import (
    Ot2Cmd,
    Ot2Status,
)

from std_msgs.msg import Bool

class OT2Ros:
    """
    ROS Wrapper for Serial Driver for waters-LCMS.
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "tcpip",
        address: Optional[str] = "169.254.227.210", # IP address
        port: Union[str, int] = 8000, # Port
        simulation: bool = False,
        experiment_name: str = "test"
    ):

        # Create device object
        self.robot = OT2Client( 
            ip= address, device_name=device_name
        )

        rospy.loginfo(f"roslabware pinging the Device : {device_name}")

        
        # rospy.loginfo(f"Device: {device_name} - connected.")

        if self.robot.ot2_connected:
            rospy.loginfo(f"Device: {device_name} - connected.")
        else:
            rospy.loginfo(f"Device: {device_name} - not connected.")


        
        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="Ot2_command",
            data_class=Ot2Cmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="ot2_info",
            data_class=Ot2Status,
            queue_size=10,
        )
        rospy.loginfo("OT2-client ROS driver started.")

        self._task_complete_pub = rospy.Publisher(
            '/ot2/task_complete',
            Bool,
            queue_size=1)
        
        # Sleeping rate
        self.rate = rospy.Rate(0.3)

        # Get data
        while not rospy.is_shutdown():
            # print(f"robot status: {self.robot.ot2.get_robot_status}")
            # result, concentration = self.get_results()
            ot2msg = Ot2Status()
            #TODO ask Hatem about how to send this dict over via ROS messages...
            ot2msg.ot2_status = self.robot_status
            self.pub.publish(ot2msg)
            self.rate.sleep()
    
    # def get_results(self):
    #     return True, 0.52

    def robot_status(self):
        if self.robot.ot2.get_robot_status == "RUNNING":
            status = Ot2Status.RUNNING
        elif self.robot.ot2.get_robot_status == "IDLE":
            status = Ot2Status.IDLE
        elif self.robot.ot2.get_robot_status == "FINISHING":
            status = Ot2Status.FINISHING
        elif self.robot.ot2.get_robot_status == "SUCCEEDED":
            status = Ot2Status.SUCCEEDED
        elif self.robot.ot2.get_robot_status == "FAILED":
            status = Ot2Status.FAILED
        elif self.robot.ot2.get_robot_status == "PAUSED":
            status = Ot2Status.PAUSED
        elif self.robot.ot2.get_robot_status == "STOPPING":
            status = Ot2Status.STOPPING
        return 0
    
    
    def light_on(self):
        self.robot.ot2.change_lights_status(True)

    def run_protocol(self, id):
        self.robot.actionCallback("run_protocol", )
        rospy.loginfo("Protocol sent to OT2")

    def move_home(self):
        rospy.loginfo("Homing command sent to OT2") 

    # Callback for subscriber.
    def callback_commands(self, msg):

        command = msg.ot2_command

        if command == msg.RUN_PROTOCOL:
            id = msg.protocol_id
            self.run_protocol(id)
        elif command == msg.LIGHT_ON:
            self.light_on()
        elif command == msg.HOME_POSITION:
            self.move_home()
        else:
            rospy.loginfo("invalid command")

rospy.loginfo("working")
