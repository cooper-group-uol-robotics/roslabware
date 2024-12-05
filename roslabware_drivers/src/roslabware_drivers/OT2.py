# external
import datetime
from typing import Optional, Union

import rospy
from labmatic import LCMS

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
        address: Optional[str] = "172.31.1.18", # IP address
        port: Union[str, int] = 8000, # Port
        simulation: bool = False,
        experiment_name: str = "test"
    ):

        # Create device object
        self.ot2 = OT2Client( 
            address= address
        )

        

        if not self.lcms.is_connected():
            rospy.loginfo("LCMS server - not connected.")


        
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
            # result, concentration = self.get_results()
            ot2msg = Ot2Status()
            #TODO ask Hatem about how to send this dict over via ROS messages...
            ot2msg.ot2_status = self.robot_status
            self.pub.publish(ot2msg)
            self.rate.sleep()
    
    # def get_results(self):
    #     return True, 0.52

    def robot_status(self):
        return ot2_status.BUSY

    
    def light_on(self):
        rospy.loginfo("Light-on command sent to OT2")


    def run_protocol(self):
        rospy.loginfo("Protocol sent to OT2")



    def move_home(self):
        rospy.loginfo("Homing command sentto OT2") 




    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.ot2_command

        if message == msg.RUN_PROTOCOL:
            self.run_protocol()
        elif message == msg.LIGHT_ON:
            self.light_on()
        elif message == msg.HOME_POSITION:
            self.move_home()
        else:
            rospy.loginfo("invalid command")

rospy.loginfo("working")
