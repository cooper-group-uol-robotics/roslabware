# external
import datetime
from typing import Optional, Union
import time
import rospy
from labmatic import LCMS

# Core
from roslabware_msgs.msg import (
    LcmsCmd,
    LcmsReading,
)

from std_msgs.msg import Bool

class LcmsRos:
    """
    ROS Wrapper for Serial Driver for waters-LCMS.
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "tcpip",
        address: str = "172.31.1.18", # IP address
        port: int = 8000, # Port
        experiment_name: str = "test",
        simulation: bool = False
    ):

        Create device object
        self.lcms = LCMS( 
            device_name = device_name, 
            connection_mode = connection_mode, 
            address= address, 
            port = port, 
            experiment_name=experiment_name
        )

        self.result_dict = None
        self.complete = False

        self.lcms.connect_socket()
        time.sleep(1)

        if not self.lcms.is_connected():
            rospy.loginfo("LCMS server not connected.")
        
        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="/lcms_command",
            data_class=LcmsCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="/lcms_info",
            data_class=LcmsReading,
            queue_size=10,
        )
        rospy.loginfo("LCMS-client ROS driver started.")

        self._task_complete_pub = rospy.Publisher(
            '/lcms/task_complete',
            Bool,
            queue_size=1)
        
        # Sleeping rate
        self.rate = rospy.Rate(2)

        # Get data
        while not rospy.is_shutdown():
            if self.result_dict is not None:
                lcmsmsg = LcmsReading()
                lcmsmsg.chemicals = self.result_dict[1]['chemicals']
                lcmsmsg.concentrtions = self.result_dict[1]['concentrations']
                lcmsmsg.y_values = self.result_dict[1]['y_values']
                self.pub.publish(lcmsmsg)
            self.rate.sleep()
            self._task_complete_pub.publish(self.complete)
            self.rate.sleep()

    def prep_analysis(self, num_samples=1):
        rospy.loginfo("in here! 1")
        self.complete = False
        self.result_dict = None
        _batch_file_create = self.lcms.create_batch_csv(num_samples=num_samples)
        rospy.sleep(2)
        if _batch_file_create:
            rospy.loginfo("Batch file created.")
        else:
            rospy.loginfo("Batch file not created.")
        #self.lcms.autosampler_initialise() # TODO maybe don't need this?
        self.complete = True
    
    def load_batch(self):
        self.complete = False
        _batch_load = self.lcms.autosampler_load()
        if _batch_load:
            rospy.loginfo("Batch loaded into LCMS.")
        else:
            rospy.loginfo("Batch load error.")
        self.complete = True

    def start_analysis(self):
        self.complete = False
        self.result_dict = self.lcms.get_lcms_results()
        if self.result_dict:
            rospy.loginfo("Analysis done and results received.")
        else:
            rospy.loginfo("Results not received.")
        self.complete = True

    def unload_batch(self):
        self.complete = False
        _batch_unload = self.lcms.autosampler_unload()
        if _batch_unload:
            rospy.loginfo("Batch unloaded from LCMS.")
        else:
            rospy.loginfo("Batch unload error.")
        self.complete = True


    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.lcms_command
        rospy.loginfo("Message received.")
        if not message == self._prev_msg: # TODO what if we do want to send the same msg twice? Use a time elapsed check (>15 secs)
            if message == msg.START_PREP:
                if msg.lcms_num_samples is not None:
                    self.prep_analysis(msg.lcms_num_samples)
                else:
                    self.prep_analysis()
            elif message == msg.LOAD_BATCH:
                self.load_batch()
            elif message == msg.UNLOAD_BATCH:
                self.unload_batch()
            elif message == msg.START_ANALYSIS:
                self.start_analysis()
            else:
                rospy.loginfo("invalid command")
            self._prev_msg = message    

rospy.loginfo("working")