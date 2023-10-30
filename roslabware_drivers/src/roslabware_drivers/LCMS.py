# external
from typing import Optional, Union

import rospy
# from labmatic import LCMS

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
        address: Optional[str] = None, # IP address
        port: Union[str, int] = None, # Port
        simulation: bool = False,
        experiment_name: str = "test"
    ):


        self.result = False
        self.concentration = None
        # Create device object
        # self.lcms = LCMS(device_name = device_name, connection_mode = connection_mode, address= address, port = port, experiment_name=experiment_name)

        # self.lcms.connect_socket()

        # if not self.lcms.is_connected():
        #     rospy.loginfo("LCMS server - not connected")
        
        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="lcms_command",
            data_class=LcmsCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="lcms_info",
            data_class=LcmsReading,
            queue_size=10,
        )
        rospy.loginfo("LCMS-client ROS Driver Started")

        self._task_complete_pub = rospy.Publisher(
            '/lcms/task_complete',
            Bool,
            queue_size=1)
        
        # Sleeping rate
        self.rate = rospy.Rate(0.3)

        # Get data
        while not rospy.is_shutdown():
            result, concentration = self.get_results()
            lcmsmsg = LcmsReading()
            lcmsmsg.result = result # self.result
            lcmsmsg.paracetamol_concentration = concentration # self.concentration
            self.pub.publish(lcmsmsg)
            self.rate.sleep()
    
    def get_results(self):
        return True, 0.52

    def prep_analysis(self):
        _batch_file_create = self.lcms.create_batch_csv()
        rospy.sleep(2)
        if _batch_file_create:
            rospy.loginfo("batch file created")
        else:
            rospy.loginfo("batch file not created")
        self.lcms.autosampler_initialise()
    
    def load_batch(self):
        _batch_load = self.lcms.autosampler_load()
        if _batch_load:
            rospy.loginfo("batch loaded into LCMS")
        else:
            rospy.loginfo("batch load error")

    def unload_batch(self):
        _batch_unload = self.lcms.autosampler_unload()
        if _batch_unload:
            rospy.loginfo("batch loaded into LCMS")
        else:
            rospy.loginfo("batch load error")

    def start_analysis(self):
        self.result,self.concentration = self.lcms.send_csv_receive_conc()
        if self.result:
            rospy.loginfo("Analysis done and results received")
        else:
            rospy.loginfo("results not received")

    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.lcms_command
        param = msg.lcms_param

        if message == msg.START_PREP:
            self.prep_analysis()
        elif message == msg.LOAD_BATCH:
            self.load_batch()
        elif message == msg.UNLOAD_BATCH:
            self.unload_batch()
        elif message == msg.START_ANALYSIS:
            self.start_analysis()
        else:
            rospy.loginfo("invalid command")

rospy.loginfo("working")
