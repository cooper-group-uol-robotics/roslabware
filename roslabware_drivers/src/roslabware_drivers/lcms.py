# external
from typing import Optional
from labmatic import LCMS
import rospy


# Core
from roslabware_msgs.msg import (
    LcmsCmd,
    LcmsStatus)
from std_msgs.msg import Bool


class LcmsRos:
    """
    ROS Wrapper for Driver for LCMS.
    """

    def __init__(self):

        # Create device object
        self.lcms = LCMS()

        self.lcms.connect_socket()
        if self.lcms.is_connected():
            rospy.loginfo("LCMS PC is now connected")
        else:
            rospy.loginfo("error: LCMS PC is not connected")

            # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_optimax",
            data_class=LcmsCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="lcms_info",
            data_class=LcmsStatus,
            queue_size=10,
        )

        rospy.loginfo("LCMS RosLabware Driver is now Started")

    def start_analysis(self):
        labmatic_methods = ['autosampler_load', 'create_batch_csv', 'move_csv_to_queue',
                          'qld_to_xml', 'xml_parse', 'concentration_from_area','autosampler_unload']

        # Call the methods in sequence
        for method in labmatic_methods:
            function = getattr(self.lcms, method)
            function()

    def stop_analysis(self):
        pass

    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.lcms_command

        if message == msg.START:
            self.start_analysis()
        elif message == msg.STOP:
            self.stop_analysis()
        else:
            rospy.loginfo("invalid command")


rospy.loginfo("working")
