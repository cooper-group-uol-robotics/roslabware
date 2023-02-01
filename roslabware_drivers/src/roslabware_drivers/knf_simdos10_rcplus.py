# external
from typing import Optional
from pylabware import RCPlus
import rospy

# Core
from roslabware_msgs.msg import (
    RCPlusCmd,
    RCPlusReading
)

# Constants - from Tecan pump, maybe not needed for KNF one
DEFAULT_SPEED = 40  # ml/min
#DEFAULT_RESOLUTION = "N2"


class RCPlusRos:
    """
    ROS Wrapper for Driver for KNF RC-Plus diaphragm pump.
    """

    def __init__(
        self,
        connection_mode: str,
        device_name: str,
        address: str,
        port: str,
        switch_address: str
    ):
        
        # Create device object
        self.knf = RCPlus(
            device_name=device_name,
            connection_mode=connection_mode,
            switch_address=switch_address,
            address=address,
            port=port
        )

        # Don't think any initial methods are needed for KNF...
        #self.knf.connect() # TODO make connect ros method
        #self.knf.set_speed(DEFAULT_SPEED) # TODO make speed ros method
        #self.knf.initialize_device() # TODO make initialise ros method

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="KNF Simdos 10 RC-Plus Commands",
            data_class=tecan_rcplus_command,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="KNF Simdos 10 RC-Plus Readings",
            data_class=tecan_rcplus_reading,
            queue_size=10
        )

        rospy.loginfo("KNF Simdos 10 RC-Plus Driver Started")

        # Get data - TODO why is this needed here?  What data should we be getting for the knf pump
        while not rospy.is_shutdown():
            plunger, valve = self.get_positions() #TODO make ros get_positions method.
            self.pub.publish(plunger, valve)
            rospy.loginfo(
                " Plunger position: "
                + str(plunger)
                + "| Valve position: "
                + str(valve))

            self.rate.sleep()

    def stop(self):
        """Stops executing any program/action immediately"""
        self.knf.stop()

    
    def dispense(
        self,
        volume: float,
        speed: Optional[float] = DEFAULT_SPEED
    ):
        """Dispense the specified volume with the defined speed
            Args:
                volume (float): volume to dispense in mL
                speed (float): speed in mL/min"""
        self.knf.set_dispense_mode()
        self.knf.set_dispence_volume(volume)
        dispense_time = (volume/speed)*60 # gives time to dispense in seconds
        self.knf.set_time(dispense_time)
        self.knf.start()

    #TODO what is this for? Ask Seb
    def callback_commands(self, msg):
        """Callback commands for susbcriber"""
        message = msg.tecan_command

        if message == msg.DISPENSE:
            self.dispense(msg.tecan_param)
        elif message == msg.WITHDRAW:
            self.withdraw(msg.tecan_param)
        else:
            rospy.loginfo("invalid command")