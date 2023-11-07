# external
from typing import Optional
from pylabware import RCPlus
import rospy

# Core
from roslabware_msgs.msg import (
    KnfRCPlusCmd,
    KnfRCPlusReading
)
from std_msgs.msg import Bool

# Constants - from Tecan pump, maybe not needed for KNF one
DEFAULT_SPEED = 90  # ml/min


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
        switch_address: str,
        simulation: bool,
    ):
        
        # Create device object
        self.knf = RCPlus(
            device_name=device_name,
            connection_mode=connection_mode,
            switch_address=switch_address,
            address=address,
            port=port
        )

        if simulation == "True":
            self.knf.simulation = True

        # Don't think any initialisation methods are needed for KNF.
        #self.knf.connect() # TODO check

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="KNF Simdos 10 RC-Plus Commands",
            data_class=KnfRCPlusCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="KNF Simdos 10 RC-Plus Readings",
            data_class=KnfRCPlusReading,
            queue_size=10
        )

        rospy.loginfo("KNF Simdos 10 RC-Plus Driver Started")

        self._task_complete_pub = rospy.Publisher(
            '/knf_rcplus/task_complete',
            Bool,
            queue_size=1)

        # Sleeping rate
        self.rate = rospy.Rate(1)

        # Get data - TODO What data should we be publishing for the knf pump? maybe dispensed volume?  if it is busy of not?
        while not rospy.is_shutdown():
            self._task_complete_pub.publish(self.operation_complete)
            rospy.sleep(5)


    def stop(self):
        """ Stops executing any program/action immediately. """
        self.knf.stop()

    
    def dispense(
        self,
        volume: float,
        speed: Optional[float] = DEFAULT_SPEED
    ):
        """ Dispense the specified volume with the defined speed.
        :param volume (float): volume to dispense in mL
        :param speed Optional(float): speed in mL/min
        """
        self.operation_complete = False
        self.knf.set_dispense_mode() # set dispense in mL and time
        self.knf.set_dispense_volume(volume)
        dispense_time = (volume/speed)*60 # gives time to dispense in seconds
        self.knf.set_time(dispense_time)
        self.knf.start()
        while not self.operation_complete:
            status = self.knf.get_status(1) #TODO check how this actually works... what status returned when it is pumping vs not pumping?
            if 'Motor' in status:
                self.operation_complete = False
            else:
                self.operation_complete = True

    def check_idle(self):
        """Reads back pump operation status."""
        status = self.knf.get_status(1) #TODO check how this actually works... what status returned when it is pumping vs not pumping?
        if 'Motor' in status:
            self.operation_complete = False
            return False
        else:
            self.operation_complete = True
            return True

    def callback_commands(self, msg):
        """ Callback commands for susbcriber. """
        message = msg.knf_command

        if message == msg.DISPENSE:
            self.dispense(msg.knf_volume)
        elif message == msg.STOP:
            self.stop()
        elif message == msg.CHECK_IDLE:
            self.check_idle()
        else:
            rospy.loginfo("invalid command")
        