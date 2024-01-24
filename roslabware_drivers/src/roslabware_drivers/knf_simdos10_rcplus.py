# external
from typing import Optional
from pylabware import RCPlus
import rospy
import time

# Core
from roslabware_msgs.msg import (
    KnfRCPlusCmd,
    KnfRCPlusReading,
    KnfRCPlusTask
)
from std_msgs.msg import Bool

# Constants - from Tecan pump, maybe not needed for KNF one
DEFAULT_SPEED = 95  # ml/min


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

        self._prev_id = -1

        # Initialise connection for KNF.
        self.knf.connect()  
        time.sleep(1)

        rospy.loginfo("KNF pump connected: %s", self.knf.is_connected())

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="/knf_rcplus_command",
            data_class=KnfRCPlusCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for status
        self.pub = rospy.Publisher(
            name="/knf_rcplus_reading",
            data_class=KnfRCPlusReading,
            queue_size=10
        )

        rospy.loginfo("KNF Simdos 10 RC-Plus Driver Started")

        self._task_complete_pub = rospy.Publisher(
            name="/knf_rcplus/task_complete",
            data_class=KnfRCPlusTask,
            queue_size=10
        )

        # Sleeping rate
        self.rate = rospy.Rate(1)

    def stop(self, id):
        """ Stops executing any program/action immediately. """
        self.knf.stop()
        for i in range(10):
            self._task_complete_pub.publish(seq=id, complete=True)

    def dispense(
        self,
        id,
        volume: float,
        speed: Optional[float] = DEFAULT_SPEED
    ):
        """ Dispense the specified volume with the defined speed.

        :param volume (float): volume to dispense in mL 
        :param speed Optional(float): speed in mL/min
        """
        operation_complete = False
        self.knf.set_dispense_mode() # Set dispense in mL and time
        rospy.sleep(3)
        self.knf.set_dispense_volume(volume*1000) # Convert dispense mL to uL (pylabware driver takes uL)
        rospy.sleep(3)
        dispense_time = (volume/speed)*60 # Gives time to dispense in seconds
        self.knf.set_time(dispense_time)
        rospy.sleep(3)
        self.knf.start()
        rospy.sleep(3)
        while operation_complete is False:
            status = self.knf.get_status(1) 
            if 'FALSE Motor turns' in str(status):
                operation_complete = True
        for i in range(10):
            self._task_complete_pub.publish(seq=id, complete=True)

    def check_idle(self):
        """Reads back pump operation status."""
        status = self.knf.get_status(1)
        if 'False Motor turns' in status:
            self.operation_complete = True
            return True
        else:
            self.operation_complete = False
            return False

    def callback_commands(self, msg):
        """ Callback commands for susbcriber. """
        message = msg.knf_command
        rospy.loginfo("Message received.")
        id = msg.seq
        if id > self._prev_id:
            if message == msg.DISPENSE:
                self.dispense(id, msg.knf_volume)
            elif message == msg.STOP:
                self.stop(id)
            elif message == msg.CHECK_IDLE:
                self.check_idle()
            else:
                rospy.loginfo("Invalid command.")
            self._prev_id = id
        