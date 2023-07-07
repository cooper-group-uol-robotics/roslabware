# external
from typing import Optional, Union

import rospy
from miscware import AutoSampler

# Core
from roslabware_msgs.msg import (
    InhouseAutosamplerCmd,
    InhouseAutosamplerReading,
)
from std_msgs.msg import Bool


class AutosamplerRos:
    """
    ROS Wrapper for Serial Driver for in-house built autosampler
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False,
    ):
        #
        # Create device object
        self.autosampler = AutoSampler(device_name=device_name, port=port, connection_mode=connection_mode, address=address)

        # TODO after (IF) API implementation
        # if simulation == "True":
        #     self.optimax.simulation = True
        self.autosampler.connect()

        rospy.sleep(2)

        self.autosampler.initialize_device()
        self.process_complete = False

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="inhouse_autosampler",
            data_class=InhouseAutosamplerCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="inhouse_autosampler_info",
            data_class=InhouseAutosamplerReading,
            queue_size=10,
        )

        self._task_complete_pub = rospy.Publisher(
            '/inhouse_autosampler/task_complete',
            Bool,
            queue_size=1)

        rospy.loginfo("Inhouse Autosampler Driver Started")

        while not rospy.is_shutdown():
            self._task_complete_pub.publish(self.process_complete)
            rospy.sleep(5)

    def homing_step(self):
        self.autosampler.home()
        rospy.loginfo("Added homing step")

    def add_move_step(self, x_pos, y_pos, z_pos):
        self.autosampler.move(x_pos, y_pos, z_pos)
        rospy.loginfo(f"Added moving step to position {x_pos}, {y_pos}, {z_pos}")

    def get_position(self):
        self.autosampler.get_position()
        rospy.loginfo("Getting the position")

    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.optimax_command
        if msg.target_x_position:
            target_x_position = msg.target_x_position
        if msg.target_y_position:
            target_y_position = msg.target_y_position
        if msg.target_z_position:
            target_z_position = msg.target_z_position

        if message == msg.HOME:
            self.homing_step()
        elif message == msg.MOVE:
            self.add_move_step(target_x_position, target_y_position, target_z_position)
        elif message == msg.GET_POSITION:
            self.get_position()
        else:
            rospy.loginfo("invalid command")


rospy.loginfo("working")
