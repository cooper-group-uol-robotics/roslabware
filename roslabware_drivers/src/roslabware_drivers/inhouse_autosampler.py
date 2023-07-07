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
from geometry_msgs.msg import Vector3


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
        message = self.autosampler.get_position()
        # removing excess text from the start of string
        if "Current Position: " in message:
            message = message[len("Current Position: "):]
            coordinates = message.split()
            coord_vector = Vector3
            coord_vector.x = float(coordinates[0])
            coord_vector.y = float(coordinates[1])
            coord_vector.z = float(coordinates[2])
            self.pub.publish(coord_vector)
        rospy.loginfo("Position published to /inhouse_autosampler_info")

    # Callback for subscriber.
    def callback_commands(self, msg):
        message = msg.autosampler_command
        if msg.position:
            target_x_position = msg.position.x
            target_y_position = msg.position.y
            target_z_position = msg.position.z

        if message == msg.HOME:
            self.homing_step()
        elif message == msg.MOVE:
            self.add_move_step(target_x_position, target_y_position, target_z_position)
        elif message == msg.GET_POSITION:
            self.get_position()
        else:
            rospy.loginfo("invalid command")


rospy.loginfo("working")
