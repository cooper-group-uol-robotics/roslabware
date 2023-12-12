
# external
from typing import Optional, Union
import rospy
import time
from pylabware import Optimax

# Core
from roslabware_msgs.msg import (
    MettlerOptimaxCmd,
    MettlerOptimaxReading,
    MettlerOptimaxTask
)
from std_msgs.msg import Bool

class OptimaxRos:
    """
    ROS Wrapper for Serial Driver for Mettler Toledo XPR226
    DRQ dispensing system.
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False

    ):
        #
        # Create device object
        self.optimax = Optimax(device_name=device_name, connection_mode=connection_mode, address=address, port=port)
        
        # TODO after (IF) API implementation
        # if simulation == "True":
        #     self.optimax.simulation = True
        # self.optimax.connect()

        self.optimax.initialize_device()
        self._prev_id = -1

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="/mettler_optimax",
            data_class=MettlerOptimaxCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="/mettler_optimax_info",
            data_class=MettlerOptimaxReading,
            queue_size=10,
        )

        self._task_complete_pub = rospy.Publisher(
            name="/mettler_optimax/task_complete",
            data_class=MettlerOptimaxTask,
            queue_size=10
        )

        rospy.loginfo("Mettler Optimax Driver Started")

        while not rospy.is_shutdown():
            self.experiment_ended = self.optimax.end_of_experiment_check()
            self.pub.publish(exp_ended=self.experiment_ended)
            rospy.sleep(3)

    def _create_experiment(self):
        self.optimax._create_experiment()
        rospy.loginfo(f"Created new experiment.")
        rospy.sleep(3)

    def _add_temp_step(self, temperature, duration=None):
        self.optimax._add_temperature_step(temperature, duration)
        rospy.loginfo(f"Added temperature step with temperature {temperature} ºC")

    def _add_stir_step(self, speed, duration):
        self.optimax._add_stirring_step(speed, duration)
        rospy.loginfo(f"Added stirring step with speed {speed} RPM")

    def _add_wait_step(self, time):
        self.optimax._add_waiting_step(time)
        rospy.loginfo(f"Added waiting step with duration {time} Minutes")

    def _add_sampling_step(self, dilution):
        self.optimax._add_sampling_step(dilution)
        rospy.loginfo(f"Added sampling step with dilution {dilution}")
    
    def _add_end_experiment_step(self):
        self.optimax._add_end_experiment_step()
        rospy.loginfo("Added end experiment step")

    def _start_experiment(self):
        self.optimax.start()
        rospy.loginfo("Experiment Started")

    def stop_experiment(self, id):
        self.optimax.stop()
        finished = self.optimax.end_of_experiment_check()
        while not finished:
            finished = self.optimax.end_of_experiment_check()
        rospy.loginfo("Experiment Stopped")
        for i in range(10):
            self._task_complete_pub(seq=id, complete=True)
        
    def heat_wait(self, id, temp, stir_speed, wait_duration):
        self._create_experiment()
        self._add_stir_step(stir_speed, 20)
        self._add_temp_step(temp)
        self._add_wait_step(wait_duration)
        self._add_end_experiment_step()
        self._start_experiment()
        time.sleep(15)
        for i in range(10):
            self._task_complete_pub(seq=id, complete=True)

    def sample(self, id, temp, stir_speed, dilution):
        self._create_experiment()
        self._add_stir_step(stir_speed, 20)
        self._add_temp_step(temp)
        self._add_sampling_step(dilution)
        self._add_end_experiment_step()
        self._start_experiment()
        finished = self.optimax.end_of_experiment_check()
        while not finished:
            finished = self.optimax.end_of_experiment_check()
        for i in range(10):
            self._task_complete_pub(seq=id, complete=True)
                
    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.optimax_command
        id = msg.seq
        if msg.temperature is not None:
            temp = msg.temperature
        if msg.stir_speed is not None:
            stir_speed = msg.stir_speed
        if msg.temp_duration is not None:
            temp_duration = msg.temp_duration
        if msg.stir_duration is not None:
            stir_duration = msg.stir_duration
        if msg.wait_duration is not None:
            wait_duration = msg.wait_duration
        if msg.dilution is not None:
            dilution = msg.dilution

        if id > self._prev_id:
            if message == msg.HEAT_WAIT:
                self.heat_wait(id, temp, stir_speed, wait_duration)
            elif message == msg.SAMPLE:
                self.sample(id, temp, stir_speed, dilution)
            elif message == msg.STOP:
                self.stop_experiment(id)
            else:
                rospy.loginfo("invalid command")
            
            self._prev_id = id
            
rospy.loginfo("working")