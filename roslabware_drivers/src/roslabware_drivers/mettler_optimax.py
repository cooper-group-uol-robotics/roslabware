
# external
from typing import Optional, Union
import rospy
from pylabware import Optimax

# Core
from roslabware_msgs.msg import (
    MettlerOptimaxCmd,
    MettlerOptimaxReading,
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
        self.process_complete = False
        self._prev_message = None

        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_optimax",
            data_class=MettlerOptimaxCmd,
            callback=self.callback_commands,
        )

        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="mettler_optimax_info",
            data_class=MettlerOptimaxReading,
            queue_size=10,
        )

        self._task_complete_pub = rospy.Publisher(
            '/mettler_optimax/task_complete',
            Bool,
            queue_size=1)

        rospy.loginfo("Mettler Optimax Driver Started")

        while not rospy.is_shutdown():
            # plunger, valve = self.get_positions()

            self.process_complete = self.optimax.end_of_experiment_check()

            self._task_complete_pub.publish(self.process_complete)
            rospy.sleep(5)

    def create_experiment(self):
        self.optimax._create_experiment()
        rospy.loginfo(f"Created new experiment.")

    def add_temp_step(self, temperature, duration):
        self.optimax._add_temperature_step(temperature, duration)
        rospy.loginfo(f"Added temperature step with temperature {temperature} ºC")

    def add_stir_step(self, speed, duration):
        self.optimax._add_stirring_step(speed, duration)
        rospy.loginfo(f"Added stirring step with speed {speed} RPM")

    def add_wait_step(self, time):
        self.optimax._add_waiting_step(time)
        rospy.loginfo(f"Added waiting step with duration {time} Minutes")

    def add_sampling_step(self, dilution):
        self.optimax._add_sampling_step(dilution)
        rospy.loginfo(f"Added sampling step with dilution {dilution}")
    
    def add_end_experiment_step(self):
        self.optimax._add_end_experiment_step()
        rospy.loginfo("Added end experiment step")

    def start_experiment(self):
        self.optimax.start()
        rospy.loginfo("Experiment Started")

    def stop_experiment(self):
        self.optimax.stop()
        rospy.loginfo("Experiment Stopped") 

    def para_heat_wait(self, temp, stir_speed, wait_duration):
        self.create_experiment()
        self.add_stir_step(stir_speed, 20)
        self.add_temp_step(temp, 10)
        self.add_wait_step(wait_duration)
        self.add_end_experiment_step()
        self.start_experiment()

    def para_sample(self, temp, stir_speed, dilution):
        self.create_experiment()
        self.add_stir_step(stir_speed, 20)
        self.add_temp_step(temp, 10)
        self.add_sampling_step(dilution)
        self.add_end_experiment_step()
        self.start_experiment()

    def paracetamol_synthesis(self): # Temporary method for whole paracetamol synthesis.
        self.add_stir_step(300, 20)
        self.add_temp_step(120,10)
        self.add_wait_step(60)
        self.add_stir_step(300,20)
        self.add_temp_step(5,30)
        self.add_wait_step(240)
        self.add_end_experiment_step()
        self.start_experiment()
                
    # Callback for subscriber.
    def callback_commands(self, msg):

        message = msg.optimax_command
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

        
        if not message == self._prev_msg: # TODO what if we do want to send the same msg twice? Use a time elapsed check (>15 secs)
            if message == msg.ADD_TEMP_STIR:
                self.add_stir_step(stir_speed, stir_duration)
                self.add_temp_step(temp, temp_duration)
            elif message == msg.ADD_END:
                self.add_end_experiment_step()
            elif message == msg.START:
                self.start_experiment()
            elif message == msg.STOP:
                self.stop_experiment()
            elif message == msg.PARA_HW:
                self.para_heat_wait(temp, stir_speed, wait_duration)
            elif message == msg.PARA_S:
                self.para_sample(temp, stir_speed, dilution)
            elif message == msg.PARACETAMOL:
                self.paracetamol_synthesis()
            else:
                rospy.loginfo("invalid command")
            
            self._prev_msg = message
            
rospy.loginfo("working")