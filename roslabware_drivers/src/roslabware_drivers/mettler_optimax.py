
# external
from typing import Optional, Union
import time
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
        simulation: bool = False,
        experiment_name: str = None
    ):
        #
        # Create device object
        self.optimax = Optimax(experiment_name=experiment_name, device_name=device_name, connection_mode=connection_mode, address=address, port=port)
        
        # TODO after (IF) API implementation
        # if simulation == "True":
        #     self.optimax.simulation = True
        self.optimax.connect()

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
        self.start_timer(21600)

    def start_timer(self, seconds):
        start_time = time.time()
        end_time = start_time + seconds
        while time.time() < end_time:
            remaining_time = int(end_time - time.time())
            print(f"Time remaining for the completion: {round(remaining_time/60)} minutes", end="\r", flush=True)
            time.sleep(1)

    def stop_experiment(self):
        self.optimax.stop()
        rospy.loginfo("Experiment Stopped")     


    def paracetamol_synthesis(self): # temporary method for paracetamol synthesis
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
        if msg.temperature:
            temp = msg.temperature
        if msg.stir_speed:
            stir_speed = msg.stir_speed
        if msg.temp_duration:
            temp_duration = msg.temp_duration
        if msg.stir_duration:
            stir_duration = msg.stir_duration
        if msg.wait_duration:
            wait_duration = msg.wait_duration
        if msg.dilution:
            dilution = msg.dilution
        
        if not message == self._prev_message:
            if message == msg.ADD_TEMP:
                self.add_temp_step(temp, temp_duration)
                # self.start_experiment() #TODO remove this and find apt way to start the experiment
            elif message == msg.ADD_STIR:
                self.add_stir_step(stir_speed, stir_duration)
            elif message == msg.ADD_WAIT:
                self.add_wait_step(wait_duration)
            elif message == msg.ADD_SAMPLE:
                self.add_sampling_step(dilution)
                # self.start_experiment() #TODO remove this and find apt way to start the experiment
            elif message == msg.ADD_TEMP_STIR:
                self.add_stir_step(stir_speed, stir_duration)
                self.add_temp_step(temp, temp_duration)
                #  self.start_experiment() #TODO remove this and find apt way to start the experiment
            elif message == msg.START:
                self.start_experiment()
            elif message == msg.STOP:
                self.stop_experiment()
            elif message == msg.PARACETAMOL: # temporary message for paracetamol synthesis
                self.paracetamol_synthesis()
            else:
                rospy.loginfo("invalid command")
            
            self._prev_message = message
rospy.loginfo("working")
