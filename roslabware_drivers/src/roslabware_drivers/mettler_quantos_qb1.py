# external
from typing import Optional, Union
from xml.dom import minidom  # noqa: DUO107

import rospy
from pylabware import QuantosQB1
from std_msgs.msg import String


# Core
from roslabware_msgs.msg import (
    MettlerQuantosQB1Cmd,
    MettlerQuantosQB1Reading
)


class QuantosQB1Ros:
    """
    ROS Wrapper for Serial Driver for Mettler Toledo Quantos
    QB1 dispensing system.
    """

    def __init__(
        self,
        device_name: str = None,
        connection_mode: str = "serial",
        address: Optional[str] = None,
        port: Union[str, int] = None,
        simulation: bool = False
    ):
       

        # Create device object
        self.quantos = QuantosQB1(
            device_name=device_name,
            connection_mode=connection_mode,
            address=address,
            port=port
        )

        if simulation == "True":
            self.quantos.simulation = True

    
        # Initialize ROS subscriber
        self.subs = rospy.Subscriber(
            name="mettler_quantos_qB1_commands",
            data_class=MettlerQuantosQB1Cmd,
            callback=self.callback_commands,
        )


        # Initialize ROS publisher for plataform info
        self.pub = rospy.Publisher(
            name="mettler_quantos_qB1_info",
            data_class=MettlerQuantosQB1Reading,
            queue_size=10,
        )


        rospy.loginfo("Mettler Quantos QB1 Driver Started")
    

    def tare(self):
        rospy.loginfo("Sending tare command")
        reply = self.quantos.tare()
        self.pub.publish( command_running = "Tare",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def get_stable_weight(self):
        rospy.loginfo("Sending get stable weight command")
        reply = self.quantos.get_stable_weight()
        
        rospy.loginfo(reply)

        self.pub.publish(command_running = "Get stable weight",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
  
        
    def get_door_position(self):
        rospy.loginfo("Sending get door position")
        reply = self.quantos.get_door_position()
        self.pub.publish(command_running = "Get door position",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        

    def get_head_data(self):
        rospy.loginfo("Sending get head data")
        reply = self.quantos.get_head_data()

        #output_list = self._flatten_dict(reply["outcomes"])
        self.pub.publish(command_running = "Get head data",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    
    def get_sample_data(self):
        rospy.loginfo("Sending get sample data")
        reply = self.quantos.get_sample_data()

        #output_list = self._flatten_dict(reply["outcomes"])
        self.pub.publish(command_running = "Get sample data",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)


    def set_tapping_before_dosing(self):
        rospy.loginfo("Setting tapping before dosing")
        reply = self.quantos.set_tapping_before_dosing()
        self.pub.publish(command_running = "Set tapping",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        
    def unset_tapping_before_dosing(self):
        rospy.loginfo("Unsetting tapping before dosing")
        reply = self.quantos.unset_tapping_before_dosing()
        self.pub.publish(command_running = "Unset tapping",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def set_tapper_intensity(self, intensity: int):
        rospy.loginfo("Setting intensity of before dosing tapping")
        reply = self.quantos.set_tapper_intensity(intensity)
        self.pub.publish(command_running = "Setting tapping intensity",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
    
    def set_tapper_duration(self, duration: int):
        rospy.loginfo("Setting duration of before dosing tapping")
        reply = self.quantos.set_tapper_duration(duration)
        self.pub.publish(command_running = "Setting tapping duration",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
    
    def lock_dosing_head_pin(self):
        rospy.loginfo("Sending lock dosing head pin")
        reply = self.quantos.lock_dosing_head_pin()
        self.pub.publish(command_running = "Locking dose head",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
    
    def unlock_dosing_head_pin(self):
        rospy.loginfo("Sending unlock dosing head pin")
        reply = self.quantos.unlock_dosing_head_pin()
        self.pub.publish(command_running = "Unlocking dose head",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def open_front_door(self):
        rospy.loginfo("Sending open front door")
        reply = self.quantos.open_front_door()
        self.pub.publish(command_running = "Opening front door",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def close_front_door(self):
        rospy.loginfo("Sending close front door")
        reply = self.quantos.close_front_door()
        self.pub.publish(command_running = "Closing front door",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        
    def open_side_doors(self):
        rospy.loginfo("Sending open side doors")
        reply = self.quantos.open_side_door()
        self.pub.publish(command_running = "Opening side doors",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def close_side_doors(self):
        rospy.loginfo("Sending close side doors")
        reply = self.quantos.close_side_door()
        self.pub.publish(command_running = "Closing side doors",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    
    def set_tolerance_standard(self):
        rospy.loginfo("Setting Tolerance mode to +/- standard")
        reply = self.quantos.set_tolerance_overdose()
        self.pub.publish(command_running = "Setting tolerance mode to +/- standard",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def set_tolerance_overdose(self):
        rospy.loginfo("Setting Tolerance mode to +/0 overdose")
        reply = self.quantos.set_tolerance_overdose()
        self.pub.publish(command_running = "Setting tolerance mode to +/0 standard",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def set_algorithm_standard(self):
        rospy.loginfo("Setting dosing algorithm to standard")
        reply = self.quantos.set_algorithm_standard()
        self.pub.publish(command_running = "Setting dosing algorithm to standard",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def set_algorithm_advanced(self):
        rospy.loginfo("Setting dosing algorithm to advanced")
        reply = self.quantos.set_algorithm_advanced()
        self.pub.publish(command_running = "Setting dosing algorithm to advanced",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)

    def _move_sampler(self, position:int) -> dict:
        rospy.loginfo(f"Moving sampler to postion {position}")
        reply = self.quantos.move_sampler(position)

        self.pub.publish(command_running = f"Moving sampler to postion {position}",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        return reply

    def get_sampler_position(self):
        rospy.loginfo("Getting current sampler position")
        reply = self.quantos.get_sampler_position()

        self.pub.publish(command_running = "Getting current sampler position",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
       
    def get_sampler_status(self):
       rospy.loginfo("Getting autosampler status")
       reply = self.quantos.get_sampler_position()
       self.pub.publish(command_running = "Getting autosampler status",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
       

    def set_value_pan(self):
        rospy.loginfo("Setting value pan")
        reply = self.quantos.set_value_pan()
        self.pub.publish(command_running = "Setting value pan",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
       

    def set_antistatic_on(self):
        rospy.loginfo("Setting antistatic on")
        reply = self.quantos.set_antistatic_on()
        self.pub.publish(command_running = "Setting antistatic on",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
       

    def set_antistatinc_off(self):
        rospy.loginfo("Sending set antistatic off")
        reply = self.quantos.set_antistatic_off()
        self.pub.publish(command_running = "Setting antistatic off",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        
    def _set_target_mass(self, mass: float) -> dict: 
        rospy.loginfo(f"Setting target mass to {mass} mg")
        reply = self.quantos.set_target_mass(mass)
        self.pub.publish(command_running = f"Setting target mass to {mass} mg",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        
        return reply
        

    def _set_tolerance(self, percentage: float) -> dict:
        rospy.loginfo(f"setting tolerance of {percentage} %")
        reply = self.quantos.set_tolerance_value(percentage)
        self.pub.publish(command_running = f"Setting tolerance of {percentage} %",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)
        return reply


    def _start_dosing(self): 
        rospy.loginfo("Starting dosing")
        reply = self.quantos.start_dosing()
        self.pub.publish(command_running = "Starting dosing",
                        success = reply["success"], output = reply["outcomes"], side_door_open = self.quantos.side_door_open,
                        front_door_open = self.quantos.front_door_open, sampler_pos = self.quantos.sampler_position)


    def _dispense_solid_at_position(self, percentage: float, amount: float):
        """
        Handles the dispensing process at a given position on the autosampler or on the basket if no autosampler.
        
        percentage: the tolerance for the dispence
        amount: the mass in mg to be dispensed. 
        
        """


        mass_reply = self._set_target_mass(amount)

        if mass_reply["success"]:
           
           tolerance_reply = self._set_tolerance(percentage)

           if tolerance_reply["success"]:
               self._start_dosing()                
        
           else: 
               rospy.loginfo("Set tolerance was unsuccesful")
                
        else:
           rospy.loginfo("Set target mass unsuccesful")

    def dispense(self, tolerance_percentage: float, amount: float, num_samples: int = 1 ):
        """
        Handles the dispensing process for a series of samples if the autosampler is mounted.
        If the autosample is not mounted it does a single dispense.

        tolerance_percentage: the tolerance for the dispense
        amount: the mass in mg to be dispensed
        num_samples: the number of samples to be dispensed into

        """

        if self.quantos.autosampler_mounted_flag:
            rospy.loginfo(f"Dispensing {amount} mg into {num_samples} samples")
            position_list = range(1,(num_samples+1), 1)

            for position in position_list:
                move_reply = self._move_sampler(position)
                
                if move_reply["success"]:
                    self._dispense_solid_at_position(tolerance_percentage, amount)
                    self.get_sample_data()

                else: 
                    rospy.loginfo("Sampler did not move!")
        
        elif self.quantos.autosampler_mounted_flag == False: #There is always the possibility that the autosampler flag may be None
            self._dispense_solid_at_position(tolerance_percentage, amount)
            self.get_sample_data()
     

    # Callback for subscriber.
    def callback_commands(self, msg):
        """Callback commands for susbcriber"""
        message = msg.quantos_command

        if message == msg.DISPENSE:
            self.dispense(msg.quantos_tolerance, msg.quantos_amount, msg.quantos_int)
        elif message == msg.GET_DOOR_POS:
            self.get_door_position()
        elif message == msg.GET_HEAD_DATA:
            self.get_head_data()
        elif message == msg.LOCK_PIN:
            self.lock_dosing_head_pin()
        elif message == msg.UNLOCK_PIN:
            self.unlock_dosing_head_pin()
        elif message == msg.OPEN_FRONT_DOOR:
            self.open_front_door()
        elif message == msg.CLOSE_FRONT_DOOR:
            self.close_front_door()
        elif message == msg.SET_TOLERANCE_STANDARD:
            self.set_tolerance_standard()
        elif message == msg.SET_TOLERANCE_OVERDOSE:
            self.set_tolerance_overdose()
        elif message == msg.SET_VALUE_PAN:
            self.set_value_pan()
        elif message == msg.SET_AS_ON:
            self.set_antistatic_on()
        elif message == msg.SET_AS_OFF:
            self.set_antistatinc_off()
        elif message == msg.TARE:
            self.tare()
        elif message == msg.GET_STABLE_WEIGHT:
            self.get_stable_weight()
        elif message == msg.GET_SAMPLE_DATA:
            self.get_sample_data()
        elif message == msg.SET_TAP_BEFORE:
            self.set_tapping_before_dosing()
        elif message == msg.UNSET_TAP_BEFORE:
            self.unset_tapping_before_dosing()
        elif message == msg.SET_TAP_INTENSITY:
            self.set_tapper_intensity(msg.quantos_int)
        elif message == msg.SET_TAP_DURATION:
            self.set_tapper_duration(msg.quantos_int)
        elif message == msg.SET_ALG_STANDARD:
            self.set_algorithm_standard()
        elif message == msg.SET_ALG_ADVANCED:
            self.set_algorithm_advanced()
        elif message == msg.MOVE_SAMPLER:
            self._move_sampler(msg.quantos_int)
        elif message == msg.GET_SAMPLER_POS:
            self.get_sampler_position()
        elif message == msg.GET_SAMPLER_STATUS:
            self.get_sampler_status()
        elif message == msg.OPEN_SIDE_DOORS:
            self.open_side_doors()
        elif message == msg.CLOSE_SIDE_DOORS:
            self.close_side_doors()

        else:
            rospy.loginfo("invalid command")

       
