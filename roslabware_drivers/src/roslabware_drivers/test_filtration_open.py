# # would have to specify path to ot2_driver_ssh.py if not in directory
# # from ot2_driver.ot2_driver_http import OT2_Config,OT2_Driver
# import requests
# import time

# # ot2 =OT2_Driver(OT2_Config(ip="169.254.227.210", port=31950))

# # Test connection
# base_url = f"http://169.254.227.210:31950"
# headers = {"Opentrons-Version": "2"}
# test_conn_url = f"{base_url}/robot/lights"

# resp = requests.get(test_conn_url, headers=headers)
# if resp.status_code != 200:
#     raise RuntimeError(f"Could not connect to opentrons with config")

# if "on" in resp.json() and not resp.json()["on"]:
#     change_lights_url = f"{base_url}/robot/lights"
#     payload = {"on": True}

#     requests.post(change_lights_url, headers=headers, json=payload)
# else:
#     change_lights_url = f"{base_url}/robot/lights"
#     payload = {"on": False}
#     requests.post(change_lights_url, headers=headers, json=payload)
#     time.sleep(1)  # Can mix later
#     change_lights_url = f"{base_url}/robot/lights"
#     payload = {"on": True}

#     requests.post(change_lights_url, headers=headers, json=payload)




########################################################################

#####ROS Message Test: OT2 ########################


import rospy

# Core
from roslabware_msgs.msg import (
    Ot2Cmd,
    Ot2Status,
    FiltrationCmd,
    FiltrationStatus
)


# Initialize the ROS node
rospy.init_node('test_publisher', anonymous=True)

# Create a publisher object
# pub = rospy.Publisher('Ot2_command', Ot2Cmd, queue_size=10)



# Wait for a short time to ensure the publisher is registered
rospy.sleep(1)


########   OT2 Messages     ###############
# message = Ot2Cmd()
# # message.ot2_command = Ot2Cmd.LIGHT_ON
# # message.ot2_command = Ot2Cmd.LIGHT_OFF
# message.ot2_command = Ot2Cmd.RUN_PROTOCOL
# message.protocol_id = 1


####
pub = rospy.Publisher('/filtration_command', FiltrationCmd, queue_size=10)

rospy.sleep(1)

########      Filtration Messages   ###############
message = FiltrationCmd()
# message.ot2_command = Ot2Cmd.LIGHT_ON
# message.ot2_command = Ot2Cmd.LIGHT_OFF
message.filtration_command = FiltrationCmd.OPEN_VALVE
message.seq = 21


# Log and publish the message
rospy.loginfo(f"Publishing: {message}")
pub.publish(message)





########################################################################

#####ROS Message Test: Filtration ########################

