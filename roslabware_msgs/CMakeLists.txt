cmake_minimum_required(VERSION 3.0.2)
project(roslabware_msgs)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

add_message_files(
DIRECTORY
  msg/fisher_pp14102
)

add_message_files(
  DIRECTORY
   msg/ika_ret_control_visc
 )

add_message_files(

  DIRECTORY
msg/ika_rct_digital

 )

add_message_files(
  DIRECTORY
   msg/kern_pcb2500
 )

add_message_files(
  DIRECTORY
   msg/mettler_quantos_qb1
 )

add_message_files(
  DIRECTORY
   msg/tecan_xlp6000
 )

add_message_files(
  DIRECTORY
   msg/mettler_xpr226_drq
 )

add_message_files(
  DIRECTORY
   msg/mettler_optimax
 )

#[[
add_message_files(
  DIRECTORY
   msg/knf_simdos10_rcplus
 )
]]

 add_message_files(
  DIRECTORY
   msg/tecan_xcalibur
 )

generate_messages(
DEPENDENCIES
std_msgs
)

catkin_package(
	CATKIN_DEPENDS
  message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
