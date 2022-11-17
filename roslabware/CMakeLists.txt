cmake_minimum_required(VERSION 3.0.2)
project(roslabware)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   ika_rc.msg
   fisher_balance_pp14012_command.msg
   fisher_balance_pp14012_reading.msg
   ret_control_visc_hotplate_command.msg
   ret_control_visc_hotplate_reading.msg
   kern_command.msg
   kern_reading.msg
   mettler_quantos_qb1_command.msg
   mettler_quantos_qb1_reading.msg
   mettler_quantos_qb1_sample.msg
 )

 generate_messages(
  DEPENDENCIES
  std_msgs
 )


catkin_package(
	CATKIN_DEPENDS
  message_runtime)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
