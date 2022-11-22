cmake_minimum_required(VERSION 3.0.2)
project(roslabware)


find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

 add_message_files(
   FILES
   fisher_balance_pp14102/fisher_balance_pp14102_command.msg
   fisher_balance_pp14102/fisher_balance_pp14102_reading.msg
   ika_ret_control_visc/ika_ret_control_visc_command.msg
   ika_ret_control_visc/ika_ret_control_visc_reading.msg
   kern_pcb_top_balance/kern_pcb_top_balance_command.msg
   kern_pcb_top_balance/kern_pcb_top_balance_reading.msg
   mettler_quantos_qb1/mettler_quantos_qb1_command.msg
   mettler_quantos_qb1/mettler_quantos_qb1_reading.msg
   mettler_quantos_qb1/mettler_quantos_qb1_sample.msg
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
