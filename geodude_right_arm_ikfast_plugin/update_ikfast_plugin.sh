search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=geodude.srdf
robot_name_in_srdf=geodude
moveit_config_pkg=geodude_moveit_config
robot_name=geodude
planning_group_name=right_arm
ikfast_plugin_pkg=geodude_right_arm_ikfast_plugin
base_link_name=right_wam_base
eef_link_name=right_wam7
ikfast_output_path=/home/atharva620/geodude_right_arm_ikfast_plugin/src/geodude_right_arm_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
