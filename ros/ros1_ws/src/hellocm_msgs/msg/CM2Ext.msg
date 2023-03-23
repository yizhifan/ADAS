#
# Message from CarMaker ROS Node to external ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)
float32 synthdelay                             # Synthetic delay to check synchronization (optional)

# Test
float32 CarPosX_m
float32 CarPosY_m
float32 CarPosA_rad
float32 CarYawRate_rads
float32 CarWhlAng_rad
float32 CarWhlAngRate_rads
float32 CarSpd_mps

# CarInfo
CMCarInfo                             CarInfo

# CarInfo for topic
mla_localization_msgs/MLALocalization    egopose
planning_msgs/Planning                   plan
endpoint_msgs/ChassisReport              chassis_report
endpoint_msgs/WheelReport                wheel_report
#planning_msgs/MpcTrajectoryResult        mpc_traj
framework_status_msgs/ModuleControl      module_control
system_manager_msgs/SysControllerRequest sys_controller_request