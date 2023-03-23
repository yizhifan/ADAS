#
# Message from external ROS Node to CarMaker ROS Node
#

# General
Header  header                                 # General ROS Header (optional)
time    time                                   # ROS time when message was sent (optional)
int64   cycleno                                # Cycle number since simulation start (optional)

# Test
float32 TargetSpd_mps

# Control module
ExtControlCmd ControlCmd

# Control command from topic
lac_msgs/LacControlCmd control_command 