#
# car info from CarMaker ROS Node to external ROS Node
#

# General
# Header  header                                 # General ROS Header (optional)
# time    time                                   # ROS time when message was sent (optional)
# int64   cycleno                                # Cycle number since simulation start (optional)
# float32 synthdelay                             # Synthetic delay to check synchronization (optional)

# MLAVelocity
geometry_msgs/Vector3 velocity_local             # 提供controller所需的自⻋⻋速信息                                