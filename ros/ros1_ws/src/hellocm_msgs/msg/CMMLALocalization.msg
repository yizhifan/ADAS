#
# car info from CarMaker ROS Node to external ROS Node
#

# General
# Header  header                                 # General ROS Header (optional)
# time    time                                   # ROS time when message was sent (optional)
# int64   cycleno                                # Cycle number since simulation start (optional)
# float32 synthdelay                             # Synthetic delay to check synchronization (optional)

# MLAlocalization
CMMLAPosition           position                 # 提供controller所需的自⻋位置信息
CMMLAOrientation        orientation              # 提供controller所需的自⻋姿态和朝向信息
CMMLAVelocity           velocity                 # 提供controller所需的自⻋⻋速信息
CMMLAAngularVelocity    angular_velocity
CMMLAAcceleration       acceleration