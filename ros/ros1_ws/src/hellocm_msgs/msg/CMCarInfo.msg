#
# car info from CarMaker ROS Node to external ROS Node
#

# General
# Header  header                                 # General ROS Header (optional)
# time    time                                   # ROS time when message was sent (optional)
# int64   cycleno                                # Cycle number since simulation start (optional)
# float32 synthdelay                             # Synthetic delay to check synchronization (optional)

# car info
uint8   CarInfoSts                               # 车辆信息输出状态，0->未输出车辆信息  1->输出车辆信息
CMMLALocalization MLAlocalization
CMMSDStateReport  MSDStateReport
CMChassisReport   ChassisReport