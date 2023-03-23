#
# car info from CarMaker ROS Node to external ROS Node
#

# General
# Header  header                                 # General ROS Header (optional)
# time    time                                   # ROS time when message was sent (optional)
# int64   cycleno                                # Cycle number since simulation start (optional)
# float32 synthdelay                             # Synthetic delay to check synchronization (optional)

# MSDStateReport
float32 acceleration_on_vehicle_wheel            # 提供controller所需的自⻋加速度信息
float32 steering_wheel_rad                       # 提供controller所需的自⻋方向盘转⻆       -> 需要进一步确认信号名字
float32 brake_pedal_input                        # 提供controller所需的自⻋刹⻋踏板开度信息
float32 throttle_pedal_input                     # 提供controller所需的自⻋油⻔踏板开度信息