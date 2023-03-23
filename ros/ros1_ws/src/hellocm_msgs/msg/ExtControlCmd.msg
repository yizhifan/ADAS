#
# Message about control
#

# General
# Header  header                                 # General ROS Header (optional)
# time    time                                   # ROS time when message was sent (optional)
# int64   cycleno                                # Cycle number since simulation start (optional)

# Command
uint8   ControlCmdEnable                         # 是否启用以下控制命令，true->使能，false->不使
# float32 BrakePedalReqCmd                         # 刹车命令请求值，命令类型为制动压力
float64 SteeringWhlRadReqCmd                     # 转角命令请求值，命令类型为前轮转角
# float32 ThrottlePedalReqCmd                      # 油门命令请求值，命令类型为踏板开度
# uint8   GearReqCmd                               # 档位命令请求值