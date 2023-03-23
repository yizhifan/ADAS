# !/bin/bash

echo "Run catkin build"

cmd="source /opt/ros/ros1/setup.bash"; echo $cmd; $cmd


cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='endpoint_msgs'";         echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='mla_localization_msgs'"; echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='planning_msgs'";         echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='framework_status_msgs'"; echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='system_manager_msgs'";   echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='lac_msgs'";              echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='cmrosutils'";            echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='hellocm'";               echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='hellocm_cmnode'";        echo $cmd; $cmd
cmd="catkin_make -DCATKIN_WHITELIST_PACKAGES='hellocm_msgs'";          echo $cmd; $cmd

cmd="catkin_make"; echo $cmd; $cmd
#cmd="catkin_make install"; echo $cmd; $cmd

# ToDo:
# - catkin may not manipulate exit status
#   - even if error occurs, script may continue without error...
#     -> e.g. when compiling msg with unknown datatype
if [ $? -ne 0 ]
    then exit $?
fi

echo "Info: Execute 'source ./devel/setup.bash' to prepare your environment!"
