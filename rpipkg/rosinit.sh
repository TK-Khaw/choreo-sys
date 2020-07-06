#!/usr/bin/env bash

#Calling the setup.bash to have ROS recognizes necessary file.
source sbotcws/devel/setup.bash

#ROS master URI environemental variable. 
export ROS_MASTER_URI=http://192.168.137.1:11311
#Self IP since not using a DHCP server. 
export ROS_HOSTNAME="192.168.137.170"
exec "$@"
