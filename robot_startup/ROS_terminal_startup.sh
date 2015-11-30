#!/bin/bash
homeIP=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d'/')
#var="gdacer.local"
#var2="georgeMacBook.local"
#var=192.168.0.20
echo "IP address: $homeIP"
export ROS_HOSTNAME="georgeMacBook.local"
export ROS_MASTER_URI="http://gdacer.local:11311"
export ROS_IP=$homeIP
echo "ROS_HOSTNAME is now: $ROS_HOSTNAME"
echo "ROS_MASTER_URI is now: $ROS_MASTER_URI"
echo "ROS_IP is now: $ROS_IP"
