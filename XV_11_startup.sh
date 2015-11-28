#!/bin/bash
#var=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1 -d'/')
var="gdacer.local"
echo "IP address: $var"
export ROS_MASTER_URI=http://$var:11311
export ROS_HOSTNAME=$var
export ROS_IP=$var
echo "ROS_MASTER_URI is now: $ROS_MASTER_URI"
echo "ROS_IP is now: $ROS_IP"
#echo -n "Enter serial port name, e.g. ACM0: "
#read port
echo "XV_11 laser started on port /dev/arduino_XV11"
rosrun xv_11_laser_driver neato_laser_publisher _port:=/dev/arduino_XV11 _firmware_version:=2
