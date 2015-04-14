#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "Usage: uninstall_scripts <vehicle-name>"
    exit 0
fi
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# Link starters
echo "sudo rm -f /usr/local/sbin/vehicle-start"
sudo rm -f /usr/local/sbin/vehicle-start
echo "sudo rm -f /usr/local/sbin/vehicle-stop"
sudo rm -f /usr/local/sbin/vehicle-stop
# Link service and configuration
echo "sudo rm -f /etc/init/ros/$1.conf"
sudo rm -f /etc/init/ros/$1.conf
echo "sudo rm -f /etc/ros/$1"
sudo rm -f /etc/ros/$1
echo "sudo mv -f /etc/hosts{.old,}"
sudo mv -f /etc/hosts{.old,}

