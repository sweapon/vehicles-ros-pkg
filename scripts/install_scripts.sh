#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "Usage: install_scripts <vehicle-name>"
    exit 0
fi
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# Link starters
echo "sudo ln -s ${DIR}/vehicle-start /usr/local/sbin/vehicle-start"
sudo ln -s ${DIR}/vehicle-start /usr/local/sbin/vehicle-start
echo "sudo ln -s ${DIR}/vehicle-stop /usr/local/sbin/vehicle-stop"
sudo ln -s ${DIR}/vehicle-stop /usr/local/sbin/vehicle-stop
# Link service and configuration
echo "sudo mkdir -p /etc/init/ros"
sudo mkdir -p /etc/init/ros
echo "sudo cp ${DIR}/../$1/data/upstart/$1.conf /etc/init/ros/$1.conf"
sudo cp ${DIR}/../$1/data/upstart/$1.conf /etc/init/ros/$1.conf
echo "sudo ln -s ${DIR}/../$1/data/upstart /etc/ros/$1"
sudo ln -s ${DIR}/../$1/data/upstart /etc/ros/$1
echo "sudo mv -f /etc/hosts{,.old}"
sudo mv -f /etc/hosts{,.old}
echo "sudo ln -s ${DIR}/etc/hosts /etc/hosts"
sudo ln -s ${DIR}/etc/hosts /etc/hosts
