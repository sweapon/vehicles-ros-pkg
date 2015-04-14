#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "Usage: update_scripts <vehicle-name>"
    exit 0
fi
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# Clean 
${DIR}/uninstall_scripts $1
# Re-install
${DIR}/install_scripts $1
