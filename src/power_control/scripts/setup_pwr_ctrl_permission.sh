#!/bin/bash

# This script requires super user permission to be executed

REQ_USER=${1-"$USER"} # Taking in the first argument

SUDOERS_FILE_PATH=/etc/sudoers
SUDOERS_CONFIG_LINE="$REQ_USER ALL=(ALL) NOPASSWD: /sbin/poweroff, /sbin/reboot, /sbin/shutdown"

echo "Setting up passwordless sytem reboot, poweroff and shutdown"

if sudo grep -qxF "$SUDOERS_CONFIG_LINE" $SUDOERS_FILE_PATH; then
    echo "$SUDOERS_FILE_PATH is configured properly for passwordless power control for user $REQ_USER"
    echo "Nothing to be done"
else
    
    echo "Configuring $SUDOERS_FILE_PATH for passwordless power control for user $REQ_USER"
    
    echo "" | sudo EDITOR='tee -a' visudo
    echo "# Passwordless power control for user $REQ_USER" | sudo EDITOR='tee -a' visudo
    echo $SUDOERS_CONFIG_LINE | sudo EDITOR='tee -a' visudo

    echo "Done"
fi
