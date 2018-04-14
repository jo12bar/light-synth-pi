#!/usr/bin/env bash
#
# install-startup-script.bash
#
# Installs a systemd startup script for this program.

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
SERVICE_FILE_SOURCE="${DIR}/startup.service"
SERVICE_FILE_DESTINATION='/lib/systemd/system/light-synth-pi.service'

echo "Copying ${SERVICE_FILE_SOURCE} to ${SERVICE_FILE_DESTINATION}"
sudo cp "${SERVICE_FILE_SOURCE}" "${SERVICE_FILE_DESTINATION}"

echo 'Setting file permissions'
sudo chmod 644 "${SERVICE_FILE_DESTINATION}"

echo 'Configuring systemd'
sudo systemctl daemon-reload
sudo systemctl enable light-synth-pi.service

echo 'Done!'
