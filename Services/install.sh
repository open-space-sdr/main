#!/bin/bash

# Exit immediately if any command fails
set -e

# Enforce root privileges
if [ "$EUID" -ne 0 ]; then
  echo "Error: Please run this install script with sudo."
  exit 1
fi

echo "Installing RF Quad Kit services..."

echo "Copying service files to systemd directory..."
cp load-quadrf.service /etc/systemd/system/load-quadrf.service
cp quadrf-gui.service /etc/systemd/system/quadrf-gui.service
cp soapysdr-server.service /etc/systemd/system/soapysdr-server.service
cp soapysdr-server.service /etc/systemd/system/kasmvnc.service

# Ensure systemd doesn't complain about overly open file permissions
chmod 644 /etc/systemd/system/load-quadrf.service
chmod 644 /etc/systemd/system/quadrf-gui.service
chmod 644 /etc/systemd/system/soapysdr-server.service
chmod 644 /etc/systemd/system/kasmvnc.service


echo "Reloading systemd daemon..."
systemctl daemon-reload

echo "Enabling and starting hardware initialization..."
systemctl enable load-quadrf.service
systemctl start load-quadrf.service

echo "Enabling and starting the web GUI..."
systemctl enable quadrf-gui.service
systemctl start quadrf-gui.service

echo "Starting SoapySDR..."
systemctl enable soapysdr-server.service
systemctl start soapysdr-server.service

echo "Starting KasmVNC..."
systemctl enable kasmvnc.service
systemctl start kasmvnc.service


echo "Installation complete!"
