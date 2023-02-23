#!/bin/bash
set -eux

# Per discussion in https://github.com/RobotLocomotion/drake-ros/issues/221

# Might not have sudo in a container
if ! command -v sudo &> /dev/null
then
    apt-get update
    apt-get install -y sudo
fi

# Disable interactive prompts from blocking CI runs.
echo 'APT::Get::Assume-Yes "true";' | sudo tee /etc/apt/apt.conf.d/90yes
echo "debconf debconf/frontend select Noninteractive" | sudo debconf-set-selections

# Prevent upgrades against packages that may interfere with testing.
sudo apt-get update
sudo apt-mark hold grub-efi-amd64-signed
