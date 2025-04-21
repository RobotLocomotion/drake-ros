#!/bin/bash

# This script updates rosidl packages generated with changes that are still not available upstream
# but are necessary for building ros2 jazzy. The debs were generated from https://github.com/ros2/rosidl/pull/857.
set -e

DEBS_DIR=rosidl_debs
REPO=frneer/rosidl
RELEASE_TAG=rosidl_cli_type_description_support

# Download the debs from the release
mkdir -p $DEBS_DIR
gh release download $RELEASE_TAG --repo $REPO --pattern "*.deb" --dir $DEBS_DIR

# Required packages in topological order
PACKAGES=(
    rosidl_cli
    rosidl_generator_c
    rosidl_generator_cpp
    rosidl_generator_type_description
    rosidl_typesupport_introspection_c
    rosidl_typesupport_introspection_cpp
)

# Install packages
for pkg in "${PACKAGES[@]}"; do
    sudo apt install -y ./$DEBS_DIR/ros-jazzy-${pkg//_/-}*.deb
done
