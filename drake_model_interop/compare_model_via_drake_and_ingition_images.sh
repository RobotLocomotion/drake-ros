#!/bin/bash

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Please provide path to model directory and model file name."
    echo "      Usage:"
    echo "                  $bash compare_model_via_drake_and_ignition_images.sh <model_directory_path> <model_file_name> "
    return 1
fi

temp_directory=$(mktemp -d)
echo "Saving temporal test files to: ${temp_directory}"

./test_models.py "$1" "$2" "$temp_directory"
