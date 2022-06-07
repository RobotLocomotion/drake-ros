#!/bin/bash

set -euo pipefail

if [[ $# -lt 2 || $# -gt 6 ]]; then
    echo "Please provide path to model directory and model file name."
    echo "      Usage:"
    echo "                  To transform an sdf model provide directory and model file name:"
    echo "                  bash model_transform.sh <model_directory_path> <model_file_name>"
    echo ""
    echo "                  To tranform and test the universal_robot provide a valid empty path:"
    echo "                  bash model_transform.sh <model_directory_path>"
    echo ""
    echo "                  Optional arguments:"
    echo "                  -i|--iou_threshold <value>"
    echo "                      Sets <value> as threshold for the intersection over union test. <value> must be a float between 0 and 1"
    echo "                  -d|--drake_visualizer"
    echo "                      Enables DrakeVisualizer"
    exit 1
fi

source setup.sh

if [[ $# -gt 2 ]]; then
    ./format_model_and_generate_manifest.py $@

    temp_directory=$(mktemp -d)
    echo "Saving temporal test files to: ${temp_directory}"
    ./compare_model_via_drake_and_ingition_images.py $@ --temp_directory "$temp_directory"
else
    set -x
    ./ros_setup.bash ./format_model_and_generate_manifest.py $@
fi
