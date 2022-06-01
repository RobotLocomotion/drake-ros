#!/bin/bash

set -euo pipefail

if [[ $# -lt 1 || $# -gt 6 ]]; then
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

echo

source setup.sh

bash format_model_and_generate_manifest.sh "$1" "$2"

bash compare_model_via_drake_and_ingition_images.sh "$1" "$2" "${@:3}"
