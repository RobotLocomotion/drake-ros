#!/bin/bash

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Please provide path to model directory and model file name."
    echo "      Usage:"
    echo "                  bash model_transform.sh <model_directory_path> <model_file_name> "
    echo "                  or to tranform the universal_robot provide a valid empty path:"
    echo "                  bash model_transform.sh <model_directory_path>"
    return 1
fi

source setup.sh

bash format_model_and_generate_manifest.sh "$1" "$2"

bash compare_model_via_drake_and_ingition_images.sh "$1" "$2"
