#!/bin/bash

_cur_dir=$(cd $(dirname ${BASH_SOURCE}) && pwd)

_provision_repos() { (
    set -eu
    cd ${_cur_dir}
    repo_dir=${PWD}/repos
    completion_token=2021-03-12.1
    completion_file=$1/.completion-token

    if [[ "$2" == *\.sdf ]]
    then
        ./render_ur_urdfs.py "$1" "$2"
    else
        if [[ -f ${completion_file} && "$(cat ${completion_file})" == "${completion_token}" ]]; then
        return 0
        fi
        set -x
        rm -rf ${repo_dir}

        mkdir ${repo_dir} && cd ${repo_dir}

        git clone https://github.com/ros-industrial/universal_robot
        cd universal_robot/
        git checkout e8234318cc94  # From melodic-devel-staging
        # Er... dunno what to do about this, so hackzzz
        cd ${_cur_dir}
        ./ros_setup.bash ./render_ur_urdfs.py "$1" "$2"
    fi

    echo "${completion_token}" > ${completion_file}
) }

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Please provide path to model directory and model file name."
    echo "      Usage:"
    echo "                  $bash format_model_and_generate_manifest.sh <model_directory_path> <model_file_name> "
    return 1
fi

_provision_repos "$1" "$2"
