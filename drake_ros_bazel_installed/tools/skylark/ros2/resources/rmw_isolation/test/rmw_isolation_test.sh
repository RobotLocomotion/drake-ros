#!/bin/bash

TALKER_EXECUTABLE=$1
LISTENER_EXECUTABLE=$2
TIMEOUT=2.0  # in seconds

# Avoid issues when trying to expand '~'
DEFAULT_ROS_ARGS="--disable-external-lib-logs"

function test_isolation
{
    rc=0

    # Start talker in the background
    TALKER_ROS_ARGS=$DEFAULT_ROS_ARGS
    $TALKER_EXECUTABLE --ros-args $TALKER_ROS_ARGS -- &
    TALKER_PID=$!

    # Run listener in the foreground
    LISTENER_ROS_ARGS="$DEFAULT_ROS_ARGS --param timeout:=$TIMEOUT"
    if ! $LISTENER_EXECUTABLE --ros-args $LISTENER_ROS_ARGS; then
        # Listener exited with an error
        rc=$?
    fi

    # Kill talker in the background
    if ! kill $TALKER_PID; then
        # Talker exited too early, likely with an error
        wait $TALKER_PID
        rc=$?
    fi

    return $rc
}

# Run as many concurrent tests as shards are available
[ -z "${TEST_SHARD_STATUS_FILE-}" ] || touch "$TEST_SHARD_STATUS_FILE"

test_isolation
