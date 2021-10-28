#!/bin/sh

talker=$1
listener=$2
$talker --ros-args --disable-external-lib-logs -- & $listener --ros-args --disable-external-lib-logs -p timeout:=2.0
rc=$?; kill $!
exit $rc
