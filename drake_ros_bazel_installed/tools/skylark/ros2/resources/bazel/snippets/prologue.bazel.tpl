# -*- python -*-

package(default_visibility = ["//visibility:public"])

load("@REPOSITORY_ROOT@:ros_py.bzl", "ros_py_import")
load("@drake_ros//tools/skylark/ros2:common.bzl", "interfaces_filegroup")
load("@drake_ros//tools/skylark/ros2:common.bzl", "share_filegroup")
