# repro amdgpu issue

```sh
$ bazel build @ros2//:rviz2_rviz2
ERROR: {bazel_cache}/external/ros2/BUILD.bazel:551:11: @ros2//:rviz_ogre_vendor_cc: invalid label '/opt/amdgpu/lib/x86_64-linux-gnu/libGL.so.1' in element 24 of attribute 'srcs' in 'cc_library' rule: invalid target name '/opt/amdgpu/lib/x86_64-linux-gnu/libGL.so.1': target names may not start with '/'
ERROR: {bazel_cache}/external/ros2/BUILD.bazel:551:11: @ros2//:rviz_ogre_vendor_cc: invalid label '/opt/amdgpu/lib/x86_64-linux-gnu/libGL.so.1' in element 25 of attribute 'data' in 'cc_library' rule: invalid target name '/opt/amdgpu/lib/x86_64-linux-gnu/libGL.so.1': target names may not start with '/'
ERROR: error loading package '@ros2//': Package '' contains errors
...
```

## repro attempt

https://www.amd.com/en/support/kb/release-notes/rn-amdgpu-unified-linux-21-50-2 , then run `sudo amdgpu-install --opencl=rocr,legacy`

docker things

- https://github.com/RadeonOpenCompute/ROCm-docker - dunno if helpful, but something?


## commamnds

```sh
/path/to/drake-ros/repro/run_on_host.sh
```
