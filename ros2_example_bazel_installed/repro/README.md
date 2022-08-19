# RViz - Weird Workflow Issues to Visualize Pose?

There are at least three ways to visualize a pose in RViz:

1. `Pose` display, using `Shape: Axes` + `Topic`
2. `MarkerArray` display, using `Topic`
3. `Axes` display, using `Reference Frame`

It is good to have all options. However, some are more awkward (or don't even
seem to work).

To reproduce:

```sh
cd drake-ros/ros2_example_bazel_installed
bazel build //repro/... //tools/...

# Terminal 1: RViz
bazel-bin/tools/rviz2 -d ./repro/rviz_pub.rviz

# Terminal 2: Script
bazel-bin/repro/rviz_pub
```

Current results:

1. `Pose` works fine, regardless of timestamp
2. `MarkerArray` seems to not show all markers? (bug in script?)
3. `Axes` has non-obvious TF2 errors. <br/>
   From looking at docs:
   <https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Learning-About-Tf2-And-Time-Cpp.html>
   Doing something equiv to `tf2::TimePointZero` on publishing side only has
   negative effects.
