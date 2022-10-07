# Drake Model Interop

## About

This tool provides an automated workflow to import and test articulated
robots (URDF / SDFormat) from other ecosystems into Drake.

It performs the necessary file conversions and model description changes
to allow their use with Drake. It also performs an
[Intersection over Union](https://en.wikipedia.org/wiki/Jaccard_index)
image based test comparing the resulting model visuals and collisions
from Drake with the ones from Gazebo.

It currently supports models in SDFormat and the
[universal robot](https://github.com/ros-industrial/universal_robot)
Urdf model. You can use any SDF models from the
[Gazebo models repository](https://app.gazebosim.org/dashboard).

## Instructions

First, install the required dependencies:

```sh
sudo ./setup/install_prereqs.sh
```

### SDF models

Download your model to your preferred location.
For `SDF` based models the following can be used:

```
$ bash setup_render_test.sh -d <unziped_model_location> -m <model.sdf> -i <0.9>
```

Where the last parameter `-i` indicates the required level of
[Intersection over Union](https://en.wikipedia.org/wiki/Jaccard_index)
image similarity required during the testing process.

### Urdf models (Universal Robot)

For `urdf` models, currently, only the
[universal robot](https://github.com/ros-industrial/universal_robot)
model is supported. Since repositories don't have a general
structure most of the process is tailored towards this specific
repository. To run the conversion of the universal robot simply
clone the repository to your prefered location:

```
git clone https://github.com/ros-industrial/universal_robot
```

Checkout the `melodic-devel-staging` branch:

```
git -C <cloned_repo_location> checkout melodic-devel-staging
```

Run the conversion and test:

```
bash setup_render_test.sh -d <cloned_repo_location>
```

## Test

You can test models using the ci script with the predefined models.
This will download, convert and test the specified models.
There is a set of predefined models or you can use your own urls for sdf
based models.
You can see the predefined name tags with the following command:

```
s:~/drake-ros/drake_model_interop$ python3 setup/ci_tests.py --default_models
List of ci default models:
        SDF models:
        - nao
        - shadow_hand
        - ur5_rg2
        - panda
        - cerberus_anymal
        - mpl_right_arm
         Urdf models:
        - universal_robot
```

You can test as many of the sdf predefined models as you want with:

```
$ python3 setup/ci_tests.py --sdf nao shadow_hand ...
```

For the urdf model you can use:

```
$ python3 setup/ci_tests.py --urdf universal_robot
```

Custom models can be tested using their download link like this:

```
$ python3 setup/ci_tests.py --sdf_url https://fuel.gazebosim.org/1.0/OpenRobotics/models/NAO%20with%20Ignition%20position%20controller/1/NAO%20with%20Ignition%20position%20controller.zip
```

Urdf custom models are still not supported.
