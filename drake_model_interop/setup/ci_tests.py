import argparse
import subprocess
import urllib.request
import os
from zipfile import ZipFile

models_dir = ".tmp"
step_msg_color = "\033[95m"

# Default URDF Mdoels info
urdf_models_url = {
    "universal_robot": "https://github.com/ros-industrial/universal_robot"
}

# Default SDF Models Urls
sdf_models_url = {
    "nao": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/NAO%20with%20Ignition%20position%20controller/1/NAO%20with%20Ignition%20position%20controller.zip",
    "shadow_hand": "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/shadow_hand/2/shadow_hand.zip",
    "ur5_rg2": "https://fuel.gazebosim.org/1.0/AndrejOrsula/models/ur5_rg2/2/ur5_rg2.zip",
    "panda": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Panda%20with%20Ignition%20position%20controller%20model/4/Panda%20with%20Ignition%20position%20controller%20model.zip",
    "cerberus_anymal": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2/6/CERBERUS_ANYMAL_C_SENSOR_CONFIG_2.zip",
    "mpl_right_arm": "https://fuel.gazebosim.org/1.0/OpenRobotics/models/MPL%20right%20arm/2/MPL%20right%20arm.zip",
}


def download_transform_test_urdf(model_url):

    # File and directory locations
    model_name = urllib.parse.unquote(
        os.path.basename(urllib.parse.urlparse(model_url).path)
    )
    model_location = os.path.join(models_dir, model_name)
    # Run the test
    print(f"{step_msg_color}Dowloading model: {model_name}\033[0m")
    subprocess.run(
        f" git clone {model_url} {model_location} ", shell=True, check=True
    )
    subprocess.run(
        f"git -C {model_location} checkout melodic-devel-staging",
        shell=True,
        check=True,
    )
    print(
        f"{step_msg_color}Setting up, rendering and testing model:"
        f" {model_name}\033[0m"
    )
    subprocess.run(
        f"bash setup_render_test.sh -d {model_location}",
        shell=True,
        check=True,
    )


def download_transform_test_sdf(model_url):

    # File and directory locations
    file_name = urllib.parse.unquote(
        os.path.basename(urllib.parse.urlparse(model_url).path)
    )
    zipfile_location = os.path.join(models_dir, file_name)
    model_name = os.path.splitext(file_name)[0]
    unziped_location = os.path.join(models_dir, model_name).replace(" ", "_")

    # Run the test
    print(f"{step_msg_color}Dowloading model: {model_name}\033[0m")
    urllib.request.urlretrieve(model_url, zipfile_location)
    print(f"{step_msg_color}Extracting {zipfile_location}.\033[0m")
    with ZipFile(zipfile_location, "r") as zObject:
        zObject.extractall(unziped_location)
    print(
        f"{step_msg_color}Setting up, rendering and testing model:"
        f" {model_name}\033[0m"
    )
    subprocess.run(
        f"bash setup_render_test.sh -d {unziped_location} -m model.sdf -i 0.9",
        shell=True,
        check=True,
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--default_models", required=False, action="store_true")
    parser.add_argument("--sdf", required=False, nargs="*")
    parser.add_argument("--sdf_url", required=False, nargs="*")
    parser.add_argument("--urdf", required=False, nargs="*")
    parser.add_argument("--urdf_url", required=False, nargs="*")
    args = parser.parse_args()

    if args.default_models:
        print("List of ci default models:")
        print(f"\tSDF models:")
        for model_tag in sdf_models_url.keys():
            print(f"\t- {model_tag}")
        print(f"\t Urdf models:")
        for model_tag in urdf_models_url.keys():
            print(f"\t- {model_tag}")
        exit(0)

    # Cleanup old tests:
    subprocess.run(f"rm -rf  {models_dir}", shell=True, check=True)
    subprocess.run(f"mkdir  {models_dir}", shell=True, check=True)

    # Checking for default sdf models to test
    if args.sdf:
        for model_tag in args.sdf:
            if model_tag in sdf_models_url.keys():
                download_transform_test_sdf(sdf_models_url[model_tag])
            else:
                print(
                    f"Model [{model_tag}] not supported, try using --sdf_url"
                    " for custom models"
                )

    # Checking for default urdf models to test
    if args.urdf:
        for model_tag in args.urdf:
            if model_tag in urdf_models_url.keys():
                download_transform_test_urdf(urdf_models_url[model_tag])
            else:
                print(
                    f"Model [{model_tag}] not supported, try using --urdf_url"
                    " for custom models"
                )

    # Checking for custom sdf models to test
    if args.sdf_url:
        for model_url in args.sdf_url:
            download_transform_test_sdf(model_url)

    # If no model was selected just run them all
    if (
        not args.urdf
        and not args.sdf
        and not args.sdf_url
        and not args.urdf_url
    ):
        for model_tag in sdf_models_url.keys():
            download_transform_test_sdf(sdf_models_url[model_tag])
        for model_tag in urdf_models_url.keys():
            download_transform_test_urdf(urdf_models_url[model_tag])


if __name__ == "__main__":
    main()
    print()
    print("[ CI Tests Done ]")
