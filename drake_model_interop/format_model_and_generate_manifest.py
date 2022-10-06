#!/usr/bin/env python3

"""
Example of doing cmdline script-y things in Python (rather than a bash script).

Derived from:
https://github.com/RobotLocomotion/drake/blob/7de24898/tmp/benchmark/generate_benchmark_from_master.py
"""

from contextlib import closing
import os
from os.path import abspath, dirname, isfile
from subprocess import PIPE, run
import sys
from textwrap import indent

from lxml import etree
import numpy as np
import pyassimp
import yaml
import argparse
import re
import fileinput
from PIL import Image

from process_util import CapturedProcess, bind_print_prefixed


class UserError(RuntimeError):
    pass


def eprint(s):
    print(s, file=sys.stderr)


def shell(cmd, check=True):
    """Executes a shell command."""
    eprint(f"+ {cmd}")
    return run(cmd, shell=True, check=check)


def subshell(cmd, check=True, stderr=None, strip=True):
    """Executes a subshell in a capture."""
    eprint(f"+ $({cmd})")
    result = run(cmd, shell=True, stdout=PIPE, stderr=stderr, encoding="utf8")
    if result.returncode != 0 and check:
        if stderr == PIPE:
            eprint(result.stderr)
        eprint(result.stdout)
        raise UserError(f"Exit code {result.returncode}: {cmd}")
    out = result.stdout
    if strip:
        out = out.strip()
    return out


def cd(p):
    eprint(f"+ cd {p}")
    os.chdir(p)


def parent_dir(p, *, count):
    for _ in range(count):
        p = dirname(p)
    return p


def load_mesh(mesh_file):
    assert isfile(mesh_file), mesh_file
    scene = pyassimp.load(mesh_file)
    assert scene is not None, mesh_file
    return scene


def get_transformed_vertices(node, v_list):
    for child in node.children:
        get_transformed_vertices(child, v_list)

        # add current node meshes to the list
        for mesh in child.meshes:
            for j in range(mesh.vertices.shape[0]):
                v_list.append(
                    child.transformation.dot(np.append(mesh.vertices[j], 1))[0:3]
                )

        # apply current transformation to vertices
        for i in range(len(v_list)):
            v_list[i] = node.transformation.dot(np.append(v_list[i], 1)).A[0, 0:3]


def get_mesh_extent(scene, mesh_file, filetype="obj"):
    # Return geometric center and size.
    v_list = []

    if filetype == ".dae":
        rotation = np.matrix([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
    else:
        rotation = np.matrix([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    scene.rootnode.transformation = rotation.dot(scene.rootnode.transformation)
    get_transformed_vertices(scene.rootnode, v_list)

    lb = np.min(v_list, axis=0)
    ub = np.max(v_list, axis=0)
    size = ub - lb
    center = (ub + lb) / 2
    return np.array([center, size])


def np_array_to_transform(np_array, transform):
    transform.a1 = np_array[0, 0]
    transform.a2 = np_array[0, 1]
    transform.a3 = np_array[0, 2]
    transform.a4 = np_array[0, 3]
    transform.b1 = np_array[1, 0]
    transform.b2 = np_array[1, 1]
    transform.b3 = np_array[1, 2]
    transform.b4 = np_array[1, 3]
    transform.c1 = np_array[2, 0]
    transform.c2 = np_array[2, 1]
    transform.c3 = np_array[2, 2]
    transform.c4 = np_array[2, 3]
    transform.d1 = np_array[3, 0]
    transform.d2 = np_array[3, 1]
    transform.d3 = np_array[3, 2]
    transform.d4 = np_array[3, 3]


# Aplies a rotation to the root node
def rotate_root_node(scene, rotation):
    transformed = rotation.dot(scene.rootnode.transformation)
    np_array_to_transform(transformed, scene.mRootNode.contents.mTransformation)


def convert_file_to_obj(mesh_file, suffix, scale=1):
    assert mesh_file.endswith(suffix), mesh_file
    obj_file = mesh_file[: -len(suffix)] + ".obj"
    print(f"Convert Mesh: {mesh_file} -> {obj_file}")
    if isfile(obj_file):
        return
    scene = load_mesh(mesh_file)

    # Workaround for issue https://github.com/assimp/assimp/issues/849.
    if suffix == ".dae":
        ROT_X_90 = np.matrix([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])
        rotate_root_node(scene, ROT_X_90)

    pyassimp.export(scene, obj_file, file_type="obj")

    # TODO(marcoag) skip sanity check for now
    # find a way to do one
    extent = get_mesh_extent(scene, mesh_file, suffix)
    scene_obj = load_mesh(obj_file)
    extent_obj = get_mesh_extent(scene_obj, mesh_file)
    np.testing.assert_allclose(
        extent,
        extent_obj,
        rtol=1e-05,
        atol=1e-07,
        err_msg=repr((mesh_file, obj_file)),
    )


def replace_text_to_obj(content, suffix):
    # Not robust, but meh.
    return content.replace(suffix, ".obj")


def find_mesh_files(d, suffix):
    files = subshell(f"find . -name '*{suffix}'").strip().split()
    files.sort()
    return files


# Workaround for https: // github.com/assimp/assimp/issues/3367
# remove this once all consumers have an assimp release
# incorporating the fix available on their distribution
def remove_empty_tags(dae_file, root):

    for element in root.xpath(".//*[not(node())]"):
        if len(element.attrib) == 0:
            element.getparent().remove(element)

    data = etree.tostring(root, pretty_print=True).decode("utf-8")
    text_file = open(dae_file, "w")
    text_file.write(data)
    text_file.close()


def obtain_scale(root):
    return float(root.xpath("//*[local-name() = 'unit']")[0].get("meter"))


# Some models contain materials that use gazebo specifics scripts
# remove them so they don't fail
def remove_gazebo_specific_scripts(description_file):
    root = etree.parse(description_file)

    for material_element in root.findall(".//material"):
        script_element = material_element.find("script")
        if script_element is not None:
            print(script_element.find("name").text)
            if script_element.find("name").text.startswith("Gazebo"):
                material_element.remove(script_element)

    data = etree.tostring(root, pretty_print=True).decode("utf-8")
    with open(description_file, "w") as text_file:
        text_file.write(data)


# Some models use pacakge based uri but don't contain a
# pacakge.xml so we create one to make sure they can be
# resolved
def create_pacakge_xml(description_file):
    if not os.path.isfile("package.xml"):
        root = etree.parse(description_file)
        model_element = root.find(".//model")
        package_name = model_element.attrib["name"]
        with open("package.xml", "w") as f:
            f.write(
                """<package format="2">\n <name>{package_name}</name>\n</package>"""
            )


FLAVORS = [
    "ur3",
    "ur3e",
    "ur5",
    "ur5e",
]


def preprocess_sdf_and_materials(model_directory, description_file):
    description_file_path = os.path.join(model_directory, description_file)
    for line in fileinput.input(description_file_path, inplace=True):
        # Some sdfs have a comment before the xml tag
        # this makes the parser fail, since the tag is optional
        # we'll remove it as safety workaround
        if not re.search(r"^\<\?xml.*", line):
            # Change the reference of the mesh file
            line = re.sub(r"\.stl|\.dae", ".obj", line)
            print(line, end="")

    for root, subdirs, files in os.walk(model_directory):
        for filename in files:
            # Convert jpg/jpeg files to png
            if filename.endswith(".jpg") or filename.endswith(".jpeg"):
                im = Image.open(os.path.join(root, filename))
                filename = re.sub(r"\.jpg|\.jpeg", ".png", filename)
                rgb_im = im.convert("RGB")
                rgb_im.save(os.path.join(root, filename))
                print("Convert jpg/jpeg: ", os.path.join(root, filename))
            # Change jpg/jpeg renferences on mtl files to png
            if filename.endswith(".mtl"):
                for line in fileinput.input(os.path.join(root, filename), inplace=True):
                    line = re.sub(r"\.jpg|\.jpeg", ".png", line)
                    print(line, end="")


def check_completion_token_exists(completion_file, completion_token):
    if os.path.exists(completion_file):
        with open(completion_file, "r") as f:
            completion_file_data = f.read()
        if completion_file_data == completion_token:
            return True
        else:
            return False
    else:
        return False


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--model_directory",
        required=True,
        help="Directory location of the model files",
    )
    parser.add_argument(
        "-m", "--model_description_file", help="Model description file name", default=""
    )
    args, unknown = parser.parse_known_args()

    # Check for completion file
    completion_token = "2021-03-12.1"
    completion_file = os.path.join(args.model_directory, ".completion-token")
    if check_completion_token_exists(completion_file, completion_token):
        print("Skipping conversion. It has been done already.")
        return

    if args.model_description_file.endswith(".sdf"):
        preprocess_sdf_and_materials(args.model_directory, args.model_description_file)

    source_tree = parent_dir(abspath(__file__), count=1)
    cd(source_tree)

    cd(args.model_directory)
    print(f"[ Convert Meshes for Drake :( ]")
    for dae_file in find_mesh_files(".", ".dae"):
        root = etree.parse(dae_file)
        remove_empty_tags(dae_file, root)
        scale = obtain_scale(root)
        convert_file_to_obj(dae_file, ".dae", scale)
    for stl_file in find_mesh_files(".", ".stl"):
        convert_file_to_obj(stl_file, ".stl")

    if args.model_description_file.endswith(".sdf"):
        print(
            "Found SDF as description file, making arrangements to ensure compatibility"
        )
        remove_gazebo_specific_scripts(args.model_description_file)
        create_pacakge_xml(args.model_description_file)
    else:
        print("Found URDF as description file, translating through ros launch")
        cd(source_tree)
        cd("repos/universal_robot")
        if "ROS_DISTRO" not in os.environ:
            raise UserError("Please run under `./ros_setup.bash`, or whatevs")

        # Use URI that is unlikely to be used.
        os.environ["ROS_MASTER_URI"] = "http://localhost:11321"
        os.environ[
            "ROS_PACKAGE_PATH"
        ] = f"{os.getcwd()}:{os.environ['ROS_PACKAGE_PATH']}"

        cd("ur_description")

        print()

        urdf_files = []
        # Start a roscore.
        roscore = CapturedProcess(
            ["roscore", "-p", "11321"],
            on_new_text=bind_print_prefixed("[roscore] "),
        )
        with closing(roscore):
            while "started core service" not in roscore.output.get_text():
                assert roscore.poll() is None

            for flavor in FLAVORS:
                shell(f"roslaunch ur_description load_{flavor}.launch")
                urdf_file = f"urdf/{flavor}.urdf"
                output = subshell(f"rosparam get /robot_description")
                content = yaml.load(output)
                content = replace_text_to_obj(content, ".stl")
                content = replace_text_to_obj(content, ".dae")
                with open(urdf_file, "w") as f:
                    f.write(content)
                urdf_files.append(urdf_file)

            print("\n\n")
            print("Generated URDF files:")
            print(indent("\n".join(urdf_files), "  "))

    # Save completion token
    cd(source_tree)
    with open(completion_file, "w") as f:
        f.write(completion_token)


if __name__ == "__main__":
    try:
        main()
        print()
        print("[ Done ]")
    except UserError as e:
        eprint(e)
        sys.exit(1)
