# See `runfiles_direct_test.sh` for motivation.
from bazel_tools.tools.python.runfiles import runfiles


def main():
    r = runfiles.Create()
    file_path = r.Rlocation(
        "ros2_example_bazel_installed/test/runfiles_test_data.txt")
    print(file_path)
    print("Good!")


assert __name__ == "__main__"
main()
