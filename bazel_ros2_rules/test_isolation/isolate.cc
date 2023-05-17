#include <unistd.h>

#include <iostream>
#include <vector>

#include "create_linux_namespaces.h"

void die(const char * message) {
    std::cerr << "isolate: " << message << ".\n";
    exit(-1);
}

int main(int argc, char ** argv) {
    if (argc < 2) {
        die("shim must be given a command to execute");
    }

    if (not bazel_ros2_rules::create_linux_namespaces()) {
        die("Failed to fully create isolated environment");
    }

    // Have to copy new a new array that terminates with a null pointer.
    std::vector<char *> new_argv;
    for (int i = 1; i < argc; ++i) {
        new_argv.push_back(argv[i]);
    }
    new_argv.push_back(nullptr);

    // Exec a new process - should never return!
    execv(new_argv.at(0), &new_argv.at(0));

    perror("execv");
    die("Call to execv failed");
    return -1;
}
