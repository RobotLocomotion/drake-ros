#!/bin/bash
set -eu -o pipefail

# Simple script to leverage Drake's custom Bazel linting rules among different
# repositories.
# The better solution is to instrument each repository with its own linting.
# However, to keep `bazel_ros2_rules` decoupled from Drake, we do this
# externally.
#
# Fixing mode:
#    ./fix_bazel_lint.sh
# Testing mode (for CI):
#    ./fix_bazel_lint.sh --test

# TODO(eric.cousineau): Consider just making this a pre-commit mechanism, and
# pull in buildifer / bzlcodestyle in a more compact way?

test=
while [[ $# -gt 0 ]]; do
    case "$1" in
        --test)
            test=1
            shift;;
        *)
            echo "Invalid flags: $@" >&2
            exit 1
            break;;
    esac
done

script_dir=$(cd $(dirname ${BASH_SOURCE}) && pwd)
cd ${script_dir}

# Build tooling.
( cd .lint/ && bazel build @drake//tools/lint:buildifier @drake//tools/lint:bzlcodestyle)
buildifer=.lint/bazel-bin/external/drake/tools/lint/buildifier
# N.B. --ignore options taken as of drake v1.10.0.
bzlcodestyle=".lint/bazel-bin/external/drake/tools/lint/bzlcodestyle --ignore=E265,E302,E305,W504"

list-bazel-files() {
    find ${1} -type f -name 'WORKSPACE' -o -name '*.bazel' -o -name '*.bzl'
}
bazel_files=$(list-bazel-files .)

buildifier-scrub() {
    # Remove fix messages.
    # TODO(eric.cousineau): Plumb as command, or just use pre-commit.
    sed -E \
        -e 's#^.*fix via.*$##g' \
        -e 's#^.*if that program.*$##g'
}

if [[ -n "${test}" ]]; then
    # Test-only mode (for CI).
    echo 'Using `./fix_bazel_lint.sh --test`.' >&2

    bad=
    if ! ${buildifer} -mode=check ${bazel_files} 2>&1 | buildifier-scrub ; then
        bad=1
    fi
    if ! ${bzlcodestyle} ${bazel_files}; then
        bad=1
    fi
    if [[ -n ${bad} ]]; then
        echo
        echo 'To fix, run `./fix_bazel_lint.sh`' >&2
        echo 'Some fixes may require manual edits' >&2
        echo
        exit 1
    else
        echo "Success!"
    fi
else
    # Fixing mode.
    ${buildifer} ${bazel_files}
    # Run styling that may need manual fixes.
    ${bzlcodestyle} ${bazel_files}
fi
