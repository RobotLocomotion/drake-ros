# -*- python -*-

load("@rules_shell//shell:sh_test.bzl", "sh_test")

def cmd_test(name, cmd, **kwargs):
    sh_test(
        name = name,
        srcs = ["//tools:cmd_exec.sh"],
        args = cmd,
        **kwargs
    )
