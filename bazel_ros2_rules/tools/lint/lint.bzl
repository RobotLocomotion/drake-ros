load("@drake//tools/lint:lint.bzl", _drake_add_lint_tests = "add_lint_tests")

def add_lint_tests(cpplint_data = None, **kwargs):
    if not cpplint_data:
        cpplint_data = []
    cpplint_data.append("//:.clang-format")
    _drake_add_lint_tests(cpplint_data=cpplint_data, **kwargs)
