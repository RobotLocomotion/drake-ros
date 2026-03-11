# Drake ROS: Contributing

## Code Style

All code in this repository should generally comply with Drake's [code style
guide](https://drake.mit.edu/code_style_guide.html).

### Pre-commit Hooks

This repository uses [pre-commit](https://pre-commit.com/) to automatically
check and format code before commits. See [`.pre-commit-config.yaml`](./.pre-commit-config.yaml)
for the complete list of configured hooks.

#### Installation

Install pre-commit (if not already installed):

```bash
pip install pre-commit
```

Install the git hooks in your local repository if you want them to run automatically on `git commit`.

```bash
pre-commit install
```

#### Usage
In case you don't want to install the commit hooks, you can trigger them manually:

```bash
pre-commit run --all-files
```
```

## Building Locally

For an example of using `colcon`, please see `../drake_ros_examples`.

To build and run all tests, use `./tools/run_all_tests.sh`.

## Continuous Integration

This repository uses GitHub Actions to perform CI.

If you are prototyping a PR, consider saving resources by disabling CI by
adding the `status: defer ci` label to your PR.

If you add or remove the label to an existing PR, you must ensure that checks
are re-run with job conditions being re-evaluated. You can do this by pushing
new changes (ideal) or closing and reopening the PR.

**Note**: Manually re-running the checks for the PR does not seem to
re-evaluate label-based conditions; i.e., in the PR, go to "Checks", click
"Re-run all jobs". If you have just removed the label, it will still run the
jobs as though they were skipped.

## Terms and Conditions

Refer to [LICENSE](./LICENSE) §5.
