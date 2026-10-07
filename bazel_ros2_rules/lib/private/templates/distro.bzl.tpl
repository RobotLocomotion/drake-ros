AVAILABLE_TYPESUPPORT_LIST = @AVAILABLE_TYPESUPPORT_LIST@

REPOSITORY_ROOT = @REPOSITORY_ROOT@

# Prepend full paths to not break workspace overlays
RUNTIME_ENVIRONMENT = {
  "AMENT_PREFIX_PATH": ["path-prepend"] + @AMENT_PREFIX_PATHS@,
  "${LOAD_PATH}": ["path-prepend"] + @LOAD_PATHS@,
  "PYTHONPATH": ["path-prepend"] + @PYTHON_PATHS@,
  "ROS_AUTOMATIC_DISCOVERY_RANGE": ["set-if-not-set", @DEFAULT_AUTOMATIC_DISCOVERY_RANGE@],
  "ROS_DISTRO": ["set-if-not-set", @DEFAULT_ROS_DISTRO@],
}
