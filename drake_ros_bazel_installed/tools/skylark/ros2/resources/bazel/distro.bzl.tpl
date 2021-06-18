AVAILABLE_TYPESUPPORTS = @AVAILABLE_TYPESUPPORTS@

# Prepend full paths to not break workspace overlays
RUNTIME_ENVIRONMENT = {
  "AMENT_PREFIX_PATH": ["path-prepend"] + @AMENT_PREFIX_PATH@,
  "${LOAD_PATH}": ["path-prepend"] + @LOAD_PATH@,
}
