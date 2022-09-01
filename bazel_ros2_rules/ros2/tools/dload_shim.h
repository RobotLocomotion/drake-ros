#include <string>
#include <vector>

int do_dload_shim(
  int argc,
  const char ** argv,
  const char * executable_path,
  std::vector<const char *> names,
  std::vector<std::vector<const char *>> actions);
