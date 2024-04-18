#include "logerr_info.h"

const char *epicsBaseDebugStripPath(const char *file) {
  const char *ret = strrchr(file, '/');
  if (ret) return ret + 1;
#if (defined(CYGWIN32) || defined(_WIN32))
  ret = strrchr(file, '\\');
  if (ret) return ret + 1;
#endif
  return file;
}
