# FindGlog.cmake
find_path(GLOG_INCLUDE_DIR glog/logging.h)
find_library(GLOG_LIBRARY NAMES glog)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Glog DEFAULT_MSG GLOG_LIBRARY GLOG_INCLUDE_DIR)

if(Glog_FOUND)
  set(GLOG_LIBRARIES ${GLOG_LIBRARY})
  set(GLOG_INCLUDE_DIRS ${GLOG_INCLUDE_DIR})
endif()

