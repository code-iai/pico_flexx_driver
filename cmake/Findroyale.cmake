# search for royale SDK
# find all `share` directories in `royale` directory
execute_process(COMMAND find "${PROJECT_SOURCE_DIR}/royale/" -type d -name "share" OUTPUT_VARIABLE PATHS_STRING)
if(PATHS_STRING)
  string(REPLACE "\n" ";" PATHS_LIST ${PATHS_STRING})
  # sort the list in reverse order to get the newest version first
  list(SORT PATHS_LIST)
  list(REVERSE PATHS_LIST)

  # store CXX flags to override the settings from royale later
  set(CMAKE_CXX_FLAGS_OLD "${CMAKE_CXX_FLAGS}")

  find_package(royale REQUIRED
    PATHS ${PATHS_LIST}
    NO_DEFAULT_PATH
  )

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_OLD}")
endif()

FIND_PACKAGE_HANDLE_STANDARD_ARGS(royale
  REQUIRED_VARS royale_LIB_DIR royale_LIBRARIES royale_INCLUDE_DIRS
  FAIL_MESSAGE "Could not find royale SDK! please make sure to extract the royale SDK to ${PROJECT_SOURCE_DIR}/royale/"
)
