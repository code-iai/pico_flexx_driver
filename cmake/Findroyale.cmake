
find_path(royale_INCLUDE_DIR
  royale.hpp
  DOC "Found royale include directory"
)

find_library(royale_LIBRARY
  NAMES royale
  DOC "Found royale library"
)

set(royale_LIBRARIES ${royale_LIBRARY} )
set(royale_INCLUDE_DIRS ${royale_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set royale_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(royale DEFAULT_MSG
  royale_LIBRARY royale_INCLUDE_DIR)
