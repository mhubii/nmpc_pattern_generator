# Find the rbdl urdf reader addon.
find_path(rbdl_urdfreader_INCLUDE_DIR urdfreader.h /usr/include/rbdl/addons/urdfreader /usr/local/include/rbdl/addons/urdfreader)

find_library(rbdl_urdfreader_LIBRARY
    NAMES rbdl_urdfreader
    PATHS /usr/lib /usr/local/lib
) 

if (rbdl_urdfreader_INCLUDE_DIR AND rbdl_urdfreader_LIBRARY)
   set(rbdl_urdfreader_FOUND TRUE)
endif (rbdl_urdfreader_INCLUDE_DIR AND rbdl_urdfreader_LIBRARY)

if (rbdl_urdfreader_FOUND)
  if (NOT rbdl_urdfreader_FIND_QUIETLY)
    message(STATUS "Found rbdl_urdfreader: ${rbdl_urdfreader_LIBRARY}")
  endif (NOT rbdl_urdfreader_FIND_QUIETLY)
else (rbdl_urdfreader_FOUND)
  if (rbdl_urdfreader_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find rbdl_urdfreader")
  endif (rbdl_urdfreader_FIND_REQUIRED)
endif (rbdl_urdfreader_FOUND)
