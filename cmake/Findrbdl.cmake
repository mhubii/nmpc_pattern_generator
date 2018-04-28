# Find the rbdl library.
find_path(rbdl_INCLUDE_DIR rbdl.h /usr/include/rbdl /usr/local/include/rbdl)

find_library(rbdl_LIBRARY
    NAMES rbdl
    PATHS /usr/lib /usr/local/lib
) 

if (rbdl_INCLUDE_DIR AND rbdl_LIBRARY)
   set(rbdl_FOUND TRUE)
endif (rbdl_INCLUDE_DIR AND rbdl_LIBRARY)

if (rbdl_FOUND)
  if (NOT rbdl_FIND_QUIETLY)
    message(STATUS "Found rbdl: ${rbdl_LIBRARY}")
  endif (NOT rbdl_FIND_QUIETLY)
else (rbdl_FOUND)
  if (rbdl_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find rbdl")
  endif (rbdl_FIND_REQUIRED)
endif (rbdl_FOUND)
