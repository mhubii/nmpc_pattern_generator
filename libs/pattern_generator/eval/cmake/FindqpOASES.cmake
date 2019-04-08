# Find the qpOASES library.
find_path(qpOASES_INCLUDE_DIR qpOASES.hpp /usr/include /usr/local/include)

find_library(qpOASES_LIBRARY
    NAMES qpOASES
    PATHS /usr/lib /usr/local/lib
) 

if (qpOASES_INCLUDE_DIR AND qpOASES_LIBRARY)
   set(qpOASES_FOUND TRUE)
endif (qpOASES_INCLUDE_DIR AND qpOASES_LIBRARY)

if (qpOASES_FOUND)
  if (NOT qpOASES_FIND_QUIETLY)
    message(STATUS "Found qpOASES: ${qpOASES_LIBRARY}")
  endif (NOT qpOASES_FIND_QUIETLY)
else (qpOASES_FOUND)
  if (qpOASES_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find qpOASES")
  endif (qpOASES_FIND_REQUIRED)
endif (qpOASES_FOUND)
