# Find the pattern generator library.
find_path(PATTERN_GENERATOR_INCLUDE_DIR base_generator.h  
                                        interpolation.h 
                                        mpc_generator.h 
                                        nmpc_generator.h
                                        utils.h ${PATTERN_GENERATOR_DIR}/include/pattern_generator)

find_library(PATTERN_GENERATOR_LIBRARY
    NAMES pattern_generator
    PATHS ${PATTERN_GENERATOR_DIR}/lib
) 

if (PATTERN_GENERATOR_INCLUDE_DIR AND PATTERN_GENERATOR_LIBRARY)
   set(PATTERN_GENERATOR_FOUND TRUE)
endif ()

if (PATTERN_GENERATOR_FOUND)
  if (NOT PATTERN_GENERATOR_FIND_QUIETLY)
    message(STATUS "Found pattern_generator: ${PATTERN_GENERATOR_LIBRARY}")
  endif ()
else (PATTERN_GENERATOR_FOUND)
  if (PATTERN_GENERATOR_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find pattern_generator")
  endif ()
endif ()
