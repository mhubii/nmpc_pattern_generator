# Find the kinematics library.
find_path(KINEMATICS_INCLUDE_DIR kinematics.h
                                        utils.h ${KINEMATICS_DIR}/include/kinematics)

find_library(KINEMATICS_LIBRARY
    NAMES kinematics
    PATHS ${KINEMATICS_DIR}/lib
) 

if (KINEMATICS_INCLUDE_DIR AND KINEMATICS_LIBRARY)
   set(KINEMATICS_FOUND TRUE)
endif ()

if (KINEMATICS_FOUND)
  if (NOT KINEMATICS_FIND_QUIETLY)
    message(STATUS "Found kinematics: ${KINEMATICS_LIBRARY}")
  endif ()
else (KINEMATICS_FOUND)
  if (KINEMATICS_FIND_REQUIRED)
    message(FATAL_ERROR "Could not find kinematics")
  endif ()
endif ()
