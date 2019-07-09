#set(PHOBOS_ROOT $ENV{PHOBOS_ROOT})
#if (NOT DEFINED ENV{PHOBOS_ROOT})
#    set(PHOBOS_ROOT "/usr/local/Phobos")
#endif()
set (Phobos_ROOT_DIR "/usr/local/Phobos")

#find_path(PHOBOS_INCLUDE_DIR
#    ${Phobos_ROOT_DIR}/include
#    DOC "The Phobos include directory"
#)

find_library(PHOBOS_LIBRARY 
    NAMES PhobosIntegration
    PATHS ${Phobos_ROOT_DIR}/lib
    DOC "The Phobos library"
)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set LOGGING_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(PHOBOS DEFAULT_MSG PHOBOS_INCLUDE_DIR PHOBOS_LIBRARY)

if (PHOBOS_FOUND)
    set(Phobos_LIBRARIES ${PHOBOS_LIBRARY} )
    set(Phobos_INCLUDE_DIRS ${Phobos_ROOT_DIR}/include )
    set(Phobos_DEFINITIONS )
else(PHOBOS_FOUND)
    message(FATAL_ERROR "Phobos not found")
endif()

# Tell cmake GUIs to ignore the "local" variables.
mark_as_advanced(Phobos_ROOT_DIR PHOBOS_INCLUDE_DIR PHOBOS_LIBRARY)