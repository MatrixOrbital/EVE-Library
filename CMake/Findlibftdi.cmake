# # Copyright 2009  SoftPLC Corporation  http://softplc.com
# Dick Hollenbeck <d...@softplc.com>
# License: GPL v2
#
# - Try to find libftdi
#
# Before calling, USE_STATIC_FTDI may be set to mandate a STATIC library
#
# Once done this will define
#
# libftdi_FOUND - system has libftdi
# libftdi_INCLUDE_DIR - the libftdi include directory
# libftdi_LIBRARIES - Link these to use libftdi
#
# also exported a modern cmake target called libftdi 
#
if (NOT libftdi_FOUND)

    if(NOT WIN32)
        include(CMakeFindDependencyMacro)
        find_dependency(PkgConfig)
        pkg_check_modules(libftdi_PKG libftdi1)
    endif(NOT WIN32)

    find_path(libftdi_INCLUDE_DIR
        NAMES
            ftdi.h
        HINTS
            ${libftdi_PKG_INCLUDE_DIRS}
        PATHS
            /usr/include
            /usr/include/libftdi1
            /usr/local/include
    )

    if(USE_STATIC_FTDI)
        set(_save ${CMAKE_FIND_LIBRARY_SUFFIXES})
        set(CMAKE_FIND_LIBRARY_SUFFIXES ${CMAKE_STATIC_LIBRARY_SUFFIX})
    endif(USE_STATIC_FTDI)

    find_library(libftdi_LIBRARIES
        NAMES
            ftdi1
        HINTS
            ${libftdi_PKG_LIBRARY_DIRS}
        PATHS
            /usr/lib
            /usr/lib/x86_64-linux-gnu/
            /usr/local/lib
    )

    if(USE_STATIC_FTDI)
        set(CMAKE_FIND_LIBRARY_SUFFIXES ${_save} )
    endif(USE_STATIC_FTDI)
    include(FindPackageHandleStandardArgs)

    # handle the QUIETLY AND REQUIRED arguments AND set libftdi_FOUND to TRUE if
    # all listed variables are TRUE
    find_package_handle_standard_args(libftdi DEFAULT_MSG libftdi_LIBRARIES libftdi_INCLUDE_DIR)

    if(USE_STATIC_FTDI)
        add_library(libftdi STATIC IMPORTED)
    else(USE_STATIC_FTDI)
        add_library(libftdi SHARED IMPORTED)
    endif(USE_STATIC_FTDI)

    set_target_properties(libftdi PROPERTIES IMPORTED_LOCATION ${libftdi_LIBRARIES})
    set_target_properties(libftdi PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${libftdi_INCLUDE_DIR})
    set(${libftdi_LIBRARIES} libftdi)

    #mark_as_advanced(libftdi_INCLUDE_DIR libftdi_LIBRARIES)

endif(NOT libftdi_FOUND)