#include (${navlaserplayer_cmake_dir}/GazeboUtils.cmake)
include (CheckCXXSourceCompiles)

MESSAGE (STATUS "\n\n====== CACHE STRINGS ======")

include (${navlaserplayer_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find packages
if (PKG_CONFIG_FOUND)

  # find GSL
  pkg_check_modules(GSL REQUIRED gsl>=0.10.0)
  if (NOT GSL_FOUND)
    set (INCLUDE_GSL OFF CACHE BOOL "Set Playerc++ Libs" FORCE)
    message (STATUS "Warning: Playerc++ not found. ${navlaserplayer} will not be built. See the following website: http://www.gnu.org/software/gsl")
  else (NOT GSL_FOUND)
    set (INCLUDE_GSL ON CACHE BOOL "Set gsl Libs" FORCE)
    set (GSL_INCLUDE_DIRS ${GSL_INCLUDE_DIRS} CACHE INTERNAL "gsl include directory")
    set (GSL_LINK_DIRS ${GSL_LIBRARY_DIRS} CACHE INTERNAL "gsl link directory")
    set (GSL_LINK_LIBS ${GSL_LIBRARIES} CACHE INTERNAL "gsl libraries")
    MESSAGE (STATUS "GSL FOUND")
  endif (NOT GSL_FOUND)  


  message("GTK INCLUDE DIRS: " ${GTK_INCLUDE_DIRS} )
  message("GTK PKG LIB DIR: " ${GTK_LIBRARY_DIRS} )
  message("GTK PKG LIB: " ${GTK_LIBRARIES} )


else (PKG_CONFIG_FOUND)
  set (BUILD_NAVLASERPLAYER OFF CACHE INTERNAL "Build Navlaserplayer" FORCE)
  message (FATAL_ERROR "\nError: pkg-config not found")
endif (PKG_CONFIG_FOUND)


message (STATUS "CURRENT boost include dirs: ${boost_include_dirs}")
message (STATUS "CURRENT boost library dirs: ${boost_library_dirs}")
message (STATUS "CURRENT boost libraries: ${boost_libraries}")

set (boost_include_dirs "")
set (boost_library_dirs "")
set (boost_libraries "")

########################################
# Find Boost, if not specified manually
IF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries )

  # Clear some variables to ensure that the checks for boost are 
  # always run
  SET (Boost_THREAD_FOUND OFF CACHE INTERNAL "" FORCE)
  SET (Boost_SIGNALS_FOUND OFF CACHE INTERNAL "" FORCE)

  SET(Boost_ADDITIONAL_VERSIONS "1.35" "1.35.0" "1.36" "1.36.1" "1.37.0" "1.39.0" "1.42.0" CACHE INTERNAL "Boost Additional versions" FORCE)
  SET(MIN_BOOST_VERSION "1.42.0")
  INCLUDE (FindBoost)
  message (STATUS "MIN BOOST ${MIN_BOOST_VERSION}")
  
  FIND_PACKAGE( Boost ${MIN_BOOST_VERSION} REQUIRED thread-mt)

  SET (boost_include_dirs ${Boost_INCLUDE_DIRS} CACHE STRING 
    "Boost include paths. Use this to override automatic detection." FORCE)

  SET (boost_library_dirs ${Boost_LIBRARY_DIRS} CACHE STRING
    "Boost link dirs. Use this to override automatic detection." FORCE)
 
  SET (boost_libraries ${Boost_LIBRARIES} CACHE STRING 
    "Boost libraries. Use this to override automatic detection." FORCE )

ENDIF (NOT boost_include_dirs AND NOT boost_library_dirs AND NOT boost_libraries ) 

STRING(REGEX REPLACE "(^| )-L" " " boost_library_dirs "${boost_library_dirs}")
STRING(REGEX REPLACE "(^| )-l" " " boost_libraries "${boost_libraries}")
#STRING(STRIP ${boost_library_dirs} boost_library_dirs)
#STRING(STRIP ${boost_libraries} boost_libraries)
STRING(REGEX REPLACE " " ";" boost_libraries "${boost_libraries}")

message (STATUS "boost include dirs: ${boost_include_dirs}")
message (STATUS "boost library dirs: ${boost_library_dirs}")
message (STATUS "boost libraries: ${boost_libraries}")

#### if python should be used within the app
#find_package(PythonLibs REQUIRED)
#message("Include dirs of Python: " ${PYTHON_INCLUDE_DIRS} )
#message("Libs of Python: " ${PYTHON_LIBRARIES} )


########################################



########################################

# Set hardcoded path guesses for various platforms
if (UNIX)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local /usr/local/lib/OGRE/cmake)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib/OGRE/cmake)
endif ()


#### SET QT
# find QT stuff - QTcore
# find and setup Qt4 for this project
FIND_PACKAGE(Qt4 REQUIRED)

