#include (${navlaserplayer_cmake_dir}/GazeboUtils.cmake)
include (CheckCXXSourceCompiles)

MESSAGE (STATUS "\n\n====== CACHE STRINGS ======")

include (${navlaserplayer_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find packages
if (PKG_CONFIG_FOUND)

  ########################################
  # Find Player
  pkg_check_modules(PLAYER REQUIRED playercore>=3.0)
  if (NOT PLAYER_FOUND)
    set (INCLUDE_PLAYER OFF CACHE BOOL "Set Player Libs" FORCE)
    message (STATUS "Warning: Player not found. ${navlaserplayer} will not be built. See the following website: http://playerstage.sourceforge.net")
  else (NOT PLAYER_FOUND)
    set (INCLUDE_PLAYER ON CACHE BOOL "Set Player Libs" FORCE)
    set (PLAYER_INCLUDE_DIRS ${PLAYER_INCLUDE_DIRS} CACHE INTERNAL
         "Player include directory")
    set (PLAYER_LINK_DIRS ${PLAYER_LIBRARY_DIRS} CACHE INTERNAL
         "Player link directory")
    set (PLAYER_LINK_LIBS ${PLAYER_LIBRARIES} CACHE INTERNAL
         "Player libraries")
    MESSAGE (STATUS "PLAYER FOUND")
  endif (NOT PLAYER_FOUND)

  # Find Player++
  pkg_check_modules(PLAYERC++ REQUIRED playerc++>=3.0) 
  if (NOT PLAYERC++_FOUND)
    set (INCLUDE_PLAYERC++ OFF CACHE BOOL "Set Playerc++ Libs" FORCE)
    message (STATUS "Warning: Playerc++ not found. ${navlaserplayer} will not be built. See the following website: http://playerstage.sourceforge.net")
  else (NOT PLAYERC++_FOUND)
    set (INCLUDE_PLAYERC++ ON CACHE BOOL "Set Player Libs" FORCE)
    set (PLAYERC++_INCLUDE_DIRS ${PLAYERC++_INCLUDE_DIRS} CACHE INTERNAL "Player include directory")
    set (PLAYERC++_LINK_DIRS ${PLAYERC++_LIBRARY_DIRS} CACHE INTERNAL "Player link directory")
    set (PLAYERC++_LINK_LIBS ${PLAYERC++_LIBRARIES} CACHE INTERNAL "Player libraries")
    MESSAGE (STATUS "PLAYERRC++ FOUND")
  endif (NOT PLAYERC++_FOUND)

  # Find Gazebo
  pkg_check_modules(GAZEBOSERVER libgazebo>=0.10.0) 
  if (NOT GAZEBOSERVER_FOUND)
    set (INCLUDE_GAZEBOSERVER OFF CACHE BOOL "Set Gazebo Libs" FORCE)
    message (STATUS "Warning: Gazebo not found. ${navlaserplayer} will not be built. See the following website: http://playerstage.sourceforge.net")
    SET (USEGAZEBO "")
  else (NOT GAZEBOSERVER_FOUND)
    set (INCLUDE_GAZEBOSERVER ON CACHE BOOL "Set Gazebo Libs" FORCE)
    set (GAZEBOSERVER_INCLUDE_DIRS ${GAZEBOSERVER_INCLUDE_DIRS} CACHE INTERNAL "Gazebo include directory")
    set (GAZEBOSERVER_LINK_DIRS ${GAZEBOSERVER_LIBRARY_DIRS} CACHE INTERNAL "Gazebo link directory")
    set (GAZEBOSERVER_LINK_LIBS ${GAZEBOSERVER_LIBRARIES} CACHE INTERNAL "Gazebo libraries")
    MESSAGE (STATUS "GAZEBO FOUND - OK")
  endif (NOT GAZEBOSERVER_FOUND)

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

  ## GTK2 needed
  pkg_check_modules(GTK REQUIRED gtk+-2.0>=2.0.0)
  if (NOT GTK_FOUND)
    set (INCLUDE_GTK OFF CACHE BOOL "SET GTK FORCE" FORCE)
    message (STATUS "Warning: GTK not found. ${navlaserplayer} will not be built. See the following website: TBC")
  else (NOT GTK_FOUND)
    set (INCLUDE_GTK2 ON CACHE BOOL "Set gtk Libs" FORCE)
    set (GTK_INCLUDE_DIRS ${GTK_INCLUDE_DIRS} CACHE INTERNAL "GTK include directory")
    set (GTK_LINK_DIRS ${GTK_LIBRARY_DIRS} CACHE INTERNAL "GTK link directory")
    set (GTK_LINK_LIBS ${GTK_LIBRARIES} CACHE INTERNAL "GTK libraries")
    MESSAGE (STATUS "GTK FOUND")
  endif (NOT GTK_FOUND) 

  message("GTK INCLUDE DIRS: " ${GTK_INCLUDE_DIRS} )
  message("GTK PKG LIB DIR: " ${GTK_LIBRARY_DIRS} )
  message("GTK PKG LIB: " ${GTK_LIBRARIES} )


  ## GTK2 needed
  pkg_check_modules(OIS REQUIRED OIS>=1.3.0)
  if (NOT OIS_FOUND)
    set (INCLUDE_OIS OFF CACHE BOOL "SET OIS FORCE" FORCE)
    message (STATUS "Warning: OIS not found. ${navlaserplayer} will not be built. See the following website: TBC")
  else (NOT OIS_FOUND)
    set (INCLUDE_OIS ON CACHE BOOL "Set OIS Libs" FORCE)
    set (OIS_INCLUDE_DIRS ${OIS_INCLUDE_DIRS} CACHE INTERNAL "OIS include directory")
    set (OIS_LINK_DIRS ${OIS_LIBRARY_DIRS} CACHE INTERNAL "OIS link directory")
    set (OIS_LINK_LIBS ${OIS_LIBRARIES} CACHE INTERNAL "OIS libraries")
    MESSAGE (STATUS "OIS FOUND")
  endif (NOT OIS_FOUND)

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
# Find libtool
FIND_PATH(libtool_include_dir ltdl.h /usr/include /usr/local/include)
IF (NOT libtool_include_dir)
  MESSAGE (STATUS "Looking for ltdl.h - not found")
  MESSAGE (STATUS "Warning: Unable to find libtool, plugins will not be supported.")
  SET (libtool_include_dir /usr/include)
ELSE (NOT libtool_include_dir)
  MESSAGE (STATUS "Looking for ltdl.h - found")
ENDIF (NOT libtool_include_dir)

FIND_LIBRARY(libtool_library ltdl /usr/lib /usr/local/lib)
IF (NOT libtool_library)
  MESSAGE (STATUS "Looking for libltdl - not found")

ELSE (NOT libtool_library)
  MESSAGE (STATUS "Looking for libltdl - found")
ENDIF (NOT libtool_library)

IF (libtool_library AND libtool_include_dir)
  SET (HAVE_LTDL TRUE)
ENDIF (libtool_library AND libtool_include_dir)



########################################

# Set hardcoded path guesses for various platforms
if (UNIX)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local /usr/local/lib/OGRE/cmake)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib/OGRE/cmake)
endif ()

########################################
# Find OGRE
find_package(OGRE REQUIRED)

#### SET QT
# find QT stuff - QTcore
# find and setup Qt4 for this project
FIND_PACKAGE(Qt4 REQUIRED)

