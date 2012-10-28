#include (${navlaserplayer_cmake_dir}/GazeboUtils.cmake)
include (CheckCXXSourceCompiles)

MESSAGE (STATUS "\n\n====== CACHE STRINGS ======")

include (${navlaserplayer_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

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

