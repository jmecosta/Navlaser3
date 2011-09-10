CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

# ADD_DEFINITIONS(-DUSEGAZEBO )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
ENDIF(COMMAND CMAKE_POLICY)

include (CheckCXXSourceCompiles)

PROJECT(navlaserplayer)

MESSAGE (STATUS "\n\n====== PROJECT PATH: ${PROJECT_SOURCE_DIR} ======")
set (navlaserplayer_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH 
     "Location of CMake scripts")

# set install prefix
SET(CMAKE_INSTALL_PREFIX "/home/jorge/PLAYERSTAGE/")

#####################################
# Build type cflags
SET (CMAKE_C_FLAGS_RELEASE " -ggdb -O2 -DNDEBUG -Wall -fPIC " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_C_FLAGS_DEBUG " -ggdb  -Wall -fPIC " CACHE INTERNAL "C Flags for debug" FORCE)
SET (CMAKE_C_FLAGS_PROFILE " -ggdb -pg -Wall -fPIC " CACHE INTERNAL "C Flags for profile" FORCE)

#####################################
# Build type link flags
SET (CMAKE_LINK_FLAGS_RELEASE "" CACHE INTERNAL "Link flags fro release" FORCE)
SET (CMAKE_LINK_FLAGS_DEBUG "" CACHE INTERNAL "Link flags for debug" FORCE)
SET (CMAKE_LINK_FLAGS_PROFILE "" CACHE INTERNAL "Link flags for profile" FORCE)

MESSAGE (STATUS "\n\n====== Finding 3rd Party Packages ======")
INCLUDE (${navlaserplayer_cmake_dir}/SearchForStuff.cmake)
MESSAGE (STATUS "----------------------------------------\n")


#### project specific stuff
INCLUDE_DIRECTORIES(${GAZEBOSERVER_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PLAYER_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES (${GTK_INCLUDE_DIRS})
INCLUDE_DIRECTORIES (${navlaserplayer_cmake_dir}/src/rtk2)

#INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS})
#INCLUDE_DIRECTORIES(${QTCORE_INCLUDE_DIRS})

LINK_DIRECTORIES(${PLAYER_LINK_DIRS})
LINK_DIRECTORIES(${GAZEBOSERVER_LINK_DIRS})
LINK_DIRECTORIES(${GSL_LINK_DIRS})
LINK_DIRECTORIES(${boost_library_dirs})
LINK_DIRECTORIES(${GTK_LIBRARY_DIRS})
#LINK_DIRECTORIES(${QTCORE_LINK_DIRS})

#  ${QTCORE_LINK_LIBS}
ADD_EXECUTABLE(navlaserplayer src/math/math_lib.cpp 
				src/rtk2/rtk.c
				src/rtk2/rtk_canvas.c
				src/rtk2/rtk_canvas_movie.c
				src/rtk2/rtk_fig.c
				src/rtk2/rtk_menu.c
				src/rtk2/rtk_region.c
				src/rtk2/rtk_table.c
				src/rtk2/mainwnd.cpp
				src/config/config_extractor.cpp 
				src/config/config_movement.cpp 
				src/config/config_map.cpp 
				src/features/feature_c.cpp 
				src/sensors/laser_t.cpp 
				src/map/grid_map_t.cpp 
				src/map/grid_map_t.cpp 
				src/map/features_map_t.cpp 
				src/sensors/lmsscan_t.cpp 
				src/features/features_extractor_t.cpp 
				src/features/point_t.cpp 
				src/features/line_t.cpp 
				src/control/nav_motionestimator.cpp  
				main.cpp)
target_link_libraries(navlaserplayer ${PLAYER_LINK_LIBS} ${PLAYERC++_LINK_LIBS} ${GAZEBOSERVER_LINK_LIBS} ${GSL_LINK_LIBS} ${boost_libraries} ${GTK_LIBRARIES}) #link the math library
