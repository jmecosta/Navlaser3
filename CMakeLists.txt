CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

# ADD_DEFINITIONS(-DUSEGAZEBO )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
  CMAKE_POLICY(SET CMP0007 NEW)
  CMAKE_POLICY(SET CMP0011 NEW)
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

set(QT_USE_OPENGL TRUE)
set(QT_USE_QTCORE TRUE)
include(${QT_USE_FILE})
MESSAGE (STATUS "QT DEFS: "  ${QT_USE_FILE} " " ${QT_INCLUDE_DIR} " " ${QT_QTOPENGL_INCLUDE_DIR} " " ${QT_QTCORE_INCLUDE_DIR} " " ${QT_LIBRARIES})

##
## SET GUI - QT WRAPPER FOR UI objs
##
SET(navlaser_UIS
    src/hmi/nav3mainwindow.ui
)
QT4_WRAP_UI(navlaser_UIS_H ${navlaser_UIS})

set(Qtwindow_HDRS
    src/hmi/nav3mainwindow.h
    ${navlaser_UIS_H}
)
QT4_WRAP_CPP(MOC_OUTFILES ${Qtwindow_HDRS})


# Add library and include paths from the dependencies
#### project specific stuff
INCLUDE_DIRECTORIES(${GAZEBOSERVER_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${PLAYER_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${GTK_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OGRE_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${QT_QTCORE_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${QT_QTOPENGL_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${OIS_INCLUDE_DIRS})



#### project specific source
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/hmi)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/config)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/control)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/features)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/map)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/math)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/rtk2)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/sensors)

LINK_DIRECTORIES(${PLAYER_LINK_DIRS})
LINK_DIRECTORIES(${GAZEBOSERVER_LINK_DIRS})
LINK_DIRECTORIES(${GSL_LINK_DIRS})
LINK_DIRECTORIES(${boost_library_dirs})
LINK_DIRECTORIES(${GTK_LIBRARY_DIRS})
LINK_DIRECTORIES(${OGRE_LIBRARY_DIRS})
LINK_DIRECTORIES(${OIS_LINK_DIRS})



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
                                src/hmi/scene_manager.cpp
                                ${MOC_OUTFILES}
                                src/hmi/nav3mainwindow.cpp
                                main.cpp)

target_link_libraries(navlaserplayer ${PLAYER_LINK_LIBS}
                                     ${PLAYERC++_LINK_LIBS}
                                     ${GAZEBOSERVER_LINK_LIBS}
                                     ${GSL_LINK_LIBS}
                                     ${boost_libraries}
                                     ${GTK_LIBRARIES}
                                     ${OGRE_LIBRARIES}
                                     ${QT_LIBRARIES}
                                     ${QT_QTOPENGL_LIBRARY}
                                     ${OIS_LINK_LIBS}
                      ) #link the math library
