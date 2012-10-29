PROJECT(navlaserplayer)

CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
  CMAKE_POLICY(SET CMP0007 NEW)
  CMAKE_POLICY(SET CMP0011 NEW)
  CMAKE_POLICY(SET CMP0002 NEW)
ENDIF(COMMAND CMAKE_POLICY)

MESSAGE (STATUS "NAVLASEREXEC: Project Source Dir:" ${PROJECT_SOURCE_DIR} )


#### project specific stuff
SET(PROJECT_INCLUDE_EXTERNALS 	${GAZEBOSERVER_INCLUDE_DIRS}
								${PLAYER_INCLUDE_DIRS}
								${GSL_INCLUDE_DIRS}
								${Boost_INCLUDE_DIRS}
								${GTK_INCLUDE_DIRS}
								${OGRE_INCLUDE_DIRS}
								${QT_QTCORE_INCLUDE_DIR}
								${QT_QTOPENGL_INCLUDE_DIR} CACHE INTERNAL "Sources for the project" FORCE)

SET(PROJECT_INCLUDE_DIRS 	${PROJECT_SOURCE_DIR}/
						 	${PROJECT_SOURCE_DIR}/hmi
							${PROJECT_SOURCE_DIR}/config
							${PROJECT_SOURCE_DIR}/control
							${PROJECT_SOURCE_DIR}/features
							${PROJECT_SOURCE_DIR}/map
							${PROJECT_SOURCE_DIR}/math
							${PROJECT_SOURCE_DIR}/rtk2
							${PROJECT_SOURCE_DIR}/sensors CACHE INTERNAL "Sources for the project" FORCE)

INCLUDE_DIRECTORIES(${PROJECT_INCLUDE_DIRS} ${PROJECT_INCLUDE_EXTERNALS})

# Set hardcoded path guesses for various platforms
MESSAGE (STATUS "\n\n====== FIND QT ====== " )

INCLUDE (${PROJECT_ROOT_DIR}/thirdparty/share/cmake-2.8/Modules/FindQt4.cmake)

#FIND_PACKAGE(Qt4 REQUIRED) 

set(QT_USE_OPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(CMAKE_AUTOMOC TRUE)

MESSAGE (STATUS "\n\n====== QTUSEFILE" ${QT_USE_FILE})
INCLUDE (${PROJECT_ROOT_DIR}/thirdparty/share/cmake-2.8/Modules/UseQt4.cmake)

ADD_DEFINITIONS(${QT_DEFINITIONS})

##
## SET GUI - QT WRAPPER FOR UI objs
##
#SET(himlib_UIS ${PROJECT_ROOT_DIR}/src/hmi/nav3mainwindow.ui)
#QT4_WRAP_UI(himlib_UIS_H ${himlib_UIS})

#set(Qtwindow_HDRS ${PROJECT_ROOT_DIR}/src/hmi/nav3mainwindow.h ${himlib_UIS_H})
#QT4_WRAP_CPP(MOC_OUTFILES ${Qtwindow_HDRS})

set(SRCDIRHMI ${PROJECT_ROOT_DIR}/src/hmi/nav3mainwindow.cpp 
		   ${PROJECT_ROOT_DIR}/src/hmi/scene_manager.cpp ) 
		   
# SET Project sources
set(HMI_SRC ${MOC_OUTFILES}  ${SRCDIRHMI} )

# SET Project sources
set(CONFIGLIB_SRC  ${PROJECT_SOURCE_DIR}/config/config_extractor.cpp
                   ${PROJECT_SOURCE_DIR}/config/config_movement.cpp
                   ${PROJECT_SOURCE_DIR}/config/config_map.cpp )

# SET Project sources
SET(CONTROL_SRC	${PROJECT_SOURCE_DIR}/control/nav_motionestimator.cpp )

# SET Project sources
SET(FEATURES_SRC	${PROJECT_SOURCE_DIR}/features/feature_c.cpp
					${PROJECT_SOURCE_DIR}/features/features_extractor_t.cpp
					${PROJECT_SOURCE_DIR}/features/line_t.cpp
					${PROJECT_SOURCE_DIR}/features/point_t.cpp )

# SET Project sources
set(MAP_SRC  ${PROJECT_SOURCE_DIR}/map/grid_map_t.cpp 
			 ${PROJECT_SOURCE_DIR}/map/features_map_t.cpp )
			 
# SET Project sources
set(MATH_SRC  ${PROJECT_SOURCE_DIR}/math/math_lib.cpp )

# SET Project sources
SET(RTK2_SRC	${PROJECT_SOURCE_DIR}/rtk2/rtk.c
            	${PROJECT_SOURCE_DIR}/rtk2/rtk_canvas.c
            	${PROJECT_SOURCE_DIR}/rtk2/rtk_canvas_movie.c
            	${PROJECT_SOURCE_DIR}/rtk2/rtk_fig.c
            	${PROJECT_SOURCE_DIR}/rtk2/rtk_menu.c
            	${PROJECT_SOURCE_DIR}/rtk2/rtk_region.c
            	${PROJECT_SOURCE_DIR}/rtk2/rtk_table.c
            	${PROJECT_SOURCE_DIR}/rtk2/mainwnd.cpp )
            	
# SET Project sources
set(SENSORS_SRC  ${PROJECT_SOURCE_DIR}/sensors/laser_t.cpp 
				 ${PROJECT_SOURCE_DIR}/sensors/lmsscan_t.cpp )
            	
			                
LINK_DIRECTORIES(${PLAYER_LINK_DIRS})
LINK_DIRECTORIES(${GAZEBOSERVER_LINK_DIRS})
LINK_DIRECTORIES(${GSL_LINK_DIRS})
LINK_DIRECTORIES(${boost_library_dirs})
LINK_DIRECTORIES(${GTK_LIBRARY_DIRS})
LINK_DIRECTORIES(${OGRE_LIBRARY_DIRS})


### sET project source files
set(NAVLASER3_SRC  ${CONFIGLIB_SRC}
							  ${CONTROL_SRC} 
							  ${FEATURES_SRC}
							  ${HMI_SRC}
							  ${MAP_SRC}
							  ${MATH_SRC}
							  ${RTK2_SRC}
							  ${SENSORS_SRC} CACHE INTERNAL "Sources for the project" FORCE)

### sET project source files
set(NAVLASER3_SRC_TEST  ${CONFIGLIB_SRC}
							  ${CONTROL_SRC} 
							  ${FEATURES_SRC}
							  ${SRCDIRHMI}
							  ${MAP_SRC}
							  ${MATH_SRC}
							  ${RTK2_SRC}
							  ${SENSORS_SRC} CACHE INTERNAL "Sources for the project" FORCE)

							                                       
#  ${QTCORE_LINK_LIBS}
ADD_EXECUTABLE(navlaserplayer main.cpp ${NAVLASER3_SRC} )


### main link libraries set      
TARGET_LINK_LIBRARIES(navlaserplayer  ${NAVLASER3_LINK_LIBS})


