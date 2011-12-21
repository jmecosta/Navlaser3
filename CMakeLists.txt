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
SET (CMAKE_CXX_FLAGS_RELEASE " -ggdb -O2 -DNDEBUG -Wall -fPIC " CACHE INTERNAL "C Flags for release" FORCE)
SET (CMAKE_C_FLAGS_DEBUG " -ggdb  -Wall -fPIC " CACHE INTERNAL "C Flags for debug" FORCE)
SET (CMAKE_CXX_FLAGS_DEBUG " -ggdb  -Wall -fPIC " CACHE INTERNAL "C Flags for debug" FORCE)
SET (CMAKE_C_FLAGS_PROFILE " -ggdb -pg -Wall -fPIC " CACHE INTERNAL "C Flags for profile" FORCE)
SET (CMAKE_CXX_FLAGS_PROFILE " -ggdb -pg -Wall -fPIC " CACHE INTERNAL "C Flags for profile" FORCE)

#####################################
# Build type link flags
#SET (CMAKE_CXX_LINK_FLAGS_RELEASE " -ldl " CACHE INTERNAL "Link flags fro release" FORCE)
#SET (CMAKE_CXX_LINK_FLAGS_DEBUG " -ldl " CACHE INTERNAL "Link flags for debug" FORCE)
#SET (CMAKE_CXX_LINK_FLAGS_PROFILE " -ldl " CACHE INTERNAL "Link flags for profile" FORCE)


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
LINK_DIRECTORIES(${CPPUNIT_LIBRARY_DIRS})
LINK_DIRECTORIES(${OGRE_LIBRARY_DIRS})


### sET project source files
set(NAVLASER3_SRC  ${PROJECT_SOURCE_DIR}/src/math/math_lib.cpp
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk_canvas.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk_canvas_movie.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk_fig.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk_menu.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk_region.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/rtk_table.c
                   ${PROJECT_SOURCE_DIR}/src/rtk2/mainwnd.cpp
                   ${PROJECT_SOURCE_DIR}/src/config/config_extractor.cpp
                   ${PROJECT_SOURCE_DIR}/src/config/config_movement.cpp
                   ${PROJECT_SOURCE_DIR}/src/config/config_map.cpp
                   ${PROJECT_SOURCE_DIR}/src/features/feature_c.cpp
                   ${PROJECT_SOURCE_DIR}/src/sensors/laser_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/map/grid_map_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/map/grid_map_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/map/features_map_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/sensors/lmsscan_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/features/features_extractor_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/features/point_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/features/line_t.cpp
                   ${PROJECT_SOURCE_DIR}/src/control/nav_motionestimator.cpp
                   ${PROJECT_SOURCE_DIR}/src/hmi/scene_manager.cpp
                   ${MOC_OUTFILES}
                   ${PROJECT_SOURCE_DIR}/src/hmi/nav3mainwindow.cpp
                   ${PROJECT_SOURCE_DIR}/main.cpp )

### main link libraries set
set(NAVLASER3_LINK_LIBS ${PLAYER_LINK_LIBS}
                        ${PLAYERC++_LINK_LIBS}
                        ${GAZEBOSERVER_LINK_LIBS}
                        ${GSL_LINK_LIBS}
                        ${boost_libraries}
                        ${GTK_LIBRARIES}
                        ${CPPUNIT_LIBRARIES}
                        ${OGRE_LIBRARIES}
                        ${QT_LIBRARIES}
                        ${QT_QTOPENGL_LIBRARY} )

#  ${QTCORE_LINK_LIBS}
ADD_EXECUTABLE(navlaserplayer ${NAVLASER3_SRC} )

TARGET_LINK_LIBRARIES(navlaserplayer  ${NAVLASER3_LINK_LIBS})

#####################################
#### unit test specific
#### definition of test environment
####
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/test)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/test/common)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/test/config)
INCLUDE_DIRECTORIES(${CPPUNIT_INCLUDE_DIRS})

#### define SRC files for project specific unit tests
set(NAVLASER3_CONFIG_TESTS_SRC ${PROJECT_SOURCE_DIR}/test/common/myxmloutputterhook.cpp
        ${PROJECT_SOURCE_DIR}/test/config/config_extractorTest.cpp
        ${PROJECT_SOURCE_DIR}/src/config/config_extractor.cpp
        ${PROJECT_SOURCE_DIR}/test/config/main.cpp )

#### define project executable for config testing
ADD_EXECUTABLE(navlaser3-config-tests ${NAVLASER3_CONFIG_TESTS_SRC})

#### add link libraries for unit testing
TARGET_LINK_LIBRARIES(navlaser3-config-tests    ${NAVLASER3_LINK_LIBS}
                                                ${CPPUNIT_LIBRARIES} )

message (STATUS "CURRENT link libraries: ${NAVLASER3_LINK_LIBS}")

#####################################
# CTEST ENABLED
ENABLE_TESTING()
#####################################
# CTEST DEFINITIONS
ADD_TEST(test_config_tests "navlaser3-config-tests")
SET_TESTS_PROPERTIES(test_config_tests PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR;FAIL;Test failed")
SET_TESTS_PROPERTIES(test_config_tests PROPERTIES PASS_REGULAR_EXPRESSION "Test passed")
ADD_TEST(test_config_tests_1 "navlaser3-config-tests")
SET_TESTS_PROPERTIES(test_config_tests PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR;FAIL;Test failed")
SET_TESTS_PROPERTIES(test_config_tests PROPERTIES PASS_REGULAR_EXPRESSION "Test passed")


#####################################
#### google test specific
#### definition of test environment
####

MESSAGE (STATUS "\n\n====== Finding Google Test Framework ======")
INCLUDE (${navlaserplayer_cmake_dir}/internal_utils.cmake)
MESSAGE (STATUS "----------------------------------------\n")

### define gtest libraries, this will be defined in project
set(GTEST_LINK_LIBS gtest gtest_main)

### define include path for gtest

### define SRC files for proect specific unit tests
set(NAVLASER3_CONFIG_GTESTS_SRC ${PROJECT_SOURCE_DIR}/gtest/config/config_extractorGTest.cpp
      ${PROJECT_SOURCE_DIR}/src/config/config_extractor.cpp
      ${PROJECT_SOURCE_DIR}/gtest/config/main.cpp )

### define targets for gtest
ADD_EXECUTABLE(navlaser3-config-gtests ${NAVLASER3_CONFIG_GTESTS_SRC}) 

### define link libraries targets for gtest
TARGET_LINK_LIBRARIES(navlaser3-config-gtests ${GTEST_LINK_LIBS} ${NAVLASER3_LINK_LIBS} )

#####################################
# CTEST DEFINITIONS FOR GTEST
ADD_TEST(test_config_tests_2 "navlaser3-config-gtests")
