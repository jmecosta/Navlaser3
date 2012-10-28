PROJECT(navlaserplayertests)

CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
  CMAKE_POLICY(SET CMP0007 NEW)
  CMAKE_POLICY(SET CMP0011 NEW)
  CMAKE_POLICY(SET CMP0002 NEW)
ENDIF(COMMAND CMAKE_POLICY)

MESSAGE (STATUS "NAVLASEREXECGTESTS: Project Source Dir:" ${PROJECT_SOURCE_DIR} )


#####################################
#### google test specific
#### definition of test environment
####

MESSAGE (STATUS "\n\n====== Finding Google Test Framework ======")
INCLUDE (${navlaserplayer_cmake_dir}/internal_utils.cmake)
MESSAGE (STATUS "----------------------------------------\n")
MESSAGE (STATUS "---------------------dourrr-------------------\n ${NAVLASER3_SRC}\n")

INCLUDE_DIRECTORIES(${PROJECT_INCLUDE_DIRS} ${PROJECT_INCLUDE_EXTERNALS} ${GTEST_INCLUDES})

FIND_PACKAGE(Qt4 REQUIRED) 

set(QT_USE_OPENGL TRUE)
set(QT_USE_QTCORE TRUE)
set(CMAKE_AUTOMOC TRUE)
include(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})


LINK_DIRECTORIES(${GTEST_LINK_DIRS})
LINK_DIRECTORIES(${PLAYER_LINK_DIRS})
LINK_DIRECTORIES(${GAZEBOSERVER_LINK_DIRS})
LINK_DIRECTORIES(${GSL_LINK_DIRS})
LINK_DIRECTORIES(${boost_library_dirs})
LINK_DIRECTORIES(${GTK_LIBRARY_DIRS})
LINK_DIRECTORIES(${OGRE_LIBRARY_DIRS})

### define SRC files for proect specific unit tests
set(NAVLASER3_CONFIG_GTESTS_SRC ${PROJECT_SOURCE_DIR}/config/config_extractorGTest.cpp
      ${NAVLASER3_SRC_TEST}
      ${PROJECT_SOURCE_DIR}/config/main.cpp )

### define targets for gtest
ADD_EXECUTABLE(navlaser3-config-gtests ${NAVLASER3_CONFIG_GTESTS_SRC}) 
                        
### define link libraries targets for gtest
TARGET_LINK_LIBRARIES(navlaser3-config-gtests ${GTEST_LIBS} ${NAVLASER3_LINK_LIBS} )

### set preprocessor flags
SET_TARGET_PROPERTIES(navlaser3-config-gtests PROPERTIES COMPILE_DEFINITIONS "UNIT_GTEST=1")

#####################################
# CTEST DEFINITIONS FOR GTEST
ADD_TEST(test_config_tests_2 "navlaser3-config-gtests")

