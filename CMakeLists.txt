PROJECT(NAVLASER3_ROOT)
CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
  CMAKE_POLICY(SET CMP0007 NEW)
  CMAKE_POLICY(SET CMP0011 NEW)
  CMAKE_POLICY(SET CMP0002 NEW)
ENDIF(COMMAND CMAKE_POLICY)

# product options
OPTION(BUILD_WITH_COVERAGE "Build with coverage analysis support" OFF)

MESSAGE (STATUS "\n\n====== PROJECT PATH: ${PROJECT_SOURCE_DIR} ======")
set (navlaserplayer_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH "Location of CMake scripts")
set (navlaserplayer_lib_dir ${PROJECT_SOURCE_DIR}/lib CACHE PATH "Location of CMake scripts")
set (PROJECT_ROOT_DIR ${PROJECT_SOURCE_DIR} CACHE PATH "Root location of navlaser3")

MESSAGE (STATUS "\n\n====== Finding 3rd Party Packages ======")
INCLUDE (${navlaserplayer_cmake_dir}/SearchForStuff.cmake)
MESSAGE (STATUS "----------------------------------------\n")

set(NAVLASER3_LINK_LIBS ${PLAYER_LINK_LIBS}
                        ${PLAYERC++_LINK_LIBS}
                        ${GAZEBOSERVER_LINK_LIBS}
                        ${GSL_LINK_LIBS}
                        ${boost_libraries}
                        ${GTK_LIBRARIES}
                        ${CPPUNIT_LIBRARIES}
                        ${OGRE_LIBRARIES}
                        ${QT_LIBRARIES}
                        ${QT_QTOPENGL_LIBRARY}  CACHE PATH "Libraries to be used in project")

# add sources for project
ADD_SUBDIRECTORY(src)

# set install prefix
SET(CMAKE_INSTALL_PREFIX "/home/jorge/PLAYERSTAGE/")

SET(CMAKE_CXX_LINK_FLAGS " -ldl" CACHE INTERNAL "Link flags for profile" FORCE)
SET(CMAKE_C_FLAGS "-g -O0 -Wall")
SET(CMAKE_CXX_FLAGS "-g -O0 -Wall")

# Unit Test
INCLUDE(CTest)
IF(BUILD_TESTING)

	IF(BUILD_WITH_COVERAGE)
	    ADD_CUSTOM_COMMAND(TARGET helloworld PRE_BUILD
        	COMMAND ${CMAKE_SOURCE_DIR}/../scripts/test4ccache ${CMAKE_C_COMPILER}
        	COMMENT "Testing for ccache"
        	)
    	# enable coverage analysis using GCC and GCOV
    	IF (CMAKE_COMPILER_IS_GNUCC)
        	SET(CMAKE_C_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
        	SET(CMAKE_CXX_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
        	SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
    	ENDIF()

	# add lcov target
	INCLUDE (${navlaserplayer_cmake_dir}/lcov.cmake)
	SET(COVERAGE_COMMAND "/usr/bin/gcov")
	add_definitions(--coverage)
        
	ENDIF(BUILD_WITH_COVERAGE)
	
	#####################################
	# CTEST ENABLED
	ENABLE_TESTING()
	#####################################

    ADD_SUBDIRECTORY(gtest)	
    ADD_SUBDIRECTORY(test)


ENDIF(BUILD_TESTING)


