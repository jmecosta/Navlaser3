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

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/thirdparty/share/cmake/Modules CACHE PATH "module path")
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/thirdparty/share/cmake-2.8/Modules CACHE PATH "module path")


MESSAGE (STATUS "\n\n====== Finding 3rd Party Packages ======")
INCLUDE (${navlaserplayer_cmake_dir}/SearchForStuff.cmake)

MESSAGE (STATUS "----------------------------------------\n")

# set thirdparty stuff
set(PLAYER_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/include/player-3.0 CACHE PATH "Includes to be used in projec")
set(GTK_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty 
                     ${PROJECT_SOURCE_DIR}/thirdparty/include/gtk-2.0
                     ${PROJECT_SOURCE_DIR}/thirdparty/include/gtk-2.0/gdk
                     ${PROJECT_SOURCE_DIR}/thirdparty/include/gtk-2.0/gtk
                     ${PROJECT_SOURCE_DIR}/thirdparty/glib
                     ${PROJECT_SOURCE_DIR}/thirdparty/cairo
                     ${PROJECT_SOURCE_DIR}/thirdparty/pango
                     CACHE PATH "Includes to be used in projec")

set(OGRE_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/ogre/include/OGRE
                     CACHE PATH "Includes to be used in projec")

set (GSL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/include CACHE INTERNAL "gsl include directory")
set (GSL_LINK_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/lib CACHE INTERNAL "gsl link directory")
set (GSL_LINK_LIBS -lgsl -lgslcblas CACHE INTERNAL "gsl libraries")

SET (Boost_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/include CACHE INTERNAL "boost libraries")
SET (boost_library_dirs ${PROJECT_SOURCE_DIR}/thirdparty/lib CACHE INTERNAL "boost libraries")
SET (boost_libraries -lboost_thread CACHE INTERNAL "boost libraries")

SET (QT_QTCORE_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/thirdparty/include/qt4 
                           ${PROJECT_SOURCE_DIR}/thirdparty/include/qt4/Qt
                           ${PROJECT_SOURCE_DIR}/thirdparty/include/qt4/QtOpenGL
                           ${PROJECT_SOURCE_DIR}/thirdparty/include/qt4/QtGui
    CACHE INTERNAL "qt libraries")

SET(QT_QTCORE_dirs ${PROJECT_SOURCE_DIR}/thirdparty/lib CACHE INTERNAL "qt libraries")
SET(QT_LIBRARIES -lQtCore -lQtOpenGL -lQtGui CACHE INTERNAL "qt libraries")


# set libraries from third party
set(PLAYERC++_LINK_LIBS -lplayerc++ -lplayerc -lz -lm   CACHE PATH "Libraries to be used in project")
set(GTK_LIBRARIES -lgtk-x11-2.0 CACHE PATH "Libraries to be used in project")
set(OGRE_LIBRARIES -lOgreMain  -lpthread  CACHE PATH "Libraries to be used in project")

#set link directories
set(PLAYER_LINK_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/lib 
                     ${PROJECT_SOURCE_DIR}/thirdparty/lib64
                     CACHE PATH "Includes to be used in project")

set(GTK_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/lib
                     CACHE PATH "Includes to be used in project")

#LINK_DIRECTORIES(${GAZEBOSERVER_LINK_DIRS})
#LINK_DIRECTORIES(${GSL_LINK_DIRS})
#LINK_DIRECTORIES(${boost_library_dirs})
set(OGRE_LIBRARY_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/ogre/lib 
                     CACHE PATH "Includes to be used in project")


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

#SET(CMAKE_CXX_LINK_FLAGS " -ldl" CACHE INTERNAL "Link flags for profile" FORCE)
#SET(CMAKE_C_FLAGS "-g -O0 -Wall" CACHE INTERNAL "Link flags for profile" FORCE)
#SET(CMAKE_CXX_FLAGS "-DQT_NO_DEBUG -DQT_GUI_LIB -DQT_CORE_LIB" CACHE INTERNAL "Link flags for profile" FORCE)

# Unit Test
INCLUDE(CTest)
IF(BUILD_TESTING)

    #set test directories
    set(GTEST_INCLUDES ${PROJECT_SOURCE_DIR}/thirdparty/include
                     CACHE PATH "Includes to be used in project")

    set(GTEST_LINK_DIRS ${PROJECT_SOURCE_DIR}/thirdparty/lib
                     CACHE PATH "Includes to be used in project")

    set(GTEST_LIBS ${PROJECT_SOURCE_DIR}/thirdparty/lib/libgtest.a
                   ${PROJECT_SOURCE_DIR}/thirdparty/lib/libgmock.a
                     CACHE PATH "Includes to be used in project")


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


ENDIF(BUILD_TESTING)


