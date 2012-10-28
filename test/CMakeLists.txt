PROJECT(navlaserplayertests)

CMAKE_MINIMUM_REQUIRED( VERSION 2.4.6 FATAL_ERROR )

IF(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
  CMAKE_POLICY(SET CMP0007 NEW)
  CMAKE_POLICY(SET CMP0011 NEW)
  CMAKE_POLICY(SET CMP0002 NEW)
ENDIF(COMMAND CMAKE_POLICY)

MESSAGE (STATUS "NAVLASEREXECTESTS: Project Source Dir:" ${PROJECT_SOURCE_DIR} )

#####################################
#### unit test specific
#### definition of test environment
####
INCLUDE_DIRECTORIES(${PROJECT_ROOT_DIR}/src/config)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/common)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/config)
INCLUDE_DIRECTORIES(${CPPUNIT_INCLUDE_DIRS})

#### define SRC files for project specific unit tests
set(NAVLASER3_CONFIG_TESTS_SRC 	${PROJECT_SOURCE_DIR}/common/myxmloutputterhook.cpp
        						${PROJECT_SOURCE_DIR}/config/config_extractorTest.cpp
        						${PROJECT_SOURCE_DIR}/config/main.cpp         
        						${PROJECT_ROOT_DIR}/src/config/config_extractor.cpp )

#### define project executable for config testing
ADD_EXECUTABLE(navlaser3-config-tests ${NAVLASER3_CONFIG_TESTS_SRC})

#### add link libraries for unit testing
TARGET_LINK_LIBRARIES(navlaser3-config-tests    ${NAVLASER3_LINK_LIBS}
                                                ${CPPUNIT_LIBRARIES} )

message (STATUS "CURRENT link libraries: ${NAVLASER3_LINK_LIBS}")

# CTEST DEFINITIONS
ADD_TEST(test_config_tests "navlaser3-config-tests")
SET_TESTS_PROPERTIES(test_config_tests PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR;FAIL;Test failed")
SET_TESTS_PROPERTIES(test_config_tests PROPERTIES PASS_REGULAR_EXPRESSION "Test passed")
#ADD_TEST(test_config_tests_1 "navlaser3-config-tests")
#SET_TESTS_PROPERTIES(test_config_tests PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR;FAIL;Test failed")
#SET_TESTS_PROPERTIES(test_config_tests PROPERTIES PASS_REGULAR_EXPRESSION "Test passed")


