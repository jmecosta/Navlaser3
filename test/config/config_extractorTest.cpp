/*
 * config_extractor.cpp
 *
 *  Created on: 27 Mar 2011
 *      Author: jmecosta
 */

#include "config_extractorTest.h"

//CPPUNIT_TEST_SUITE_REGISTRATION( config_extractorTest );

void config_extractorTest::test_config_extractor_Constructors() {
    CPPUNIT_ASSERT( 0 == 0);
}
void config_extractorTest::test_setLineEstimationMethod (  ) {
    CPPUNIT_ASSERT( setLineEstimationMethod(0) == true);
    CPPUNIT_ASSERT( LineEstimationMethod == 0);
}

void config_extractorTest::test_setCircleEstimationMethod (  ) {

}

void config_extractorTest::test_setBreakPointMethod ( ) {
}
void config_extractorTest::test_setCornerEstimationMethod (  ) {
    CPPUNIT_ASSERT (setCornerEstimationMethod(CORNERRAMSAC, 0.1) == true );
    CPPUNIT_ASSERT (CornerEstimationMethod == CORNERRAMSAC );
    CPPUNIT_ASSERT (RANSAC_FACTOR == 0.1 );

    CPPUNIT_ASSERT (setCornerEstimationMethod(0, 0.1) == true );
    CPPUNIT_ASSERT (CornerEstimationMethod == 0 );
    CPPUNIT_ASSERT (RANSAC_FACTOR == 0.1 );
}
void config_extractorTest::test_setMinDistanceBtwLines (  ){
}
void config_extractorTest::test_setMinLinePoints ( ) {
}



void config_extractorTest::setUp() {
    ConfExtractor_1 = new config_extractor ();
}
void config_extractorTest::tearDown() {
    delete ConfExtractor_1;
}
