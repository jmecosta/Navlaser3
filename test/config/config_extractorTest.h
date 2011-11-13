/*
 * config_extractorTest.h
 *
 *  Created on: 11 Nov 2011
 *      Author: jmecosta
 */

#ifndef CONFIG_EXTRACTORTEST_H_
#define CONFIG_EXTRACTORTEST_H_

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>

#include "config_extractor.h"

class config_extractorTest : public CPPUNIT_NS::TestFixture, public config_extractor
{
    CPPUNIT_TEST_SUITE(config_extractorTest);
    CPPUNIT_TEST(test_config_extractor_Constructors);
    CPPUNIT_TEST(test_setLineEstimationMethod);
    CPPUNIT_TEST(test_setCircleEstimationMethod);
    CPPUNIT_TEST(test_setBreakPointMethod);
    CPPUNIT_TEST(test_setCornerEstimationMethod);
    CPPUNIT_TEST(test_setMinDistanceBtwLines);
    CPPUNIT_TEST(test_setMinLinePoints);
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();
    void tearDown();
    void test_config_extractor_Constructors ();
    void test_setLineEstimationMethod (  );
    void test_setCircleEstimationMethod (  );
    void test_setBreakPointMethod ( );
    void test_setCornerEstimationMethod (  );
    void test_setMinDistanceBtwLines (  );
    void test_setMinLinePoints ( );

private:
   config_extractor * ConfExtractor_1;
};

#endif /* CONFIG_EXTRACTORTEST_H_ */
