
#include <cppunit/BriefTestProgressListener.h>
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/TestResult.h>
#include <cppunit/TestResultCollector.h>
#include <cppunit/TestRunner.h>

#include "config_extractorTest.h"

using namespace std;

int main(int argc, char **argv) {

    CPPUNIT_NS::TestSuite suite;
    // Create the event manager and test controller
    CPPUNIT_NS::TestResult controller;
    // Add a listener that colllects test result
    CPPUNIT_NS::TestResultCollector result;
    controller.addListener( &result );

    // add tests
    suite.addTest( config_extractorTest::suite() );

    // run tests
    suite.run(&controller);

    // Print test in a compiler compatible format.
    CPPUNIT_NS::CompilerOutputter outputter( &result, CPPUNIT_NS::stdCOut() );
    outputter.write();

    if ( result.wasSuccessful() == true ) {
        printf ("Test passed\n");
    } else {
        printf ("Test failed\n");
    }
    return result.wasSuccessful() ? 0 : 1;

}
