
#include <cppunit/BriefTestProgressListener.h>
#include <cppunit/CompilerOutputter.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/TestResult.h>
#include <cppunit/TestResultCollector.h>
#include <cppunit/TestRunner.h>
#include <cppunit/XmlOutputter.h>
#include <cppunit/XmlOutputterHook.h>


#include "config_extractorTest.h"
#include "myxmloutputterhook.h"

using namespace std;

int main(int argc, char **argv) {
    
    CPPUNIT_NS::TestSuite suite;
    CPPUNIT_NS::TestResult controller;

    // Add a listener that colllects test result
    CPPUNIT_NS::TestResultCollector result;
    controller.addListener( &result );

    // SETUP XML WRITE RESULTS
    std::ofstream outputFile("testResults.xml");
    CPPUNIT_NS::XmlOutputter* outputter = new CppUnit::XmlOutputter( &result,
            outputFile );
    MyXmlOutputterHook hook("NavLaser3 - config_extractor", "J. Costa");
    outputter->addHook(&hook);

    // ADD AND RUN TESTS
    suite.addTest( config_extractorTest::suite() );
    suite.run(&controller);

    // WRITE RESULTS
    outputter->write();
    outputFile.close();

    if ( result.wasSuccessful() == true ) {
        printf ("Test passed\n");
    } else {
        printf ("Test failed\n");
    }
    return result.wasSuccessful() ? 0 : 1;

}
