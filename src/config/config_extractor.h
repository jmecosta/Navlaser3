/*
 * config_extractor.h
 *
 *  Created on: 27 Mar 2011
 *      Author: jmecosta
 */

#ifndef CONFIG_EXTRACTOR_H_
#define CONFIG_EXTRACTOR_H_

#include "gtest/gtest_prod.h"

#define LINEENDPOINTFIT 0
#define LINELEASTSQUAREMETHOD 1
#define LINESIMPLEREGRESSION 2

#define SIMPLEBREAKMETHOD 0
#define ADPTBREAKMETHOD 1

#define CORNER2DERIVATETEST 0
#define CORNERRAMSAC 1

class config_extractor
{
protected:
    FRIEND_TEST(config_extractorTest, setCornerEstimationMethodReturnBool);
    FRIEND_TEST(config_extractorTest, setLineEstimationMethodReturnBool);
    int MIN_LINE_POINTS; // minimum points a line can contain to be valid
    double RANSAC_FACTOR;
    double ADPTLAMBDA;
    double MIN_DIST_BTW_LINES;

    int CornerEstimationMethod;
    int BreakPointMethod;
    int LineEstimationMethod;
    int CircleEstimationMethod;

    // class properties methods
    bool setLineEstimationMethod ( int method );
    bool setCircleEstimationMethod ( int method );
    bool setBreakPointMethod ( int method, double optional );
    bool setCornerEstimationMethod ( int method, double optional );
    bool setMinDistanceBtwLines ( double distance );
    bool setMinLinePoints (int value);
    
public:
	config_extractor();
	virtual ~config_extractor();



};

#endif /* CONFIG_EXTRACTOR_H_ */
