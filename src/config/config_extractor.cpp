/*
 * config_extractor.cpp
 *
 *  Created on: 27 Mar 2011
 *      Author: jmecosta
 */

#include "config_extractor.h"

config_extractor::config_extractor()
{
	// TODO Auto-generated constructor stub

}

config_extractor::~config_extractor()
{
	// TODO Auto-generated destructor stub
}

bool config_extractor::setLineEstimationMethod(int method)
{
	LineEstimationMethod = method;
	return true;
}
bool config_extractor::setBreakPointMethod(int method, double optional)
{
	BreakPointMethod = method;
	if (method == ADPTBREAKMETHOD)
		ADPTLAMBDA = optional;
	return true;
}
bool config_extractor::setMinLinePoints(int value)
{
	this->MIN_LINE_POINTS = value;
}
bool config_extractor::setCircleEstimationMethod(int method)
{
	CircleEstimationMethod = method;
	return true;
}
bool config_extractor::setCornerEstimationMethod(int method,
		double optional)
{
	CornerEstimationMethod = method;

	if (method == CORNERRAMSAC)
		RANSAC_FACTOR = optional;

	return true;
}
bool config_extractor::setMinDistanceBtwLines(double distance)
{
	MIN_DIST_BTW_LINES = distance;
	return true;
}

