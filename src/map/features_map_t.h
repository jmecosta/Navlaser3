/*
 Copyright (c) <year>, <copyright holder>
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef FEATURES_MAP_T_H
#define FEATURES_MAP_T_H

#include <list>
#include <boost/ptr_container/ptr_list.hpp>
#include <boost/shared_ptr.hpp>

#include "feature_c.h"
#include "point_t.h"
#include "line_t.h"
#include "features_extractor_t.h"
#include "config_map.h"

using namespace std;

struct MatchedFeatures_C
{
	feature_t* Observed;
	feature_t* InMapFeat;
	vector<double> InovVect;
};
struct LinesPair_C
{
	line_t Observed;
	line_t InMapLine;
	double InovVect[2];
};
struct FeaturesPair_C
{
	feature_t* Observed;
	feature_t* InMapFeat;
	vector<double> Inovation;
};

class features_map_t: public features_extractor_t, public config_map
{
protected:

	// the map of features
	boost::ptr_list<feature_t>  MapofFeatures;

	// matrixes for matching
	gsl_matrix * Qj; // 2x2
	gsl_matrix * gL; // 2x3
	gsl_matrix * gLT; // 3x2
	gsl_matrix * glTemp; // 2x3
	gsl_matrix * Sij; // 2x2
	gsl_matrix * InvSij; // 2x2
	gsl_matrix * Vij; // 1x2
	gsl_matrix * VijT; // 2x1
	gsl_matrix * VijInvSij; // 1x2
	gsl_matrix * Gres; // 1x1

	// methods to check if two features are the same
	bool MahalanobisCheck(FeaturesPair_C TestPair, gsl_matrix * cov);
	bool MahalanobisCheck(LinesPair_C * TestPair, gsl_matrix * cov);
	bool PointInTheMiddleCheck(double, double, double, double, double, double);
	bool LineEndPointsCheck (FeaturesPair_C TestPair, gsl_matrix * cov);

public:
	features_map_t();
	bool AddFeatureToMap (feature_t * FeatureToAdd );
	list<feature_t*> ListGetFeatureFromMap();
	void RemoveFalsePositives ();
	void OptimizeListOfFeatures ();
	bool FeaturesPairMatchCheck(FeaturesPair_C TestPair, gsl_matrix * cov);
	void printFeatures (string id);

};

#endif // FEATURES_MAP_T_H
