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

#ifndef POINT_T_H
#define POINT_T_H

#include <gsl/gsl_matrix.h>

#include "feature_c.h"

#define NOT_FINAL_POINT 0
#define FINAL_POINT 1

class point_t : public feature_t
{
  
private:
	int PointId;
    // line identifir
    int ScanIndex; // index of the scan
    int Maturity;
    int FeatType;
    bool PointMatched;
      
    //global reference frame  
    double x;
    double y;

    // polar coordinates
    double theta;
    double p;
    
    //point uncertanty
    double Cp[2][2];

    double GradP[2][2];
    double GradX[2][3];

public:
    point_t ();
    virtual ~point_t ();
	
	// methods for implementation of abastrac class
	point_t& operator=(const point_t& other);
	bool operator==(const point_t& other);		
	bool toGlobalCordinates ( gsl_matrix * PoseMean, gsl_matrix * Covariance );
	bool toLocalCordinates ( gsl_matrix * PoseMean, gsl_matrix * Covariance ); 
    
    bool isMature ();
    void increaseMaturity ( );
    void decreaseMaturity ( );

    // statistical methods
    void getPoseGradient ( gsl_matrix * GradOut, int * index_i );
    void getCovariance ( gsl_matrix * GradOut, int * index_i );
    void getCovarianceDiagonalElems ( gsl_matrix * GradOut, int * index_i );

    // merge methods
    void mergeWithFeature ( feature_t * featIn );
    bool isMatch () { return PointMatched;};
    void setMatchFlag ( bool Match ) { PointMatched = Match; };

	// set methods
    int setXYCoordinates ( double xIn, double yIn, int Id );
    void setCp(double Cp[2][2]);
    void setFeatType(int FeatType);
    void setMaturity(int Maturity);
    void setP(double p);
    void setScanIndex(int ScanIndex);
    void setTheta(double theta);
    void setX(double x);
    void setY(double y);


	// get methods
    double getTheta ();
    double getP ();    
    double getX ();
    double getY ();
    double * getCp();
    int getFeatType() const;
    int getScanIndex() const;
    int getMaturity () { return Maturity; };
    int getFeatureType ()  { return FeatType; };

    int getId () {return PointId;};
    void setId (int id) {PointId = id;};
};

#endif // POINT_T_H
