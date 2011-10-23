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

#ifndef LINE_T_H
#define LINE_T_H

#include "feature_c.h"
#include "defs.h"

class line_t : public feature_t
{
private:
    int LineId;
    int FeatType;
    int Maturity;
    bool LineMatched;

    // line properties
    double StartPointXY[2];
    double StartPointAlphaR[2]; // alpha, r
    int	StartPointIndex;
    double EndPointXY[2];
    double EndPointAlphaR[2];
    int	EndPointIndex;
    
    // end point attributes
    int StartPointAttr;
    int EndPointAttr;    
    
    // polar coordinates representation of the line and its uncertanty
    double r;	// line distance to ref frame
    double alpha; // line angle
    double CLR[2][2];

    double GradL[2][2];
    double GradX[2][3];

    // statistical methods
    void ToLocalgetGradient ( gsl_matrix * GradOut, int * index_i );
    void ToGlobalgetGradient ( gsl_matrix * GradOut, int * index_i );

public:
    line_t ();
    virtual ~line_t();
    
    // methods for implementation of abastrac class
    line_t& operator=(const line_t& other);
    bool operator==(const line_t& other);
    bool toGlobalCordinates ( gsl_matrix * PoseMean, gsl_matrix * PoseCovariance );
    bool toLocalCordinates ( gsl_matrix * PoseMean, gsl_matrix * PoseCovariance );

    // statistical methods
    void getPoseGradient ( gsl_matrix * GradOut, int * index_i );
    void getCovariance ( gsl_matrix * GradOut, int * index_i );
    void getCovarianceDiagonalElems ( gsl_matrix * GradOut, int * index_i );

    // merge methods
    void mergeWithFeature ( feature_t * featIn );

    // methods to check the maturity of the line
    bool isMature ();
    void increaseMaturity ( );
    void decreaseMaturity ( );
    bool isMatch () { return LineMatched;};
    void setMatchFlag ( bool Match ) { LineMatched = Match; };

    bool setStartPoint(double X, double Y, double bearing, double r, int index);
    bool setEndPoint(double X, double Y, double bearing, double r, int index);
    bool setLineParametersGSL ( double r, double alpha, gsl_matrix * CLR );
    bool setLineParameters ( double r, double alpha, double CL[2][2] );
    bool setEndPointAttr ( int endAttr );
    bool setStartPointAttr ( int startAttr );
    void setAlpha(double alpha);
    void setCLR(double CLR[2][2]);
    void setFeatType(int FeatType);
    void setLineId(int LineId);
    void setMaturity( int Maturity );
    void setR(double r);
    void setReferencial(int referencial);

    // get properties of the line
    double getLineAlpha ();
    double getLineR ();
    int getEndPointAttr ();
    int getStartPointAttr ();
    double getStartPointX () {return StartPointXY[0]; };
    double getStartPointY () {return StartPointXY[1]; };
    double getStartPointBearing () {return StartPointAlphaR[0]; };
    double getStartPointR () {return StartPointAlphaR[1]; };
    int getStartPointId () {return StartPointIndex; };
    double getEndPointX () {return EndPointXY[0]; };
    double getEndPointY () {return EndPointXY[1]; };
    double getEndPointBearing () {return EndPointAlphaR[0]; };
    double getEndPointR () {return EndPointAlphaR[1]; };
    int getEndPointId () {return EndPointIndex; };
    int getMaturity () { return Maturity; };
    int getFeatureType ()  { return FeatType; };
    void getLineCovariance ( gsl_matrix * OutMatrix );
    double getAlpha() const;
    double * getCLR();
    int getFeatType() const;
    int getLineId() const;
    double getR() const;

    int getId () {return LineId;};
    void setId (int id) {LineId = id;};
};

#endif // LINE_T_H
