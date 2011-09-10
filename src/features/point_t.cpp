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

#include "point_t.h"
#include <typeinfo>
#include <iostream>
#include <math.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>

point_t::point_t()
{

	//std::cout << "new point" << std::endl;
	// alloc memory

	//global reference frame
	double x = 0;
	double y = 0;

	// polar coordinates
	double theta = 0;
	double p = 0;

	Maturity = 0;
}
point_t::~point_t()
{

	//std::cout << "delete point" << std::endl;
}

point_t& point_t::operator=(const point_t& other)
{
	//global reference frame
	x = other.x;
	y = other.y;

	// polar coordinates
	theta = other.theta;
	p = other.p;

	//point uncertanty
	Cp[0][0] = other.Cp[0][0];
	Cp[0][1] = other.Cp[0][1];
	Cp[1][0] = other.Cp[1][0];
	Cp[1][1] = other.Cp[1][1];

    GradP[0][0] = other.GradP[0][0];
    GradP[0][1] = other.GradP[0][1];
    GradP[1][0] = other.GradP[1][0];
    GradP[1][1] = other.GradP[1][1];

    GradX[0][0] = other.GradX[0][0];
    GradX[0][1] = other.GradX[0][1];
    GradX[0][2] = other.GradX[0][2];
    GradX[1][0] = other.GradX[1][0];
    GradX[1][1] = other.GradX[1][1];
    GradX[1][2] = other.GradX[1][2];

	PointId = other.PointId;
    ScanIndex = other.ScanIndex; // index of the scan
    Maturity = other.Maturity;
    FeatType = other.Maturity;
    PointMatched = other.PointMatched;


}
bool point_t::operator==(const point_t& other)
{
	printf("TBD\n");
}
void point_t::mergeWithFeature ( feature_t * featIn ) {

	// check if the feature being merged is of the same type
	if (typeid(this).name() != typeid((*featIn)).name() )
	{
		return;
	}
	// down cast the in same type
	point_t * Observed = static_cast<point_t*>(&*featIn);

}
/*
// set the point coordinates
// Xl = Xw - Xp
// Yl = Yw - Yp
// point gradient
// derive point transformation equations in order of robot pose
// [ dXl/dXp dXl/dYp dXl/dTheta_p ] = [1  0 0 ] = GradPose
// [ dYl/dXp dYl/dYp dYl/dTheta_p ]   [ 0 1 0 ]
*/
bool point_t::toGlobalCordinates(gsl_matrix * PoseMean, gsl_matrix * Covariance)
{

	x = -x + gsl_matrix_get(PoseMean, 0, 0); // x coordinate
	y = -y + gsl_matrix_get(PoseMean, 1, 0); // y coordinate

	// RpL = RpW + GradPose * Pose * GradPose_T
	gsl_matrix * GradPose = gsl_matrix_alloc(2, 3);
	gsl_matrix * GradPose_T = gsl_matrix_alloc(3, 2);
	gsl_matrix * mat2x3 = gsl_matrix_alloc(2, 3);
	gsl_matrix * mat2x2 = gsl_matrix_alloc(2, 2);

	gsl_matrix_set(GradPose, 0, 0, 1);
	gsl_matrix_set(GradPose, 0, 1, 0);
	gsl_matrix_set(GradPose, 0, 2, 0);
	gsl_matrix_set(GradPose, 1, 0, 1);
	gsl_matrix_set(GradPose, 1, 1, 1);
	gsl_matrix_set(GradPose, 1, 2, 0);

	gsl_matrix_transpose_memcpy(GradPose_T, GradPose);

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, GradPose, Covariance, 0.0,
			mat2x3);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat2x3, GradPose_T, 0.0,
			mat2x2);

	// UPDATE POINT GRADIENT
	Cp[0][0] = gsl_matrix_get(mat2x2, 0, 0) + Cp[0][0];
	Cp[1][1] = gsl_matrix_get(mat2x2, 1, 1) + Cp[1][1];

    GradX[0][0] = gsl_matrix_get(GradPose,0,0);
    GradX[0][1] = gsl_matrix_get(GradPose,0,1);
    GradX[0][2] = gsl_matrix_get(GradPose,0,2);
    GradX[1][0] = gsl_matrix_get(GradPose,1,0);
    GradX[1][1] = gsl_matrix_get(GradPose,1,1);
    GradX[1][2] = gsl_matrix_get(GradPose,1,2);

	gsl_matrix_free(GradPose_T);
	gsl_matrix_free(GradPose);
	gsl_matrix_free(mat2x3);
	gsl_matrix_free(mat2x2);

}
void point_t::getCovariance ( gsl_matrix * GradOut, int * index_i ) {
	// do some error checking
	if ( *index_i + 2 > (GradOut->size1)  || (GradOut->size2) != 3 )
		return;
	gsl_matrix_set(GradOut,*index_i,*index_i		,Cp[0][0]);
	gsl_matrix_set(GradOut,*index_i,*index_i+1		,Cp[0][1]);
	gsl_matrix_set(GradOut,*index_i+1,*index_i		,Cp[1][0]);
	gsl_matrix_set(GradOut,*index_i+1,*index_i+1	,Cp[1][1]);
	*index_i+=2;
}
void point_t::getCovarianceDiagonalElems ( gsl_matrix * GradOut, int * index_i ) {
	// do some error checking
	if ( *index_i + 2 > (GradOut->size1)  || (GradOut->size2) != 3 )
		return;
	gsl_matrix_set(GradOut,*index_i,*index_i		,Cp[0][0]);
	gsl_matrix_set(GradOut,*index_i+1,*index_i+1	,Cp[1][1]);
	*index_i+=2;
}
void point_t::getPoseGradient(gsl_matrix * GradOut, int * index_i)
{

	// do some error checking
	if (*index_i + 2 > (GradOut->size1) || (GradOut->size2) != 3)
		return;

	gsl_matrix_set(GradOut,*index_i,0	,GradX[0][0]);
	gsl_matrix_set(GradOut,*index_i,1	,GradX[0][1]);
	gsl_matrix_set(GradOut,*index_i,2	,GradX[0][2]);

	// fill nex row
	*index_i++;
	gsl_matrix_set(GradOut,*index_i,0	,GradX[1][0]);
	gsl_matrix_set(GradOut,*index_i,1	,GradX[1][0]);
	gsl_matrix_set(GradOut,*index_i,2	,GradX[1][0]);
	// fill nex row
	*index_i++;

}
bool point_t::toLocalCordinates(gsl_matrix * PoseMean, gsl_matrix * Covariance)
{
	// set the point coordinates
	// Xl = Xw - Xp
	// Yl = Yw - Yp
	x = x - gsl_matrix_get(PoseMean, 0, 0); // x coordinate
	y = y - gsl_matrix_get(PoseMean, 1, 0); // y coordinate

	// point end gradient
	// derive point transformation equations in order of robot pose
	// [ dXl/dXp dXl/dYp dXl/dTheta_p ] = [-1  0 0 ] = GradPose
	// [ dYl/dXp dYl/dYp dYl/dTheta_p ]   [ 0 -1 0 ]
	//
	// RpL = RpW + GradPose * Pose * GradPose_T
	gsl_matrix * GradPose = gsl_matrix_alloc(2, 2);
	gsl_matrix * GradPose_T = gsl_matrix_alloc(2, 2);
	gsl_matrix * mat2x3 = gsl_matrix_alloc(2, 3);
	gsl_matrix * mat2x2 = gsl_matrix_alloc(2, 2);

	gsl_matrix_set(GradPose, 0, 0, -1);
	gsl_matrix_set(GradPose, 1, 1, -1);
	gsl_matrix_transpose_memcpy(GradPose_T, GradPose);

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, GradPose, Covariance, 0.0,
			mat2x3);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat2x3, GradPose_T, 0.0,
			mat2x2);

	Cp[0][0] = gsl_matrix_get(mat2x2, 0, 0) + Cp[0][0];
	Cp[1][1] = gsl_matrix_get(mat2x2, 1, 1) + Cp[1][1];

	gsl_matrix_free(GradPose_T);
	gsl_matrix_free(GradPose);
	gsl_matrix_free(mat2x3);
	gsl_matrix_free(mat2x2);

}

int point_t::setXYCoordinates(double xIn, double yIn, int IdIn)
{
	x = xIn;
	y = yIn;
	ScanIndex = IdIn;

	return 0;
}

double point_t::getX()
{
	return x;
}
double point_t::getY()
{
	return y;
}
double * point_t::getCp()
{
	return &Cp[0][0];
}
bool point_t::isMature()
{
	return (Maturity == MATURITY_LEVEL);
}
void point_t::increaseMaturity()
{
	if (Maturity < MATURITY_LEVEL)
		Maturity++;
}
void point_t::decreaseMaturity()
{
	if (Maturity > 0)
		Maturity--;
}
int point_t::getFeatType() const
{
	return FeatType;
}

int point_t::getScanIndex() const
{
	return ScanIndex;
}

void point_t::setCp(double Cp[2][2])
{
	this->Cp[0][0] = Cp[0][0];
	this->Cp[0][1] = Cp[0][1];
	this->Cp[1][0] = Cp[1][0];
	this->Cp[1][1] = Cp[1][1];
}

void point_t::setFeatType(int FeatType)
{
	this->FeatType = FeatType;
}

void point_t::setMaturity(int Maturity)
{
	this->Maturity = Maturity;
}

void point_t::setP(double p)
{
	this->p = p;
}

void point_t::setScanIndex(int ScanIndex)
{
	this->ScanIndex = ScanIndex;
}

void point_t::setTheta(double theta)
{
	this->theta = theta;
}

void point_t::setX(double x)
{
	this->x = x;
}

void point_t::setY(double y)
{
	this->y = y;
}

double point_t::getTheta()
{
	return theta;
}
double point_t::getP()
{
	return p;
}

