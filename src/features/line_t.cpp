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

#include "line_t.h"
#include "math_lib.h"

#include <gsl/gsl_blas.h>
#include <gsl/gsl_matrix.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <typeinfo>


using namespace std;

line_t::line_t()
{

	//cout << "construct line" << endl;
	FeatType = LINE_FEATURE;
	Maturity = 0;
	// other stuff
	LineId = 0;
	LineMatched = 0;

	//line uncertanty
	CLR[0][0] = 0;
	CLR[0][1] = 0;
	CLR[1][0] = 0;
	CLR[1][1] = 0;

	GradL[0][0] = 0;
	GradL[0][1] = 0;
	GradL[1][0] = 0;
	GradL[1][1] = 0;

	GradX[0][0] = 0;
	GradX[0][1] = 0;
	GradX[0][2] = 0;
	GradX[1][0] = 0;
	GradX[1][1] = 0;
	GradX[1][2] = 0;

}
line_t::~line_t()
{
	//cout << "delete line" << endl;
}
line_t& line_t::operator=(const line_t& other)
{
	// line properties
	this->StartPointAlphaR[0] = other.StartPointAlphaR[0];
	this->StartPointAlphaR[1] = other.StartPointAlphaR[1];
	this->StartPointXY[0] = other.StartPointXY[0];
	this->StartPointXY[1] = other.StartPointXY[1];

	// end point attributes
	StartPointAttr = other.StartPointAttr;
	EndPointAttr = other.EndPointAttr;

	// polar coordinates representation of the line and its uncertanty
	r = other.r; // line distance to ref frame
	alpha = other.alpha; // line angle

	//line uncertanty
	CLR[0][0] = other.CLR[0][0];
	CLR[0][1] = other.CLR[0][1];
	CLR[1][0] = other.CLR[1][0];
	CLR[1][1] = other.CLR[1][1];

	GradL[0][0] = other.GradL[0][0];
	GradL[0][1] = other.GradL[0][1];
	GradL[1][0] = other.GradL[1][0];
	GradL[1][1] = other.GradL[1][1];

	GradX[0][0] = other.GradX[0][0];
	GradX[0][1] = other.GradX[0][1];
	GradX[0][2] = other.GradX[0][2];
	GradX[1][0] = other.GradX[1][0];
	GradX[1][1] = other.GradX[1][1];
	GradX[1][2] = other.GradX[1][2];

	// other stuff
	Maturity = other.Maturity;
	LineId = other.LineId;
	LineMatched = other.LineMatched;

}
bool line_t::operator==(const line_t& other)
												{
	printf("TBD\n");

												}
/* as per (4.6),
 * 	g^⁻1 transformation to world model
	g^-1 =  [αlw] = [αli + θi]
    		[rlw]   [rli + δrli]

    	δrli = xi cos(αli + θi ) + yi sin(αli + θi )		(4.7)

    	θi - robot angular pose value
    	xi - robot x pose value
    	yi - robot y pose value
    	αli - polar angle alpha in robot frame
    	αlw - polar angle alpha in world frame
    	rlw - polar phi in world frame

	  	A line covariance matrix can be propagated into a global frame, using the following
	  	equations:

	  	QLw = (GLi^-1).QLr.(GLi^-1)T + (Gxi^-1).Ri.(Gxi^-1)T (4.20)

			Ri - pose covariance
			QLr - line covariance in the local frame
			QLw - line covariance in the world frame
			GLi^-1, Gx^-1 - are the linearization matrix wrt to line and pose variables

		The lineariation matrixs are calculated using the following eq:

	  	  Gx^-1  =  [ 0 				0 				1   ] 		 (4.19)
	  		 		[ cos(αli + θi) 	sin(αli + θi)	δψi ]


	  	  GLi^−1 =  [ 1    0 ]		(4.14)
	  	  	  	  	[ δψi  1 ]

	  	  	  	  	where δψi = yi cos(αli + θi ) − xi sin(αli + θi )	  (4.15)

 */
bool line_t::toGlobalCordinates(gsl_matrix * Mean, gsl_matrix * Cov)
{
	double rli = 0;
	double Srli = 0;
	double Swi = 0;

	double x = gsl_matrix_get(Mean, 0, 0);
	double y = gsl_matrix_get(Mean, 1, 0);
	double th = gsl_matrix_get(Mean, 2, 0);

	// private matrix
	gsl_matrix * GradGLi = gsl_matrix_alloc(2, 2);
	gsl_matrix * GradGLiT = gsl_matrix_alloc(2, 2);
	gsl_matrix * QLi = gsl_matrix_alloc(2, 2);
	gsl_matrix * QgL = gsl_matrix_alloc(2, 2);

	gsl_matrix * GradGx = gsl_matrix_alloc(2, 3);
	gsl_matrix * GradGxT = gsl_matrix_alloc(3, 2);
	gsl_matrix * Qgx = gsl_matrix_alloc(2, 2);

	// temp matrix
	gsl_matrix * mat2x2 = gsl_matrix_alloc(2, 2);
	gsl_matrix * mat2x3 = gsl_matrix_alloc(2, 3);

	// As per section 4.1.1 of final  test report
	// estimate first δrli = xi cos(αli + θi ) + yi sin(αli + θi ) (4.7)
	Srli = x * cos(alpha + th) + y * sin(alpha + th );

	// estimate first δψi = yi cos(αli + θi ) − xi sin(αli + θi ) (4.15)
	Swi = y * cos(alpha + th) - x * sin(alpha + th);

	// Propagate the covariance of the pose to the line to the world reference using
	// QgL = rgLi −1QLi(rgLi−1)T (4.13)
	gsl_matrix_set(QLi, 0, 0, CLR[0][0]);
	gsl_matrix_set(QLi, 0, 1, CLR[0][1]);
	gsl_matrix_set(QLi, 1, 0, CLR[1][0]);
	gsl_matrix_set(QLi, 1, 1, CLR[1][1]);

	// (4.14)
	gsl_matrix_set(GradGLi, 0, 0, 1);
	gsl_matrix_set(GradGLi, 0, 1, 0);
	gsl_matrix_set(GradGLi, 1, 0, Swi);
	gsl_matrix_set(GradGLi, 1, 1, 1);

	// (4.19)
	gsl_matrix_set(GradGx,	0,	0	,0);
	gsl_matrix_set(GradGx,	0,	1	,0);
	gsl_matrix_set(GradGx,	0,	2	,1);
	gsl_matrix_set(GradGx,	1,	0	,cos(alpha + th));
	gsl_matrix_set(GradGx,	1,	1	,sin(alpha + th));
	gsl_matrix_set(GradGx,	1,	2	,Swi);

	// transpose the matrix GradGLi
	gsl_matrix_transpose_memcpy(GradGLiT, GradGLi);
	// transpose the matrix GradGx
	gsl_matrix_transpose_memcpy(GradGxT, GradGx);

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, GradGLi, QLi, 0.0, mat2x2); // (4.16)
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat2x2, GradGLiT, 0.0, QgL); // (4.16)

	// (4.17) Qgx = rg−1x Ri(rg−1x )T
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, GradGx, Cov, 0.0, mat2x3);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat2x3, GradGxT, 0.0, Qgx);

	// TRANSFORM ALPHA AND R
	alpha = alpha + th;
	r = r + Srli;

	// (4.20) QL! = rg−1Li QgL(rg−1Li )T + (rgx)−1Ri(rg−1x )T
	CLR[0][0] = gsl_matrix_get(QgL,0,0) + gsl_matrix_get(Qgx,0,0);
	CLR[0][1] = gsl_matrix_get(QgL,0,1) + gsl_matrix_get(Qgx,0,1);
	CLR[1][0] = gsl_matrix_get(QgL,1,0) + gsl_matrix_get(Qgx,1,0);
	CLR[1][1] = gsl_matrix_get(QgL,1,1) + gsl_matrix_get(Qgx,1,1);

	GradL[0][0] = gsl_matrix_get(GradGLi,0,0);
	GradL[0][1] = gsl_matrix_get(GradGLi,0,1);
	GradL[1][0] = gsl_matrix_get(GradGLi,1,0);
	GradL[1][1] = gsl_matrix_get(GradGLi,1,1);

	GradX[0][0] = gsl_matrix_get(GradGx,0,0);
	GradX[0][1] = gsl_matrix_get(GradGx,0,1);
	GradX[0][2] = gsl_matrix_get(GradGx,0,2);
	GradX[1][0] = gsl_matrix_get(GradGx,1,0);
	GradX[1][1] = gsl_matrix_get(GradGx,1,1);
	GradX[1][2] = gsl_matrix_get(GradGx,1,2);

	// do the same thing for the end point
	StartPointXY[0] = -StartPointXY[0] + gsl_matrix_get(Mean, 0, 0); // x start coordinate
	StartPointXY[1] = -StartPointXY[1] + gsl_matrix_get(Mean, 1, 0); // y start coordinate
	EndPointXY[0] = -EndPointXY[0] + gsl_matrix_get(Mean, 0, 0); // x end coordinate
	EndPointXY[1] = -EndPointXY[1] + gsl_matrix_get(Mean, 1, 0); // y end coordinate

	// dealocation
	gsl_matrix_free(GradGLi);
	gsl_matrix_free(GradGLiT);
	gsl_matrix_free(QLi);
	gsl_matrix_free(QgL);
	gsl_matrix_free(GradGx);
	gsl_matrix_free(GradGxT);
	gsl_matrix_free(Qgx);
	gsl_matrix_free(mat2x2);
	gsl_matrix_free(mat2x3);
}
/** as per (4.5)
 *  g, transformation to local frame
 *
	g = [αli] = [αlω − θi]
    	[rli]   [rlω − δrli]

    	δrli = xi cos(αli + θi ) + yi sin(αli + θi ) 	(4.7)

    	θi - robot angular pose value
    	xi - robot x pose value
    	yi - robot y pose value
    	αli - polar angle alpha in robot frame
    	αlw - polar angle alpha in world frame
    	rlw - polar phi in world frame

	  	A line covariance matrix can be propagated into a local frame, using the following
	  	equations:

	  	QLr = GLi.QLw.GLi^T + Gxi.Ri.GxiT (4.20)

			Ri - pose covariance
			QLr - line covariance in the local frame
			QLw - line covariance in the world frame
			GLi, Gx - are the linearization matrix wrt to line and pose variables

	  	The linearization matrix are as follow:

	  	  Gx  =  [ 0 				0 		-1 ] 		 			(5.3)
	  		 	 [ -cos(αlω) 	-sin(αlω)	0 ]

	  	  GLi	 =  [ 1    0 ]		(4.14)
	  	  	  	  	[ -δψi  1 ]

					// to be calc
	  	  	  	  	where δψi = yi cos( αlω ) - xi sin( αlω )	  (4.15)

 **/
bool line_t::toLocalCordinates(gsl_matrix * Mean, gsl_matrix * Ri)
{
	double x = gsl_matrix_get(Mean,0,0);
	double y = gsl_matrix_get(Mean,1,0);
	double th = gsl_matrix_get(Mean,2,0);

	double Srli = x*cos(alpha + th) + y*sin(alpha + th); // (4.7)
	double Swi = y * cos(alpha) - x * sin(alpha);

	// private matrix
	gsl_matrix * GradGLi = gsl_matrix_alloc(2, 2);
	gsl_matrix * GradGLiT = gsl_matrix_alloc(2, 2);
	gsl_matrix * GradGx = gsl_matrix_alloc(2, 3);
	gsl_matrix * GradGxT = gsl_matrix_alloc(3, 2);

	gsl_matrix * QLi = gsl_matrix_alloc(2, 2);
	gsl_matrix * QGL = gsl_matrix_alloc(2, 2);
	gsl_matrix * QLw = gsl_matrix_alloc(2, 2);

	// temp matrix
	gsl_matrix * mat2x2 = gsl_matrix_alloc(2, 2);
	gsl_matrix * mat2x3 = gsl_matrix_alloc(2, 3);

	/* GLi */
	gsl_matrix_set(GradGLi,0,0,	1);
	gsl_matrix_set(GradGLi,0,1,	0);
	gsl_matrix_set(GradGLi,1,0,	-Swi);
	gsl_matrix_set(GradGLi,1,1,	1);
	/* Gxi */
	// (4.19)
	gsl_matrix_set(GradGxT,0	,0	,0);
	gsl_matrix_set(GradGxT,0	,1	,0);
	gsl_matrix_set(GradGxT,0	,2	,-1);
	gsl_matrix_set(GradGxT,1	,0	,-cos(alpha));
	gsl_matrix_set(GradGxT,1	,1	,-sin(alpha));
	gsl_matrix_set(GradGxT,1	,2	,0);
	/* QLw */
	gsl_matrix_set(QLw,0,0,CLR[0][0]);
	gsl_matrix_set(QLw,0,1,CLR[0][1]);
	gsl_matrix_set(QLw,1,0,CLR[1][0]);
	gsl_matrix_set(QLw,1,1,CLR[1][1]);

	gsl_matrix_transpose_memcpy(GradGLiT,GradGLi);
	gsl_matrix_transpose_memcpy(GradGxT,GradGx);

	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
			1.0, GradGx, Ri,
			0.0, mat2x3);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
			1.0, mat2x3, GradGxT,
			0.0, QGL);

	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
			1.0, GradGLi, QLw,
			0.0, mat2x2);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
			1.0, mat2x2, GradGLiT,
			0.0, QLi);

	/* SET LINEARIZATION MATRIX FOR THIS OPERATION*/
	GradL[0][0] = gsl_matrix_get(GradGLi,0,0);
	GradL[0][1] = gsl_matrix_get(GradGLi,0,1);
	GradL[1][0] = gsl_matrix_get(GradGLi,1,0);
	GradL[1][1] = gsl_matrix_get(GradGLi,1,1);

	GradX[0][0] = gsl_matrix_get(GradGx,0,0);
	GradX[0][1] = gsl_matrix_get(GradGx,0,1);
	GradX[0][2] = gsl_matrix_get(GradGx,0,2);
	GradX[1][0] = gsl_matrix_get(GradGx,1,0);
	GradX[1][1] = gsl_matrix_get(GradGx,1,1);
	GradX[1][2] = gsl_matrix_get(GradGx,1,2);

	CLR[0][0] = gsl_matrix_get(QGL,0,0) + gsl_matrix_get(QLi,0,0);
	CLR[0][1] = gsl_matrix_get(QGL,0,1) + gsl_matrix_get(QLi,0,1);
	CLR[1][0] = gsl_matrix_get(QGL,1,0) + gsl_matrix_get(QLi,1,0);
	CLR[1][1] = gsl_matrix_get(QGL,1,1) + gsl_matrix_get(QLi,1,1);

	// TRANSFORM LINE ELEMENTS TO LOCAL FRAME
	alpha = alpha - th;
	r = r - Srli;

	/**
		SET the end points to robot frame coordinates
	 */
	StartPointXY[0] = StartPointXY[0] - gsl_matrix_get(Mean, 0, 0); // x coordinate
	StartPointXY[1] = StartPointXY[1] - gsl_matrix_get(Mean, 1, 0); // y coordinate

	gsl_matrix_free(GradGLi);
	gsl_matrix_free(GradGLiT);
	gsl_matrix_free(QLi);
	gsl_matrix_free(QGL);
	gsl_matrix_free(GradGx);
	gsl_matrix_free(GradGxT);
	gsl_matrix_free(QLw);
	gsl_matrix_free(mat2x2);
	gsl_matrix_free(mat2x3);

}
/*
 * 	Merge two lines, following method described in report. Section 5.1.6, eq 5.13 to 5.14

	[ theta_f ] = Ck*(Cli^-1*pi + Clj^-1*pj)   (5.13)
	[ phi_f   ]

	where
		Ck = (Cli^-1+Clj^-1)^-1
		pi and pj are the line parameters from the two lines, Cli and Clj are the covariance matrix

	Pre Conditions: Both features have the same referential
 *
 * */
void line_t::mergeWithFeature ( feature_t * featIn ) {

	// check if the feature being merged is of the same type
	if (typeid(*this).name() != typeid((*featIn)).name() )
	{
		return;
	}
	// down cast the in same type
	line_t * Observed = static_cast<line_t*>(&*featIn);
	gsl_matrix * CLWMap = gsl_matrix_alloc (2,2);
	gsl_matrix * CLObs = gsl_matrix_alloc (2,2);
	gsl_matrix * CLWMapInv = gsl_matrix_alloc (2,2);
	gsl_matrix * CLObsInv = gsl_matrix_alloc (2,2);
	gsl_matrix * LW = gsl_matrix_alloc (2,1);
	gsl_matrix * LObs = gsl_matrix_alloc (2,1);
	gsl_matrix * Sum =  gsl_matrix_alloc (2,1);
	gsl_matrix * SumWi =  gsl_matrix_alloc (2,2);
	gsl_matrix * SumWiInv =  gsl_matrix_alloc (2,2);
	gsl_matrix * res2x1 =  gsl_matrix_alloc (2,1);


	// get line parameters from the Observed Line
	Observed->getLineCovariance ( CLObs );
	getLineCovariance ( CLWMap );
	gsl_matrix_set(LW,0,0,alpha);
	gsl_matrix_set(LW,1,0,r);
	gsl_matrix_set(LObs,0,0,Observed->getAlpha());
	gsl_matrix_set(LObs,1,0,Observed->getR());
	// get inv matrix of both matrix
	CalcInvMatrix ( CLWMapInv, CLWMap, 2);
	CalcInvMatrix ( CLObsInv, CLObs, 2);

	/*
	 * merge both features
	 * */
	gsl_matrix_set_zero(Sum);
	gsl_matrix_set_zero(SumWi);

	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, CLObsInv,  LObs,  	0.0, res2x1);
	gsl_matrix_add(Sum,res2x1);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, CLWMapInv, LW,  	0.0, res2x1);
	gsl_matrix_add(Sum,res2x1);

	gsl_matrix_add(SumWi,CLObsInv);
	gsl_matrix_add(SumWi,CLWMapInv);

	// final variance
	CalcInvMatrix( SumWiInv, SumWi, 2);
	// final alpha, r
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans,1.0, SumWiInv, Sum, 0.0, res2x1);

	CLR[0][0] = gsl_matrix_get(SumWiInv,0,0);
	CLR[0][1] = gsl_matrix_get(SumWiInv,0,1);
	CLR[1][0] = gsl_matrix_get(SumWiInv,1,0);
	CLR[1][1] = gsl_matrix_get(SumWiInv,1,1);
	alpha = gsl_matrix_get(res2x1,0,0);
	r = gsl_matrix_get(res2x1,1,0);

	gsl_matrix_free( CLWMap );
	gsl_matrix_free( CLObs );
	gsl_matrix_free( CLWMapInv );
	gsl_matrix_free( CLObsInv );
	gsl_matrix_free( LW );
	gsl_matrix_free( LObs );
	gsl_matrix_free( Sum );
	gsl_matrix_free( SumWi );
	gsl_matrix_free( SumWiInv );
	gsl_matrix_free( res2x1 );

}
void line_t::getCovariance ( gsl_matrix * GradOut, int * index_i ) {
	// do some error checking

	if ( *index_i + 2 > (GradOut->size1) )
		return;

	gsl_matrix_set(GradOut,*index_i,*index_i		,CLR[0][0]);
	gsl_matrix_set(GradOut,*index_i,*index_i+1		,CLR[0][1]);
	gsl_matrix_set(GradOut,*index_i+1,*index_i		,CLR[1][0]);
	gsl_matrix_set(GradOut,*index_i+1,*index_i+1	,CLR[1][1]);
	(*index_i) = (*index_i) + 2;

}
void line_t::getCovarianceDiagonalElems ( gsl_matrix * GradOut, int * index_i ) {
	// do some error checking
	if ( *index_i + 2 > (GradOut->size1) )
		return;
	gsl_matrix_set(GradOut,*index_i,*index_i		,CLR[0][0]);
	gsl_matrix_set(GradOut,*index_i+1,*index_i+1	,CLR[1][1]);
	(*index_i) = (*index_i) + 2;

}
void line_t::getPoseGradient ( gsl_matrix * GradOut, int * index_i ) {

	// do some error checking
	if ( *index_i + 2 > (GradOut->size1) )
		return;

	// (4.19)
	gsl_matrix_set(GradOut,*index_i,0	,GradX[0][0]);
	gsl_matrix_set(GradOut,*index_i,1	,GradX[0][1]);
	gsl_matrix_set(GradOut,*index_i,2	,GradX[0][2]);
	gsl_matrix_set(GradOut,*index_i + 1,0	,GradX[1][0]);
	gsl_matrix_set(GradOut,*index_i + 1,1	,GradX[1][1]);
	gsl_matrix_set(GradOut,*index_i + 1,2	,GradX[1][2]);
	// fill nex row
	(*index_i) = (*index_i) + 2;



	//	printf ("GradX = [%g  %g]\n [%g  %g]\n [%g  %g] \n\n",
	//			GradX[0][0],
	//			GradX[0][1],
	//			GradX[0][2],
	//			GradX[1][0],
	//			GradX[1][1],
	//			GradX[1][2]
	//	);
}
bool line_t::setStartPoint(double X, double Y, double bearing, double r, int index)
{
	this->StartPointXY[0] = X;
	this->StartPointXY[1] = Y;
	this->StartPointAlphaR[0] = bearing;
	this->StartPointAlphaR[1] = r;
	this->StartPointIndex = index;
}
bool line_t::setEndPoint(double X, double Y, double bearing, double r, int index)
{
	this->StartPointXY[0] = X;
	this->StartPointXY[1] = Y;
	this->StartPointAlphaR[0] = bearing;
	this->StartPointAlphaR[1] = r;
	this->StartPointIndex = index;
}
bool line_t::setLineParametersGSL(double rIn, double alphaIn, gsl_matrix * CLRIN)
{
	this->alpha = alphaIn;
	this->r = rIn;

	if (CLRIN != NULL) {// set covariace matrix
		CLR[0][0] = gsl_matrix_get(CLRIN,0,0);
		CLR[0][1] = gsl_matrix_get(CLRIN,0,1);
		CLR[1][0] = gsl_matrix_get(CLRIN,1,0);
		CLR[1][1] = gsl_matrix_get(CLRIN,1,1);
	}
}
bool line_t::setLineParameters(double rIn, double alphaIn, double CLRIN[2][2])
{
	this->alpha = alphaIn;
	this->r = rIn;

	if (CLRIN != NULL) {// set covariace matrix
		CLR[0][0] = CLRIN[0][0];
		CLR[0][1] = CLRIN[0][1];
		CLR[1][0] = CLRIN[1][0];
		CLR[1][1] = CLRIN[1][1];
	}
}
double line_t::getAlpha() const
{
	return alpha;
}

double * line_t::getCLR()
{
	return &CLR[0][0];
}

int line_t::getFeatType() const
{
	return FeatType;
}

bool line_t::isMature () {
	return ( Maturity == MATURITY_LEVEL );
}
void line_t::increaseMaturity ( ) {
	if ( Maturity < MATURITY_LEVEL )
		Maturity++;
}
void line_t::decreaseMaturity ( ) {
	if ( Maturity > 0 && Maturity < MATURITY_LEVEL )
		Maturity--;
}
int line_t::getLineId() const
{
	return LineId;
}

double line_t::getR() const
{
	return r;
}

void line_t::setAlpha(double alpha)
{
	this->alpha = alpha;
}

void line_t::setCLR(double CLR[2][2])
{
	this->CLR[0][0] = CLR[0][0];
	this->CLR[0][1] = CLR[0][1];
	this->CLR[1][0] = CLR[1][0];
	this->CLR[1][1] = CLR[1][1];
}
void line_t::setFeatType(int FeatType)
{
	this->FeatType = FeatType;
}

void line_t::setLineId(int LineId)
{
	this->LineId = LineId;
}

void line_t::setMaturity(int Maturity)
{
	this->Maturity = Maturity;
}

void line_t::setR(double r)
{
	this->r = r;
}
void line_t::getLineCovariance(gsl_matrix *OutMatrix)
{

	if (OutMatrix != NULL) {
		gsl_matrix_set(OutMatrix,0,0,CLR[0][0]);
		gsl_matrix_set(OutMatrix,0,1,CLR[0][1]);
		gsl_matrix_set(OutMatrix,1,0,CLR[1][0]);
		gsl_matrix_set(OutMatrix,1,1,CLR[1][1]);
	}
}
bool line_t::setEndPointAttr(int endAttr)
{
	this->EndPointAttr = endAttr;
	return true;
}
bool line_t::setStartPointAttr(int startAttr)
{
	this->StartPointAttr = startAttr;
	return true;
}
double line_t::getLineAlpha()
{
	return this->alpha;
}
double line_t::getLineR()
{
	return this->r;
}
int line_t::getEndPointAttr()
{
	return this->EndPointAttr;
}
int line_t::getStartPointAttr()
{
	return this->StartPointAttr;
}
