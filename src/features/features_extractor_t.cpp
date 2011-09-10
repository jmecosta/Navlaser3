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

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>

#include "features_extractor_t.h"

// constructor
features_extractor_t::features_extractor_t()
{

}
// line extraction main method
void features_extractor_t::features_LineExtractor(laser_t * ScanObj, list<line_t> * ExtractedLines )
{
	double angle_resolution;
	double sin_angle_resolution;
	double sin_angle_diff;
	double sin_total;
	double euclid_distance;
	double diff_x;
	double diff_y;
	int first_point = 0;
	int i = 0;
	int first_line = 0;
	int last_point = 0;
	double d_max;
	int fix_last_line = 0;
	line_t * new_line = NULL;
	line_t * lineregression = NULL;

	first_point = i;

	for (i = 1; i < (ScanObj->getNmbSamples()); i++)
	{
		last_point = i - 1;
		int Pts = (last_point - first_point + 1);

		// sample segmentation
		if (Pts >= MIN_LINE_POINTS && IsABreakPoint(ScanObj, i))
		{
			// gets all lines from this subset point sample
			LineEstimator_RecursiveCornerDetectorSplitEstimateAndMergeLines(
					ExtractedLines, ScanObj, first_point, last_point);

			first_point = i;
		}
		else
		{
			if (ScanObj->getRange(i) > ScanObj->getLaserMaxRange())
			{
				first_point = i + 1;
			}
		}
	}
	LinesEstimator_MergeContiguos(ExtractedLines, ScanObj);
	LinesEstimator_SetEndPointsAttr(ExtractedLines, ScanObj);

}

/** Break Point Detectors Methods **/
bool features_extractor_t::IsABreakPoint(laser_t * ScanObj, int i)
{
	bool retval = false;
	switch (BreakPointMethod)
	{
	case SIMPLEBREAKMETHOD:
		retval = IsABreakPoint_SimpleMethod(ScanObj, i);
		break;
	case ADPTBREAKMETHOD:
		retval = IsABreakPoint_AdptiveRuptureMethod(ScanObj, i);
		break;
	default:
		retval = IsABreakPoint_AdptiveRuptureMethod(ScanObj, i);
		break;
	}
	return retval;
}

bool features_extractor_t::IsABreakPoint_SimpleMethod(laser_t * ScanObj, int i)
{
	// todo
	return false;
}
bool features_extractor_t::IsABreakPoint_AdptiveRuptureMethod(
		laser_t * ScanObj, int i)
{

	double angle_resolution = ScanObj->getLaserResolution();
	double sin_angle_resolution = sin(angle_resolution);
	double sin_angle_diff = sin(ADPTLAMBDA - angle_resolution);
	double sin_total = sin_angle_resolution / sin_angle_diff;

	double d_max = ScanObj->getRange(i - 1) * sin_total + 3 * ScanObj->getSigmaR(i);
	double diff_x = ScanObj->getX(i) - ScanObj->getX(i-1);
	double diff_y = ScanObj->getY(i) - ScanObj->getY(i-1);
	double euclid_distance = sqrt(pow(diff_x, 2) + pow(diff_y, 2));

	if ((euclid_distance > d_max) || (i == (ScanObj->getNmbSamples() - 1)))
	{ //new break point
		return true;
	}
	return false;
}

bool features_extractor_t::IsABreakPoint_IsThereaBreakPoint(laser_t * ScanObj,
		int start, int end)
{
	for (int i = 1; i < (ScanObj->getNmbSamples()); i++)
	{
		if (IsABreakPoint(ScanObj, i))
		{
			return true;
		}
	}
}
/** Corner Detection Methods **/
bool features_extractor_t::Corners_LineValidationAndCornerDetection(
		laser_t * ScanObj, int start_point, int end_point,
		list<int> * CornersIndex)
{
	bool retval = false;
	switch (CornerEstimationMethod)
	{
	case CORNER2DERIVATETEST:
		retval = Corners_2derivativeTestEstimation(ScanObj, start_point,
				end_point, CornersIndex);
		break;
	case CORNERRAMSAC:
		retval = Corners_RamsacCornerDetection(ScanObj, start_point, end_point,
				CornersIndex);
		break;
	default:
		retval = Corners_RamsacCornerDetection(ScanObj, start_point, end_point,
				CornersIndex);
		break;
	}
	return retval;
}

bool features_extractor_t::Corners_2derivativeTestEstimation(laser_t * ScanObj,
		int start_point, int end_point, list<int> * CornersIndex)
{
	double distance;
	int n_pts = end_point - start_point + 1;

	double gradDown[5];
	double gradUp[5];

	double dx[n_pts - 1];
	double dx2[n_pts - 2];

	int j = 0;
	int k = 0;
	double distAnt = 0;

	line_t line;

	dx2[0] = 0;

	// first estimate the line that connects the two end points,
	// this is a quick test to reduce the amount of operations to be performed
	if (!LineEstimator_FitLine(start_point, end_point, ScanObj, &line))
	{
		return false;
	}

	double line_alpha = line.getLineAlpha();
	double line_r = line.getLineR();

	for (int i = start_point; i < (end_point + 1); i++)
	{
		distance = fabs(ScanObj->getX(i) * cos(line_alpha) + ScanObj->getY(i) * sin(
				line_alpha) - line_r);
		if (j != 0) //first derivate
			dx[j - 1] = distance - distAnt;
		++j;
		distAnt = distance;
	}

	// filter the first derivate
	median_filter_points(&dx[0], n_pts - 1, 5);

	for (j = 1; j < (n_pts - 1); j++)
	{
		//second derivate
		dx2[j - 1] = dx[j] - dx[j - 1];
	}

	// filter the second derivate
	median_filter_points(&dx2[0], n_pts - 2, 5);

	// find maximums and minimums of the second derivate
	for (j = 1; j < (n_pts - 2); j++)
	{

		// todo

	}
	return true;

}

// returns false
bool features_extractor_t::Corners_RamsacCornerDetection(laser_t * ScanObj,
		int start_point, int end_point, list<int> * CornersIndex)
{
	double ransac_threshold = 3 * RANSAC_FACTOR;
	double distance = 0.0;
	double max_distance = 0.0;
	int irregular_points = 0;
	double percent = 0.0;
	int break_ind = 0;

	line_t line;

	// first estimate the line that connects the two end points,
	// this is a quick test to reduce the amount of operations to be performed
	if (!LineEstimator_EndPointLineFit(start_point, end_point, ScanObj, &line))
	{
		return false;
	}

	double line_alpha = line.getLineAlpha();
	double line_r = line.getLineR();

	for (int i = start_point; i < (end_point); i++)
	{
		distance = fabs(ScanObj->getX(i) * cos(line_alpha) + ScanObj->getY(i) * sin(
				line_alpha) - line_r);

		if (distance > ransac_threshold)
		{
			irregular_points++;
		}

		if (distance > max_distance)
		{
			break_ind = i;
			max_distance = distance;
		}
	}

	percent = (double) irregular_points / (end_point - start_point + 1);

	if (percent > 0.05)
	{
		(*CornersIndex).push_back(break_ind);
		return false;
	}

	return true;
}

/** LINE ESTIMATION METHODS **/
// main method, 
bool features_extractor_t::LineEstimator_FitLine(int start, int end,
		laser_t * ScanObj, line_t * new_line)
{
	bool retval = false;
	switch (LineEstimationMethod)
	{
	case LINEENDPOINTFIT:
		retval = LineEstimator_EndPointLineFit(start, end, ScanObj, new_line);
		break;
	case LINELEASTSQUAREMETHOD:
		//retval = LineEstimator_LeastSquareLineFit(start, end, ScanObj,
		//		new_line);
		retval = LineEstimator_MinimizePointToLineDifferential(start, end, ScanObj,
				new_line);
		break;
	case LINESIMPLEREGRESSION:
		// todo
		break;
	default:
		break;
	}
	return retval;

}

// false line not computed
bool features_extractor_t::LineEstimator_EndPointLineFit(int start, int end,
		laser_t * ScanObj, line_t * new_line)
{
	double alpha = 0.0;
	double r = 0.0;
	double wiS = 0.0;
	double wiE = 0.0;
	double xmw = 0.0;
	double ymw = 0.0;
	double sumWii = 0.0;
	double x[2] =
	{ ScanObj->getX(start), ScanObj->getX(end) };
	double y[2] =
	{ ScanObj->getY(start), ScanObj->getY(end) };
	;

	if ((end - start + 1) < MIN_LINE_POINTS)
	{
		return false;
	}

	// use least square line fitting using only the two end points to estimate the line
	wiS = 1 / pow(ScanObj->getSigmaR(start), 2);
	wiE = 1 / pow(ScanObj->getSigmaR(end), 2);
	xmw = wiS * x[0] + wiE * x[1];
	ymw = wiS * y[0] + wiE * y[1];
	sumWii = wiS + wiE;

	ymw = ymw / sumWii;
	xmw = xmw / sumWii;

	double nom = -2 * wiS * (x[0] - xmw) * (y[0] - ymw) + wiE * (x[1] - xmw)
															* (y[1] - ymw);
	double denom = wiS * (pow(y[0] - ymw, 2) - pow(x[0] - xmw, 2)) + wiE
			* (pow(y[1] - ymw, 2) - pow(x[1] - xmw, 2));

	alpha = 0.5 * atan2(nom, denom);
	r = xmw * cos(alpha) + ymw * sin(alpha);

	if (r < 0)
	{
		alpha = alpha + M_PI;
		r = -r;
	}

	if (alpha > M_PI)
		alpha = alpha - 2 * M_PI;

	// update line end point ids
	/*point start and end point that belongs to the line and line parameters */
	double CL[2][2];
	new_line->setLineParameters(r, alpha, CL);
	new_line->setStartPoint(ScanObj->getX(start), ScanObj->getY(start), ScanObj->getBearings(start), ScanObj->getRange(start), start);
	new_line->setEndPoint(ScanObj->getX(end), ScanObj->getY(end), ScanObj->getBearings(end), ScanObj->getRange(end), end);

	return true;
}
/*
 * Arras implementation of a line estimator, by minimizing the distance to the line equations
 * C.1 First Moments
	∂S	= 0
	∂r
				(C.1)
	∂S = 0
	∂α

	where S is the weighted sum of squared errors

		S = ∑ w i ( ρ i cos θ i cos α + ρ i sin θ i sin α – r )^2   	(C.2)

		line parameters are found by solving equations C.1

				∑ wi ρ i cos ( θ i – α )
		r = ---------------------------------------------- 		(C.8)
						∑ wi

		α  														(C.20)

 *
 * */
bool features_extractor_t::LineEstimator_MinimizePointToLineDifferential (int start, int end,
		laser_t * ScanObj, line_t * new_line )
{
	int NP = end - start + 1;
	if(NP < MIN_LINE_POINTS)
	{
		return NULL;
	}

	int i = 0;
	int j = 0;
	double N = 0;
	double D = 0;
	double SumXi = 0; // C.44
	double SumYi = 0; // C.44
	double AverageXi = 0; // C.44
	double AverageYi = 0; // C.44
	double alpha = 0;
	double r = 0;
	double sigmaASquare = 0;
	double sigmaRSquare = 0;
	double sigmaAR = 0;
	double C72_1 = 0;
	double C72_2 = 0;
	double C72_C72 = 0;
	double C72 = 0;
	double C73 = 0;
	double C73_1 = 0;
	double C74 = 0;
	double DN_Dpi = 0;
	double DD_Dpi = 0;
	double DAlpha_Dpi = 0;
	double Dr_Dpi = 0;
	double R_Neg = 0;

	double N1 = 0;
	double N2 = 0;
	// C.42, 2*NP complexity
	for (  i = start; i < end; i++ ) {
		SumXi += ScanObj->getX(i);
		SumYi += ScanObj->getY(i);

//		cout << "X:  " << ScanObj->getX(i) << " Y: " << ScanObj->getY(i) << endl;
//		cout << "ALPHA:  " << ScanObj->getBearings(i) << " R: " << ScanObj->getRange(i) << endl;
//		cout << "SIGMA R: " << ScanObj->getSigmaR(i) << endl;
	}
	AverageXi = (1.0 / (double)NP ) * SumXi;
	AverageYi = (1.0 / (double)NP ) * SumYi;

	for (  i = start; i < end; i++ ) {
		D += (AverageYi - ScanObj->getY(i)) * (AverageXi - ScanObj->getX(i));
		N += (AverageYi - ScanObj->getY(i))*(AverageYi - ScanObj->getY(i))  -  (AverageXi - ScanObj->getX(i))*(AverageXi - ScanObj->getX(i));
	}
	D = -2*D;

	// line parameters, r and alpha C.42 C.43
	alpha = 0.5 * atan2 ( D, N);//  C.42
	r = AverageXi * cos (alpha) + AverageYi * sin (alpha); // C.43

//    cout << "LINE Alpha: " << alpha << "  R: " << r << endl;

	if (r < 0)
	{
		alpha = alpha + M_PI;
		r = -r;
	}
	if (alpha > M_PI)
		alpha = alpha - 2 * M_PI;

	if (alpha < -M_PI)
		alpha = alpha + 2 * M_PI;


	// line uncertainty C.72, C.73 and C.74
	R_Neg =  AverageYi * cos (alpha) - AverageXi * sin (alpha); // C.43
	for (  i = start; i < end; i++ ) {

		// reuse members of eq. C.51 to C.74
		double SigmaRSquare = ScanObj->getSigmaR(i) *  ScanObj->getSigmaR(i);
		double Member_1 = AverageXi * cos (ScanObj->getBearings(i)) - AverageYi * sin (ScanObj->getBearings(i)) - ScanObj->getRange(i) * cos ( 2 * ScanObj->getBearings(i) );
		double Member_2 = AverageXi * sin (ScanObj->getBearings(i)) + AverageYi * cos (ScanObj->getBearings(i)) - ScanObj->getRange(i) * sin ( 2 * ScanObj->getBearings(i) );

		// C.72 temp members
		C72_1 = N * Member_1;
		C72_2 = D * Member_2;
		C72_C72 = (C72_1 - C72_2)*(C72_1 - C72_2);

		// calc C.45
		// C.51
		DN_Dpi = 2 * Member_2;
		// C.60
		DD_Dpi = 2 * Member_1;
		// C.63
		DAlpha_Dpi = (1 /(D*D+N*N)) *  ( C72_1 - C72_2 );
		// C.70
		Dr_Dpi = ( 1 / NP ) * cos ( ScanObj->getBearings(i) - alpha ) +  DAlpha_Dpi * ( R_Neg );


		// C.72
		C72 += C72_C72 * SigmaRSquare;
		// C.73
		C73_1 = (1 / NP ) * cos ( ScanObj->getBearings(i) - alpha) + DAlpha_Dpi*R_Neg;
		C73 += C73_1 * C73_1 *  SigmaRSquare;
		// C.74
		C74 += DAlpha_Dpi * DAlpha_Dpi *  SigmaRSquare;

	}

	// final update in C.72
	C72 = (1 / (D*D + N*N) * (D*D + N*N) ) * C72;

	/*
	 * Calc the end points that belong to the line
	 * */
	double InSV[2] = {ScanObj->getX(start),ScanObj->getY(start)};
	double OutSV[2];
	CalcInterSectionPointBtwLineAndPoint(alpha, r, InSV, &OutSV[0]);
	double InEV[2] = {ScanObj->getX(end),ScanObj->getY(end)};
	double OutEV[2];
	CalcInterSectionPointBtwLineAndPoint(alpha, r, InEV, &OutEV[0]);

	point_t pointStart;
	point_t pointEnd;

	/*point start and end point that belongs to the line and line parameters */
	double SV[2][2];
	SV[0][0] = C72;
	SV[0][1] = C74;
	SV[1][0] = C74;
	SV[1][1] = C73;
	new_line->setLineParameters(r, alpha, SV);
	// update line end point ids
	new_line->setStartPoint(OutSV[0], OutSV[1], ScanObj->getBearings(start), ScanObj->getRange(start), start);
	new_line->setEndPoint(OutEV[0], OutEV[1], ScanObj->getBearings(end), ScanObj->getRange(end), end);

    cout << "LINE Alpha: " << alpha << "  R: " << r << endl;

	     printf ("Hl = [%g  %g]\n [%g  %g]\n\n\n",
	             C72,
	             C74,
	             C74,
	             C73
	            );

return true;

}
bool features_extractor_t::LineEstimator_VandorpeLineRegression(int start, int end,
		laser_t * ScanObj, line_t * new_line )
{ // From vandorpe
	int NP = end-start;
	if(NP < MIN_LINE_POINTS)
	{
		return NULL;
	}
	gsl_matrix * Cs = gsl_matrix_alloc(2,2);
	gsl_matrix * Cl = gsl_matrix_alloc(2,2);
	gsl_matrix * Jyx_tranpose = gsl_matrix_alloc(2,2);
	gsl_matrix * Jyx = gsl_matrix_alloc(2,2);
	gsl_matrix * Jxy_tranpose = gsl_matrix_alloc(2,2);
	gsl_matrix * Jxy = gsl_matrix_alloc(2,2);
	gsl_matrix * Cvyx = gsl_matrix_alloc(2,2);
	gsl_matrix * Cvxy = gsl_matrix_alloc(2,2);
	gsl_matrix * Hl = gsl_matrix_alloc(2,2);
	gsl_matrix * Hl_tranpose = gsl_matrix_alloc(2,2);

	gsl_matrix * result_2x2 = gsl_matrix_alloc(2,2);
	gsl_matrix * result_2x3 = gsl_matrix_alloc(2,3);
	gsl_matrix * Gs_result_2x2 = gsl_matrix_alloc(2,2);
	gsl_matrix * Gst_result_2x2 = gsl_matrix_alloc(2,2);


	gsl_matrix * G = gsl_matrix_alloc(2,2);
	gsl_matrix * G_T = gsl_matrix_alloc(2,2);
	gsl_matrix * Cpi = gsl_matrix_alloc(2,2);

	gsl_matrix_set_zero (Cpi);
	gsl_matrix_set_zero (Cl);
	gsl_matrix_set_zero (Cvyx);
	gsl_matrix_set_zero (Cvxy);


	int i;
	double Rx = 0.0;
	double Rxx = 0.0;
	double Ry = 0.0;
	double Ryy = 0.0;
	double Rxy = 0.0;

	double T= 0.0;
	double wi = 0.0;

	for (i = start; i < end; i++)
	{
		Rx += ScanObj->getX(i);
		Ry += ScanObj->getY(i);
		Rxy += ScanObj->getX(i)*ScanObj->getY(i);
		Rxx += ScanObj->getX(i)*ScanObj->getX(i);
		Ryy += ScanObj->getY(i)*ScanObj->getY(i);

		wi = pow(ScanObj->getSigmaR(i),2);
		gsl_matrix_set_zero(Cs);
		gsl_matrix_set(Cs,0,0,wi*pow(cos(ScanObj->getBearings(i)),2)); //
		gsl_matrix_set(Cs,0,1,wi*sin(ScanObj->getBearings(i))*cos(ScanObj->getBearings(i)));
		gsl_matrix_set(Cs,1,0,wi*sin(ScanObj->getBearings(i))*cos(ScanObj->getBearings(i)));
		gsl_matrix_set(Cs,1,1,wi*pow(sin(ScanObj->getBearings(i)),2)); //


		/*lets calc the uncertanty Cp of the current point  Cpi = G*Cs*G_T

    	F(r,alpha) = {r*cos(alpha),r*sin(alpha)}

    	G[2x2] = [[ cos(alfa_vector[i]) -r_vector[i]*sin(alfa_vector[i]) ]   (1.1)
    			  [ sin(alfa_vector[i])  r_vector[i]*cos(alfa_vector[i]) ]]

			G = deferential of F wrt to alfa and r
    	F[3x3]

		 */

		// set (1.1)
		gsl_matrix_set(G,0,0,cos(ScanObj->getBearings(i)));
		gsl_matrix_set(G,0,1,-ScanObj->getRange(i)*sin(ScanObj->getBearings(i)));
		gsl_matrix_set(G,1,0,sin(ScanObj->getBearings(i)));
		gsl_matrix_set(G,1,1,ScanObj->getRange(i)*cos(ScanObj->getBearings(i)));
		gsl_matrix_transpose_memcpy(G_T, G);

		// Propagate point uncertanty to sensor referential
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, G, Cs,0.0, Gs_result_2x2);
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, Gs_result_2x2, G_T,0.0, Cpi);

		// SUM new found point uncertanty to the current line uncertanty
		gsl_matrix_add(Cl,Cpi);

		/*lets calc the uncertanty Cv of the vector V=[p,theta] of the current line under extraction
    Cv = sum(J*Cp*Jt) and y=a+bx
    J is the jacobian of b=f1((x1,y1),...,(xn,yn)) e a=f2((x1,y1),...,(xn,yn))
    for all point we define Ji asjacobian of b=f1((xi,yi)) and a=f2(xi,yi)

    we now define the regression of y to x like y=mx+q obtaining Jyx
    and the regression of x to y like x=sy+t obtaining Jxy

    at the end we only pick the best, in terms of N1 and N2
		 */

		//regression y to x
		gsl_matrix_set_zero(Jyx);
		if(ScanObj->getX(i) != 0)
		{
			gsl_matrix_set(Jyx, 0, 0, -ScanObj->getY(i)/(ScanObj->getX(i)*ScanObj->getX(i)));
			gsl_matrix_set(Jyx, 0, 1, 1/(ScanObj->getX(i)));
		} else {
			cout << " INVALID POINT X = 0" << endl;
		}

		gsl_matrix_transpose_memcpy (Jyx_tranpose, Jyx);

		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, Jyx, Cpi,0.0, result_2x2);
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, result_2x2, Jyx_tranpose,0.0, Jyx);

		gsl_matrix_add(Cvyx,Jyx);

		//regression x to y
		gsl_matrix_set_zero(Jxy);
		if(ScanObj->getY(i) != 0)
		{
			gsl_matrix_set(Jxy, 0, 0, 1/(ScanObj->getY(i)));
			gsl_matrix_set(Jxy, 0, 1,  -ScanObj->getX(i)/(ScanObj->getY(i)*ScanObj->getY(i)) );
		} else {
			cout << " INVALID POINT Y = 0" << endl;
		}

		gsl_matrix_transpose_memcpy (Jxy_tranpose,Jxy);

		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, Jxy, Cpi,0.0, result_2x2);
		gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, result_2x2, Jxy_tranpose,0.0, Jxy);

		gsl_matrix_add(Cvxy,Jxy);

	}

	T = Rxy*NP - Rx*Ry;
	double N1 = Rxx*NP - Rx*Rx;
	double N2 = Ryy*NP - Ry*Ry;

	/*finaly the incertanty related to the current line under extraction putting all the paramentres related to the robot and the mesure
	Cl = Hl*Cvxy*Hlt
	were Hl is the jacobian of Xl to V=[m,q]
	 */

	//     double temp =s*s+1;
	//
	//     gsl_matrix_set(Hl, 0, 0, -t*s/(sqrt(temp*temp*temp)));
	//     gsl_matrix_set(Hl, 0, 1, 1/sqrt(temp));
	//     gsl_matrix_set(Hl, 1, 0, -1/temp);
	//     gsl_matrix_set(Hl, 1, 1, 0);

	//     printf ("Hl = [%g  %g]\n [%g  %g]\n\n\n",
	//             gsl_matrix_get (Hl,0,0),
	//             gsl_matrix_get (Hl,0,1),
	//             gsl_matrix_get (Hl,1,0),
	//             gsl_matrix_get (Hl,1,1)
	//            );

	gsl_matrix_transpose_memcpy (Hl_tranpose,Hl);

	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, Hl, Cvxy,0.0, result_2x2);
	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,1.0, result_2x2, Hl_tranpose,0.0, Cl);

	double r = 0;
	double alpha = 0;

	if( N1 > N2)
	{ //horizontal line, y=mx+q y = q para m = 0,

		double m = T/N1;
		double q = (Ry-m*Rx)/NP;

		new_line->setR(fabs(q/sqrt(m*m+1)));

		if(m == 0) {
			if(Ry<0) alpha = -M_PI/2.0;
			if(Ry>0) alpha = M_PI/2.0;
		} else {
			if(q > 0) {
				if(m>0) alpha = 0.5*M_PI+atan(fabs(m));
				else alpha = 0.5*M_PI-atan(fabs(m));
			} else {
				if(m>0) alpha = -0.5*M_PI+atan(fabs(m));
				else alpha = -0.5*M_PI-atan(fabs(m));
			}
		}
	} else { //vertical line x = s*y + t -> x = t para s =0; line completly vertival

		double s = T/N2;
		double t = (Rx-s*Ry)/(double)NP;

		new_line->setR(fabs(t/sqrt(s*s+1)));

		if(s == 0) {
			if(Rx<0) new_line->setAlpha(M_PI);
			if(Rx>0) new_line->setAlpha(0.0);
		} else {
			if(t>0) {
				if(s>0) new_line->setAlpha(-atan(fabs(s)));
				else new_line->setAlpha(atan(fabs(s)));
			} else {
				if(s>0) new_line->setAlpha(M_PI-atan(fabs(s)));
				else new_line->setAlpha(-M_PI+atan(fabs(s)));
			}
		}
	}

	/*
	 * Calc the end points that belong to the line
	 * */
	double InSV[2] = {ScanObj->getX(start),ScanObj->getY(start)};
	double OutSV[2];
	CalcInterSectionPointBtwLineAndPoint(alpha, r, InSV, &OutSV[0]);
	double InEV[2] = {ScanObj->getX(end),ScanObj->getY(end)};
	double OutEV[2];
	CalcInterSectionPointBtwLineAndPoint(alpha, r, InEV, &OutEV[0]);

	/*point start and end point that belongs to the line and line parameters */
	new_line->setLineParametersGSL(r, alpha, Cl);
	new_line->setStartPoint(OutSV[0], OutSV[1], ScanObj->getBearings(start), ScanObj->getRange(start), start);
	new_line->setEndPoint(OutEV[0], OutEV[1], ScanObj->getBearings(end), ScanObj->getRange(end), end);


	gsl_matrix_free(Jyx_tranpose);
	gsl_matrix_free(Jyx);
	gsl_matrix_free(Jxy_tranpose);
	gsl_matrix_free(Jxy);
	gsl_matrix_free(Cvyx);
	gsl_matrix_free(Cvxy);
	gsl_matrix_free(Hl);
	gsl_matrix_free(Hl_tranpose);
	gsl_matrix_free(G);
	gsl_matrix_free(G_T);
	gsl_matrix_free(Cpi);
	gsl_matrix_free(result_2x2);
	gsl_matrix_free(result_2x3);
	gsl_matrix_free(Gs_result_2x2);
	gsl_matrix_free(Gst_result_2x2);
	gsl_matrix_free(Cs);
	gsl_matrix_free(Cl);



	//   printf("p=%g,theta = %g\n",new_line->r,new_line->alpha);
	//
	//   if(new_line->N1 > new_line->N2)
	//   {
	//     printf ("# best fit: Y = %g + %g X\n", new_line->q, new_line->m);
	//   }
	//   else
	//   {
	//     printf ("# best fit: X = %g + %g Y\n", new_line->t, new_line->s);
	//   }
	//
	//   printf ("# covariance matrix:\n");
	//   printf ("CL = [[%g  %g]\n [%g  %g]]\n",gsl_matrix_get (Cl,0,0),gsl_matrix_get (Cl,0,1),
	//           gsl_matrix_get (Cl,1,0),gsl_matrix_get (Cl,1,1));
	//   //
	//   printf ("# points = %i\n", end-start+1);
	//     cout << "x_start = " << new_line->x_start << " Y_start = "<< new_line->x_end << endl;
	//     cout << "x_end = " << new_line->y_start << " Y_end = " << new_line->y_end <<endl;

}


bool features_extractor_t::LineEstimator_LeastSquareLineFit(int start, int end,
		laser_t * ScanObj, line_t * new_line)
{
	int i = 0;
	int j = 0;
	double D = 0.0;
	double N = 0.0;
	double xmw = 0.0;
	double ymw = 0.0;
	double nom = 0.0;
	double denom = 0.0;
	double line_alpha = 0.0;
	double line_r = 0.0;
	double wi[end - start + 1];
	double sumWii = 0.0;
	double Sx2 = 0.0;
	double Sy2 = 0.0;
	double Sxy = 0.0;
	double dr_alpha = 0.0;
	double dr_Sxy_yi = 0.0;
	double dr_Sxy_xi = 0.0;
	double dr_Sx2_yi = 0.0;
	double dr_Sx2_xi = 0.0;
	double dr_Sy2_yi = 0.0;
	double dr_Sy2_xi = 0.0;
	double Ai11;
	double Ai12;
	double Ai21;
	double Ai22;
	int n_points = (end - start + 1);

	gsl_matrix * Ai = gsl_matrix_alloc(2, 2);
	gsl_matrix * Bi = gsl_matrix_alloc(2, 2);
	gsl_matrix * Cpi = gsl_matrix_alloc(2, 2);
	gsl_matrix * Ji = gsl_matrix_alloc(2, 2);
	gsl_matrix * JiT = gsl_matrix_alloc(2, 2);
	gsl_matrix * res2x2 = gsl_matrix_alloc(2, 2);
	gsl_matrix * res2x2_2 = gsl_matrix_alloc(2, 2);
	gsl_matrix * CLR = gsl_matrix_alloc(2, 2);


	if (n_points < MIN_LINE_POINTS)
		return false;

	for (i = start; i < (end + 1); i++)
	{
		double mn = ScanObj->getSigmaR(i);
		double df = i;
		wi[j] = 1 / pow(ScanObj->getSigmaR(i), 2);
		xmw += wi[j] * ScanObj->getX(i);
		ymw += wi[j] * ScanObj->getY(i);
		sumWii += wi[j];
		j++;
	}
	// find weight average
	ymw = ymw / sumWii;
	xmw = xmw / sumWii;

	j = 0;
	for (i = start; i < (end + 1); i++)
	{
		Sxy += wi[j] * (ScanObj->getX(i) - xmw) * (ScanObj->getY(i) - ymw); // N
		Sx2 += wi[j] * pow(ScanObj->getX(i) - xmw, 2);
		Sy2 += wi[j] * pow(ScanObj->getY(i) - ymw, 2);
		++j;
	}
	N = -2 * Sxy;
	D = Sy2 - Sx2;

	line_alpha = 0.5 * atan2(N, D);
	line_r = xmw * cos(line_alpha) + ymw * sin(line_alpha);

	if (line_r < 0)
	{
		line_alpha = line_alpha + M_PI;
		line_r = -line_r;
	}
	if (line_alpha > M_PI)
		line_alpha = line_alpha - 2 * M_PI;

	if (line_alpha < -M_PI)
		line_alpha = line_alpha + 2 * M_PI;

	dr_alpha = ymw * cos(line_alpha) - xmw * sin(line_alpha);

	gsl_matrix_set_zero(CLR);
	for (i = start; i < (end + 1); i++)
	{
		dr_Sxy_yi = ScanObj->getX(i) - xmw;
		dr_Sxy_xi = ScanObj->getY(i) - ymw;
		dr_Sx2_yi = 2.0 * dr_Sxy_yi;
		dr_Sx2_xi = 2.0 * dr_Sxy_xi;

		nom = (ScanObj->getX(i) - xmw) * (Sy2 - Sx2) - 2 * Sxy * (ScanObj->getY(i)
		                                                                  - ymw);
		denom = pow(Sy2 - Sx2, 2) + 4 * pow(Sxy, 2);
		Ai22 = nom / denom;
		nom = (ScanObj->getY(i) - ymw) * (Sy2 - Sx2) + 2 * Sxy * (ScanObj->getX(i)
		                                                                  - xmw);
		denom = pow(Sy2 - Sx2, 2) + 4 * pow(Sxy, 2);
		Ai21 = nom / denom;
		Ai12 = sin(line_alpha) / sumWii - xmw * sin(line_alpha) * Ai22 + ymw
				* cos(line_alpha) * Ai22;
		Ai11 = cos(line_alpha) / sumWii - xmw * sin(line_alpha) * Ai21 + ymw
				* cos(line_alpha) * Ai21;

		gsl_matrix_set(Ai, 0, 0, Ai11);
		gsl_matrix_set(Ai, 0, 1, Ai12);
		gsl_matrix_set(Ai, 1, 0, Ai21);
		gsl_matrix_set(Ai, 1, 1, Ai22);

//		printgslMatrix ( Ai, 2, 2, "Ai");

		gsl_matrix_set(Bi, 1, 0, cos(ScanObj->getBearings(i)));
		gsl_matrix_set(Bi, 1, 1, -ScanObj->getRange(i) * sin(ScanObj->getBearings(i)));
		gsl_matrix_set(Bi, 0, 0, sin(ScanObj->getBearings(i)));
		gsl_matrix_set(Bi, 0, 1, ScanObj->getRange(i) * cos(ScanObj->getBearings(i)));

//		printgslMatrix ( Bi, 2, 2, "Bi");

		gsl_matrix_set(Cpi, 0, 0, pow(ScanObj->getSigmaR(i), 2));
		gsl_matrix_set(Cpi, 0, 1, 0);
		gsl_matrix_set(Cpi, 1, 0, 0);
		gsl_matrix_set(Cpi, 1, 1, pow(ScanObj->getSigmaPhi(i), 2));

//		printgslMatrix ( Cpi, 2, 2, "Cpi");

		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ai, Bi, 0.0, Ji);
		gsl_matrix_transpose_memcpy(JiT, Ji);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Ji, Cpi, 0.0, res2x2);
		gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, res2x2, JiT, 0.0,
				res2x2_2);
		gsl_matrix_add(CLR, res2x2_2);


//		printgslMatrix ( CLR, 2, 2, "PER POINT COV");
	}

	// update line parameters
	new_line->setLineParametersGSL(line_r, line_alpha, CLR);
	// update line end point ids
	new_line->setStartPoint(ScanObj->getX(start), ScanObj->getY(start), ScanObj->getBearings(start), ScanObj->getRange(start), start);
	new_line->setEndPoint(ScanObj->getX(end), ScanObj->getY(end), ScanObj->getBearings(end), ScanObj->getRange(end), end);



	cout << "LINE Alpha: " << line_alpha << "  R: " << line_r << endl;
	printgslMatrix ( CLR, 2, 2, "NEW LINE COVARIANCE");

	gsl_matrix_free(Ai);
	gsl_matrix_free(Bi);
	gsl_matrix_free(Cpi);
	gsl_matrix_free(Ji);
	gsl_matrix_free(JiT);
	gsl_matrix_free(res2x2);
	gsl_matrix_free(res2x2_2);
	gsl_matrix_free(CLR);

	return true;
}

/** RECURSIVE Methods to estimate lines **/

bool features_extractor_t::LineEstimator_RecursiveCornerDetectorSplitEstimateAndMergeLines(
		list<line_t> * ExtractedLines, laser_t * ScanObj, int start_point,
		int end_point)
{

	list<int> cornerInd;
	list<int>::iterator cornerIt;

	if ((end_point - start_point + 1) < MIN_LINE_POINTS)
	{
		return false;
	}

	if (Corners_LineValidationAndCornerDetection(ScanObj, start_point,
			end_point, &cornerInd))
	{
		// ramsac consensus shows that the points can be fitted directly to a line without breaking in subsets
		// or number of point is not big enough - with this we can estimate the final line parameters
		line_t LineEnd;
		bool retval = LineEstimator_FitLine(start_point, end_point, ScanObj,
				&LineEnd);
		if (retval)
		{
			cout << "PT END : " << LineEnd.getStartPointId() << " PT START: " <<LineEnd.getEndPointId() << endl;
			ExtractedLines->push_back(LineEnd);
		}
		return retval;
	}
	else
	{

		bool retval = true;

		int ind_str = start_point;
		int ind_end = end_point;
		cornerIt = cornerInd.begin();

		if (ind_end == *cornerIt)
			*cornerIt = *cornerIt - 1;
		if (start_point == *cornerIt)
			*cornerIt = *cornerIt + 1;

		for (int i = 0; i < cornerInd.size(); i++)
		{

			// recursive using the estimated corners
			retval
			= LineEstimator_RecursiveCornerDetectorSplitEstimateAndMergeLines(
					ExtractedLines, ScanObj, ind_str, *cornerIt);
			retval = retval | retval; // try to get one true
			ind_str = *cornerIt;
			cornerIt++;
		}

		// recursive using the estimated corners last segment
		retval
		= LineEstimator_RecursiveCornerDetectorSplitEstimateAndMergeLines(
				ExtractedLines, ScanObj, ind_str, end_point);
		retval = retval | retval; // try to get one true

		return retval;
	}
}

// Auxiliary methods
bool features_extractor_t::LinesEstimator_MergeContiguos(
		list<line_t> * listofLines, laser_t * ScanObj)
{

	double distance;
	double x_diff;
	double y_diff;
	double alpha_diff;
	double r_diff;
	bool erasePrev = false;

	list<line_t>::iterator lines_iter;
	list<line_t>::iterator lines_iter_next;
	list<line_t>::iterator prevIter;

	int i = 0;

	double line_diference = 0.0;
	lines_iter = (*listofLines).begin();
	if (listofLines->size() == 0)
		return 0;
	for (i = 0; i < ((listofLines)->size() - 1); i++)
	{
		if (erasePrev)
		{
			(*listofLines).erase(prevIter);
			erasePrev = false;
		}
		lines_iter_next = lines_iter;
		++lines_iter_next;
		alpha_diff = fabs(lines_iter->getLineAlpha()
				- lines_iter_next->getLineAlpha());
		r_diff = fabs(lines_iter->getLineR() - lines_iter_next->getLineR());
		line_diference = sqrt(alpha_diff * alpha_diff + r_diff * r_diff);

		//     cout << "<TRACE><LOG><features_extractor_t><LinesEstimator_MergeContiguos> Alpha 1: " << lines_iter->getLineAlpha() << " R1: " << lines_iter->getLineR() << endl;
		//     cout << "<TRACE><LOG><features_extractor_t><LinesEstimator_MergeContiguos> Alpha 2: " << lines_iter_next->getLineAlpha() << " R2: " << lines_iter_next->getLineR() << endl;

		if (alpha_diff < 0.0266 && r_diff < 0.025)
		{
			x_diff = lines_iter->getEndPointX() - lines_iter_next->getStartPointX();
			y_diff = lines_iter->getEndPointY() - lines_iter_next->getStartPointY();

			distance = sqrt(x_diff * x_diff + y_diff * y_diff);

			if (distance < MIN_DIST_BTW_LINES)
			{ //lines the same

				line_t newLine;

				int xstart = lines_iter->getStartPointId();
				int ystart = lines_iter_next->getEndPointId();

				if (LineEstimator_FitLine(xstart, ystart, ScanObj, &newLine))
				{
					// replace line next and erase the current line
					*lines_iter_next = newLine;
					erasePrev = true;
					prevIter = lines_iter;
				}
			}
		}

		lines_iter++;
	}

	// fix ids
	i = 0;
	for (lines_iter = (*listofLines).begin(); lines_iter
	!= (*listofLines).end(); lines_iter++)
	{
		lines_iter->setId(i);
		++i;
	}
	if ((*listofLines).size() < 2) //corresponde to minimum of tree lines
		return i + 1;

	return 0;
}

void features_extractor_t::LinesEstimator_SetEndPointsAttr(
		list<line_t> * listofLines, laser_t * ScanObj)
{

	int point_diff;

	list<line_t>::iterator lines_iter;
	list<line_t>::iterator lines_iter_next;
	list<line_t>::iterator lines_iter_previous;

	/* IMPLEMENTATION OF ALGORITHM 2 from proj report */

	for (lines_iter = (*listofLines).begin(); lines_iter
	!= (*listofLines).end(); lines_iter++)
	{
		lines_iter_next = lines_iter;
		++lines_iter_next;

		if (lines_iter == (*listofLines).begin())
		{ //first observed line
			//caracterize start point
			if (lines_iter->getStartPointId() < 5)
			{ //it's possible that the start point isn't the last point of the segment
				// 	cout << "Point Start Is Not Final" << endl;
				lines_iter->setStartPointAttr(NOT_FINAL_POINT);
			}
			else
			{ //start point fully caracterized, the line end's at the observed start point
				lines_iter->setStartPointAttr(FINAL_POINT);
				// 	cout << "Point Start Is Final" << endl;
			}

			//caracterize current line end point
			point_diff = lines_iter_next->getStartPointId() - lines_iter->getEndPointId();
			//       cout << "Point diff = " << point_diff << endl;
			if (abs(point_diff) < 5) //means that both lines are in adjacent points or near
			{

				if (IsABreakPoint_IsThereaBreakPoint(ScanObj,
						lines_iter->getEndPointId(),
						lines_iter_next->getStartPointId())) //is a break point so the end point mighty or not be the final point of the line
				{
					//we will test the distance from the points to the laser
					if (lines_iter->getEndPointR()
							> lines_iter_next->getStartPointR())
					{ //its the final point
						lines_iter->setEndPointAttr(NOT_FINAL_POINT);
						// 	cout << "Point Start Is NOt Final" << endl;
					}
					else
					{ //is not the final point
						lines_iter->setEndPointAttr(FINAL_POINT);
						// 	cout << "Point Start Is Final" << endl;
					}

				}
				else
				{ //is not break point so the end point is the final point
					lines_iter->setEndPointAttr(FINAL_POINT);
					// 	cout << "Point Start Is Final" << endl;
				}
			}
			else
			{ //both point are the final points from the lines
				lines_iter->setEndPointAttr(FINAL_POINT);
				// 	cout << "Point Start Is Final" << endl;
			}
		}
		else
		{ //other than first

			lines_iter_previous = lines_iter;
			--lines_iter_previous;
			if (lines_iter_next == (*listofLines).end()) //final line
			{ //only test the last line
				//caracterize start point

				if (abs(lines_iter->getEndPointId()
						- ScanObj->getNmbSamples()) < 5) //equal to 360
				{ //it's possible that the end point isn't the last point of the segment
					lines_iter->setEndPointAttr(NOT_FINAL_POINT);
					// 	cout << "Point Start Is Not Final" << endl;

				}
				else
				{ //start point fully caracterized, the line end's at the observed start point
					lines_iter->setEndPointAttr(FINAL_POINT);
					// 	cout << "Point Start Is Final" << endl;

				}
				point_diff = lines_iter->getStartPointId() - lines_iter_previous->getEndPointId();
				if (abs(point_diff) < 5) //means that both lines are in adjacent points
				{
					if (IsABreakPoint_IsThereaBreakPoint(ScanObj,
							lines_iter->getEndPointId(),
							lines_iter_next->getStartPointId()))
					{
						if (lines_iter->getStartPointR()
								< lines_iter_previous->getEndPointR())
						{
							lines_iter->setStartPointAttr(FINAL_POINT);
							// 	cout << "Point Start Is Final" << endl;
						}
						else
						{
							lines_iter->setStartPointAttr(NOT_FINAL_POINT);
							// 	cout << "Point Start Is Not Final" << endl;
						}
					}
					else
					{ //is a break point so the end point is not the final point

						lines_iter->setStartPointAttr(FINAL_POINT);
						// 	cout << "Point Start Is Final" << endl;
					}

				}
				else
				{ //both point are the final points from the lines
					lines_iter->setStartPointAttr(FINAL_POINT);
					// 	cout << "Point Start Is Final" << endl;
				}

			}
			else
			{ //test both previous and next line

				//caracterize current line end point
				int point_diff = lines_iter_next->getStartPointId() - lines_iter->getEndPointId();
				if (abs(point_diff) < 5) //means that both lines are in adjacent points
				{
					if (IsABreakPoint_IsThereaBreakPoint(ScanObj,
							lines_iter->getEndPointId(),
							lines_iter_next->getStartPointId()))
					{
						//we will test the distance from the points to the laser
						if (lines_iter->getEndPointR()
								> lines_iter_next->getStartPointR())
						{ //its the final point
							lines_iter->setEndPointAttr(NOT_FINAL_POINT);
							// 	cout << "Point Start Not Is Final" << endl;

						}
						else
						{ //is not the final point

							lines_iter->setEndPointAttr(FINAL_POINT);
							// 	cout << "Point Start Is Final" << endl;
						}
					}
					else
					{ //is a break point so the end point is not the final point
						lines_iter->setEndPointAttr(FINAL_POINT);
						// 	cout << "Point Start Is Final" << endl;
					}

				}
				else
				{ //both point are the final points from the lines

					lines_iter->setEndPointAttr(FINAL_POINT);
					// 	cout << "Point Start Is Final" << endl;
				}

				//caracterize current line start point
				point_diff = lines_iter->getStartPointId() - lines_iter_previous->getEndPointId();

				if (abs(point_diff) < 5) //means that both lines are in adjacent points
				{
					if (IsABreakPoint_IsThereaBreakPoint(ScanObj,
							lines_iter->getEndPointId(),
							lines_iter_next->getStartPointId()))
					{
						if (lines_iter->getStartPointR()
								< lines_iter_previous->getEndPointR())
						{

							lines_iter->setStartPointAttr(FINAL_POINT);
							// 	cout << "Point Start Is Final" << endl;
						}
						else
						{

							lines_iter->setStartPointAttr(NOT_FINAL_POINT);
							// 	cout << "Point Start Is Not Final" << endl;
						}

					}
					else
					{ //is not break point so the end point is not the final point

						lines_iter->setStartPointAttr(FINAL_POINT);
						// 	cout << "Point Start Is Final" << endl;
					}

				}
				else
				{ //both point are the final points from the lines
					lines_iter->setStartPointAttr(FINAL_POINT);
					// 	cout << "Point Start Is Final" << endl;

				}

			}

		}
	}
}
