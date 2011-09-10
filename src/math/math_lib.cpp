/*
 * math_lib.cpp
 *
 *  Created on: 26 Mar 2011
 *      Author: jmecosta
 */

#include "math_lib.h"

#include <math.h>

using namespace std;
bool set_determinant_via_LU_decomp(Determinant * D, bool CALC_INV)
{
	int s;
	gsl_matrix_memcpy(D->ludecomp, D->matrix);
	gsl_linalg_LU_decomp(D->ludecomp, D->perm, &s);
	D->det = gsl_linalg_LU_det(D->ludecomp, s);
	if (CALC_INV)
	{
		gsl_linalg_LU_invert(D->ludecomp, D->perm, D->inverse);
	}
	return true;
}
double CalcInvMatrix(gsl_matrix * invMat, gsl_matrix * NonInvMat, int size )
{
	double det;

	/* matrix invertion Sk */
	Determinant * d = new Determinant();
	d->ludecomp = gsl_matrix_calloc(size, size);
	d->perm = gsl_permutation_alloc(size);

	d->matrix = NonInvMat;
	d->inverse = invMat;
	d->size = size;
	d->spin = 1;
	d->det = 0;

	set_determinant_via_LU_decomp(d, true);
	invMat = d->inverse; //matrix_cal

	gsl_permutation_free(d->perm);
	gsl_matrix_free(d->ludecomp);
	delete d;

	return det;
}
void printgslMatrix(gsl_matrix * Mat,int lines,int colons, char * name)
{

	cout << name << " = [" ;
	for(int i=0;i<lines;i++)
	{
		cout << "[";
		for(int j=0;j<colons;j++)
		{
			cout << gsl_matrix_get(Mat,i,j) << " " ;

		}
		if(i == lines-1)
			cout << "]]" << endl;
		else
			cout << "]" << endl;
	}


}
/*****************************************************************/
/*								 */
/*		Statistical functions			 	 */
/*								 */
/*****************************************************************/
/*!
	This function is use to calculate the elipse related to the pose uncertainty
	it as a degree of confidence of 95 %, the position covaricance matrix is define
	as Pk, we are going only calculate the elipse for the x,y position, representing
	the submatrix 2x2 corresponding to (x,y)

	Pk=[sxx,sxy,sxt][syx,syy,syt][stx,sty,stt]

	lambda1 = (1/2)*[sxx+syy+sqrt((sxx-syy)^2+4*sxy^2)]	//major axis
	lambda2 = (1/2)*[sxx+syy-sqrt((sxx-syy)^2+4syy^2)]	//minor axis
	alpha = (1/2)*atan2((2*sxy)/(sxx-syy))

	-PI/4<= alpha <= PI/4 , sxx != syy

	number_sigmas = 1 -> elipse covers 39.3 % of the data
	number_sigmas = 2 -> elipse covers 86.5 % of the data
	number_sigmas = 3 -> elipse covers 98.9 % of the data

 */
void GetUncertantyElipse(gsl_matrix * Pk, int number_sigmas, double * elipse)
{

	double Sxx = gsl_matrix_get(Pk, 0, 0);
	double Sxy = gsl_matrix_get(Pk, 0, 1);
	double Syy = gsl_matrix_get(Pk, 1, 1);
	double sqrt_value = sqrt((Sxx - Syy) * (Sxx - Syy) + 4 * Sxy * Sxy);

	double lambda1 = 0.5 * (Sxx + Syy + sqrt_value);
	double lambda2 = 0.5 * (Sxx + Syy - sqrt_value);

	elipse[0] = 0.5 * atan2(2 * Sxy, (Sxx - Syy)); // slope
	elipse[1] = 2 * number_sigmas * sqrt(lambda2); // minor axis
	elipse[2] = 2 * number_sigmas * sqrt(lambda1); // major axis

}
void median_filter_points(double * array_r, int DATA_SIZE, int mediaWindow)
{

	double data_r[mediaWindow];
	int medWind = mediaWindow / 2;

	for (int i = 0; i < DATA_SIZE; i++)
	{
		if (i > medWind && i < (DATA_SIZE - medWind))
		{
			for (int j = 0; j < mediaWindow; j++)
			{
				data_r[j] = array_r[i - medWind + j];
			}
			gsl_sort(data_r, 1, mediaWindow);
			array_r[i] = gsl_stats_median_from_sorted_data(data_r, 1,
					mediaWindow);
		}
	}

}
/*****************************************************************/
/*								 */
/*		Geometry functions			 	 */
/*								 */
/*****************************************************************/
void CalcInterSectionPointBtwLineAndPoint(double alpha, double r,
		double xyIn[2], double xyOut[])
{
	double p_st = xyIn[1] * sin(alpha) + xyIn[0] * cos(alpha) - r;
	xyOut[0] = xyIn[0] + p_st*cos(alpha);
	xyOut[1] =  xyIn[1]  +   p_st*sin(alpha);
	double psss = xyOut[1] * sin(alpha) + xyOut[0] * cos(alpha) - r;
	if(fabs(psss) > fabs(p_st))
	{
		xyOut[0] = xyIn[0] - p_st * cos(alpha);
		xyOut[1] = xyIn[1] - p_st * sin(alpha);
	}
}
