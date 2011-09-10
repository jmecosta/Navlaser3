/*
 * math_lib.h
 *
 *  Created on: 26 Mar 2011
 *      Author: jmecosta
 */

#ifndef MATH_LIB_H_
#define MATH_LIB_H_

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_sort.h>

#include <iostream>

struct Determinant
{
	gsl_matrix * matrix; //!original matrix
	gsl_matrix * inverse; //!inverse matrix
	gsl_matrix * ludecomp; //!LU decomposition
	int size; //!number of electrons
	int spin; //!spin type of the determinant
	double det; //! determinant
	gsl_permutation * perm; //! permutation matrix
};

/*****************************************************************/
/*								 */
/*		geometry operation			 	 */
/*								 */
/*****************************************************************/
/* Calcs intersection point between a line defined in polar coordinates and a prepedicular line that contains a given point */
void CalcInterSectionPointBtwLineAndPoint(double alpha, double r,
		double xyIn[2], double * xyOut);

/*****************************************************************/
/*								 */
/*		Matrix operation			 	 */
/*								 */
/*****************************************************************/
double CalcInvMatrix(gsl_matrix * invMat, gsl_matrix * NonInvMat, int size );
bool set_determinant_via_LU_decomp(Determinant * D, bool CALC_INV);
void printgslMatrix(gsl_matrix * Mat, int lines, int colons, char*  name);
/*****************************************************************/
/*								 */
/*		Statistical functions			 	 */
/*								 */
/*****************************************************************/
/*
 * This functions gets the parameters of an uncertanty elipse described by a matrix
 * @param Pk uncertanty elipse matrix
 * @param number_sigmas number of sigmas to describe the elipse
 * @param out elipse parameters
 */
void GetUncertantyElipse(gsl_matrix * Pk, int number_sigmas, double * elipse);

/* median filter, using a array of samples apply a media filter */
void median_filter_points(double * array_r, int DATA_SIZE, int mediaWindow);

#endif /* MATH_LIB_H_ */
