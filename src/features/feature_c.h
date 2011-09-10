/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) <year>  <name of author>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef FEATURE_T_H
#define FEATURE_T_H

#include <gsl/gsl_matrix.h>

#define LINE_FEATURE 1
#define POINT_FEATURE 2
#define CORNER_FEATURE 3
#define CIRCLE_FEATURE 4

#define MATURITY_LEVEL 5

class feature_t
{
public:
  virtual ~feature_t() = 0;

  // methods to do frame transformation
  virtual bool toGlobalCordinates ( gsl_matrix * Mean, gsl_matrix * Cov ) = 0;
  virtual bool toLocalCordinates ( gsl_matrix * Mean, gsl_matrix * Cov ) = 0;

  // statistical methods
  virtual void getPoseGradient ( gsl_matrix * GradOut, int * index_i ) = 0;
  virtual void getCovariance ( gsl_matrix * GradOut, int * index_i ) = 0;
  virtual void getCovarianceDiagonalElems ( gsl_matrix * GradOut, int * index_i ) = 0;

  // merge methods
  virtual void mergeWithFeature ( feature_t * featIn) = 0;

  // properties of the feature
  virtual int getMaturity () = 0;
  virtual int getFeatureType () = 0;
  virtual bool isMature () = 0;
  virtual void increaseMaturity ( ) = 0;
  virtual void decreaseMaturity ( ) = 0;
  virtual bool isMatch () = 0;
  virtual void setMatchFlag (bool) = 0;

  virtual int getId () = 0;
  virtual void setId (int id) = 0;

};



#endif // FEATURE_T_H
