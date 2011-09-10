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

#include "features_map_t.h"

features_map_t::features_map_t()
{

	Qj = gsl_matrix_alloc(2, 2); // 2x2
	gL = gsl_matrix_alloc(2, 3); // 2x3
	gLT = gsl_matrix_alloc(3, 2); // 3x2
	glTemp = gsl_matrix_alloc(2, 3); // 2x3
	Sij = gsl_matrix_alloc(2, 2); // 2x2
	InvSij = gsl_matrix_alloc(2, 2); // 2x2
	Vij = gsl_matrix_alloc(1, 2); // 1x2
	VijT = gsl_matrix_alloc(2, 1); // 2x1

	VijInvSij = gsl_matrix_alloc(1, 2); // 1x2
	Gres = gsl_matrix_alloc(1, 1); // 1x2; // 1x1

}
bool features_map_t::MahalanobisCheck(LinesPair_C * TestPair, gsl_matrix * cov)
{
	// get observed line covariance
	(TestPair)->Observed.getLineCovariance(Qj);

	// setup linearisation matrixes, gL and gLT, eq. 5.3
	gsl_matrix_set(gL, 0, 0, 0);
	gsl_matrix_set(gL, 0, 1, 0);
	gsl_matrix_set(gL, 0, 2, -1);
	gsl_matrix_set(gL, 1, 0, -cos((TestPair)->InMapLine.getLineAlpha()));
	gsl_matrix_set(gL, 1, 1, -sin((TestPair)->InMapLine.getLineAlpha()));
	gsl_matrix_set(gL, 1, 2, 0);
	// transpose matrix
	gsl_matrix_transpose_memcpy(gLT, gL);

	// Calculate Sij, eq 5.4
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, gL, cov, 0.0,
			glTemp);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, glTemp, gLT, 0.0,
			Sij);
	// add final component of the observation covariance
	gsl_matrix_add(Sij, Qj);

	/** Setup Inovation Vectot Vij
	 */
	gsl_matrix_set(Vij, 0, 0, (TestPair)->InMapLine.getLineR()
			- (TestPair)->Observed.getLineR());
	gsl_matrix_set(Vij, 0, 1, (TestPair)->InMapLine.getLineAlpha()
			- (TestPair)->Observed.getLineAlpha());
	gsl_matrix_transpose_memcpy(VijT, Vij);

	/**
	 Validation gate test - vij(k+1)*Sij(k+1)^-1*vij(k+1)^T <= g²
	 it can exist a case where all are scalar, and so the expression turns to

	 vij(k+1)²/Sij(k+1) <= g²

	 in our case we will use both angular predcition as distance prediction, so the
	 following expression used, eq 5.5
	 vιj (k + 1).Sιj (k + 1)−1 .vιj (k + 1)T ≤ g2
	 **/
	// calc of the s_k^-1

	CalcInvMatrix( InvSij, Sij, 2);
	// calc distance
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Vij, InvSij, 0.0,
			VijInvSij);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, VijInvSij, VijT,
			0.0, Gres);

	cout << "G: " << gsl_matrix_get(Gres,0,0) <<  endl;

	// setup innovation vector and return match test
	(TestPair)->InovVect[0] = gsl_matrix_get(Vij, 0, 0);
	(TestPair)->InovVect[1] = gsl_matrix_get(Vij, 0, 1);
	return (gsl_matrix_get(Gres, 0, 0) < pow(LINE_VALIDATION_GATE, 2));

}
bool features_map_t::MahalanobisCheck(FeaturesPair_C TestPair, gsl_matrix * cov)
{
	/** calculation if the innovation covariance Sij
	 Sιj (k + 1) = gι .R(k + 1|k). gT + Qj (k + 1)
	 gι .R(k + 1|k). gT, line in world uncertainty taking into account robot uncertainty
	 Qj (k + 1), line uncertainty in the world referential
	 */
	// check the type of both test pairs
	if (typeid(*((TestPair).Observed)).name() != typeid(*((TestPair).InMapFeat)).name() )
	{
		return false;
	}
	line_t lineTest;
	// do a line check test
	if ( typeid(*((TestPair).Observed)).name() == typeid(((lineTest))).name() ) {
		LinesPair_C LineTestPair;
		LineTestPair.Observed = *static_cast<line_t*>(TestPair.Observed);
		LineTestPair.InMapLine = *static_cast<line_t*>(TestPair.InMapFeat);
		if ( MahalanobisCheck(&LineTestPair, cov) ) {
			TestPair.Inovation.push_back(LineTestPair.InovVect[0]);
			TestPair.Inovation.push_back(LineTestPair.InovVect[1]);
			return true;
		}
	}
	return false;
}
bool features_map_t::FeaturesPairMatchCheck(FeaturesPair_C TestPair, gsl_matrix * cov)
{

	if ( MahalanobisCheck( TestPair, cov ) )
	{
		/*verify distance from the closest end points*/
		line_t lineTest;
		string type = typeid(*((TestPair).Observed)).name();
		if ( type == typeid(((lineTest))).name() ) {
			// do a check on end points of the lines
			if ( LineEndPointsCheck ( TestPair, cov ) ) {
				return true;
			}
		} else {


		}
	}
	return false;
}
bool features_map_t::PointInTheMiddleCheck(double x1,double y1,double x2,double y2,double xtest,double ytest)
{
	double ydiff = fabs(y1 - y2);
	double xdiff = fabs(x1 - x2);
	if(xdiff > ydiff)
	{//horizontal line
		if(xtest >= (x1) && xtest <= (x2) || xtest <=  (x1) && xtest >= (x2))
			return true;
		else
			return false;
	}
	else
	{//vertical line
		if(ytest >= (y1) && ytest <= (y2) || ytest <=  (y1) && ytest >= (y2))
			return true;
		else
			return false;
	}
}
bool  features_map_t::LineEndPointsCheck (FeaturesPair_C TestPair, gsl_matrix * cov)
{
	/*verify distance from the closest end points*/
	LinesPair_C LineTestPair;
	LineTestPair.Observed = *static_cast<line_t*>(TestPair.Observed);
	LineTestPair.InMapLine = *static_cast<line_t*>(TestPair.InMapFeat);

	double min_distance /*= obj->MAX_DIST_FOR_TWO_LINES_TO_BE_MERGED*M2INC*/;
	min_distance = 10000.0;
	double euclid;
	double obs_x_start = LineTestPair.Observed.getStartPointX();
	double obs_y_start = LineTestPair.Observed.getStartPointY();
	double obs_x_end = LineTestPair.Observed.getEndPointX();
	double obs_y_end = LineTestPair.Observed.getEndPointY();

	double pre_x_start = LineTestPair.InMapLine.getStartPointX();
	double pre_y_start = LineTestPair.InMapLine.getStartPointY();
	double pre_x_end = LineTestPair.InMapLine.getEndPointX();
	double pre_y_end = LineTestPair.InMapLine.getEndPointY();

	//if one of the observed points falls in the predicted points merge
	if(PointInTheMiddleCheck(pre_x_start,pre_y_start,pre_x_end,pre_y_end,obs_x_start,obs_y_start) ||
			PointInTheMiddleCheck(pre_x_start,pre_y_start,pre_x_end,pre_y_end,obs_x_end,obs_y_end)  ||
			PointInTheMiddleCheck(obs_x_start,obs_y_start,obs_x_end,obs_y_end,pre_x_start,pre_y_start) ||
			PointInTheMiddleCheck(obs_x_start,obs_y_start,obs_x_end,obs_y_end,pre_x_end,pre_y_end) )
		return true;

	struct AssociationEndPoint{
		int AttribOne;
		int AttribTwo;
		double euclid;
	} Min;
	euclid = sqrt((pre_x_end-obs_x_start)*(pre_x_end-obs_x_start) + (pre_y_end-obs_y_start)*(pre_y_end-obs_y_start));
	if(euclid < min_distance)
	{
		Min.AttribOne = LineTestPair.InMapLine.getEndPointAttr();
		Min.AttribTwo = LineTestPair.Observed.getStartPointAttr();
		Min.euclid = euclid;
		min_distance = euclid;
	}
	//observed end point
	euclid = sqrt((pre_x_end-obs_x_end)*(pre_x_end-obs_x_end) + (pre_y_end-obs_y_end)*(pre_y_end-obs_y_end));
	if(euclid  < min_distance)
	{
		Min.AttribOne = LineTestPair.InMapLine.getEndPointAttr();
		Min.AttribTwo = LineTestPair.Observed.getEndPointAttr();
		Min.euclid = euclid;
		min_distance = euclid;
	}
	euclid = sqrt((pre_x_start-obs_x_end)*(pre_x_start-obs_x_end) + (pre_y_start-obs_y_end)*(pre_y_start-obs_y_end));
	if(euclid < min_distance)
	{
		Min.AttribOne = LineTestPair.InMapLine.getStartPointAttr();
		Min.AttribTwo = LineTestPair.Observed.getEndPointAttr();
		Min.euclid = euclid;
		min_distance = euclid;
	}
	euclid = sqrt((pre_x_start-obs_x_start)*(pre_x_start-obs_x_start) + (pre_y_start-obs_y_start)*(pre_y_start-obs_y_start));
	if(euclid < min_distance)
	{
		//     cout << "four" << endl;
		Min.AttribOne = LineTestPair.InMapLine.getStartPointAttr();
		Min.AttribTwo = LineTestPair.Observed.getStartPointAttr();
		Min.euclid  = euclid;
		min_distance = euclid;
	}
	if(Min.euclid <= min_distance)
	{
		if(Min.AttribOne  == NOT_FINAL_POINT && Min.AttribTwo  == NOT_FINAL_POINT)
		{
			return true;
		}
	}

	return false;
}

void features_map_t::printFeatures (string id) {
	int i = 0;
	boost::ptr_list<feature_t>::iterator featItWorld;
	cout << "PRINTING: " <<  id  << endl;
	for (featItWorld = MapofFeatures.begin(); featItWorld
	!= MapofFeatures.end(); featItWorld++)
	{
		line_t Observed1;
		if ( typeid((*featItWorld)).name() == typeid((Observed1)).name() ) {
			line_t * Observed = static_cast<line_t*>(&*featItWorld);
			cout << "i: " << i
					<< " r = " << Observed->getLineR()
					<< " alpha = " << Observed->getLineAlpha()
					<< " Maturity: " << (*featItWorld).getMaturity () << endl;

			gsl_matrix * Gg = gsl_matrix_alloc ( 2, 3);
			gsl_matrix * QK = gsl_matrix_alloc(2, 2);
			int FeaturePoseGradIndex = 0;
			(*featItWorld).getPoseGradient(Gg,&FeaturePoseGradIndex);
			FeaturePoseGradIndex = 0;
			(*featItWorld).getCovariance(QK,&FeaturePoseGradIndex);

			printgslMatrix ( Gg, 2 , 3, "Gg");
			printgslMatrix ( QK, 2 , 2 , "QK");

			gsl_matrix_free(Gg);
			gsl_matrix_free(QK);
		}
		++i;
	}

}
// remove features from map read maturity to zero
void features_map_t::RemoveFalsePositives () {
	boost::ptr_list<feature_t>::iterator  MapofIt;
	boost::ptr_list<feature_t>::iterator  MapofItNext;

	// remove features from map read maturity to zero
	MapofIt = MapofFeatures.begin();
	while ( MapofIt != MapofFeatures.end() && MapofFeatures.size ()  > 0)
	{
		MapofItNext = MapofIt;
		MapofItNext++;

		if ( (*MapofIt).getMaturity () == 0 && !(*MapofIt).isMatch () ) {
			MapofFeatures.release(MapofIt);
			if ( MapofFeatures.size () > 0)
				MapofIt = MapofItNext;
		} else {
			if ( !(*MapofIt).isMatch () ) {
				(*MapofIt).decreaseMaturity();
			}
			MapofIt++;
		}
	}
}
void features_map_t::OptimizeListOfFeatures () {
	boost::ptr_list<feature_t>::iterator  MapofIt;
	boost::ptr_list<feature_t>::iterator  MapofItNext;

	bool notEnd = true;
	MapofIt = MapofFeatures.begin();
	while ( MapofIt != MapofFeatures.end() && notEnd )
	{
		bool bringToFront = false;
		if ( (*MapofIt).isMatch () ) {
			bringToFront = true;

			(*MapofIt).setMatchFlag (false);

			if ( bringToFront ) {
				MapofItNext = MapofIt;
				MapofItNext++;
				line_t Observed;
				if ( typeid((*MapofIt)).name() == typeid((Observed)).name() ) {
					line_t * Observed = new line_t();
					*Observed = *static_cast<line_t*>(&*MapofIt);
					MapofFeatures.push_front (Observed);
					MapofFeatures.release(MapofIt);
				}
				MapofIt = MapofItNext;
			}
		} else {
			MapofIt++;
		}
	}
}
bool features_map_t::AddFeatureToMap ( feature_t * FeatureToAdd ) {

	line_t Observed1;
	if ( typeid((*FeatureToAdd)).name() == typeid((Observed1)).name() ) {
		line_t * Observed = new line_t();
		*Observed = *static_cast<line_t*>(&*FeatureToAdd);
		Observed->increaseMaturity();
		Observed->setMatchFlag ( true );
		MapofFeatures.push_back( Observed );
	}
	return true;
}

