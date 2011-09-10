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
#include "nav_motionestimator.h"
#include "../math/math_lib.h"

Nav_MotionEstimator::Nav_MotionEstimator(int modeltoUse, string dev)
{

#ifdef USEGAZEBO

	int serverId = 0;
	// start client
	try {
		client = new Client();
		client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
		client->Connect(serverId);
	} catch (std::string e) {
		std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
		fflush( stdout);
		throw e;
	}
	// Create and subscribe to a position
	posIface = new PositionIface();
	/// Open the global control stuff
	try	{
		posIface->Open(client, "pioneer2dx_model1::position_iface_0");
	} catch (std::string e) {
		delete posIface;
		std::cerr << "Gazebo error: Unable to connect to an  interface\n" << e
				<< std::endl;
		throw e;
	}
	// start laser
	ScanObj = new lmsScan_t(client, "pioneer2dx_model1::laser::laser_iface_0");
#elif defined PLAYERPLUGGING
#else
	// Create a client and connect it to the server.
	client = playerc_client_create(NULL, "localhost", 6665);
	if (0 != playerc_client_connect(client))
		return;
	// Create and subscribe to a position2d device.
	position2d = playerc_position2d_create(client, 0);
	if (playerc_position2d_subscribe(position2d, PLAYER_OPEN_MODE))
		return;
	playerc_position2d_enable (position2d, true);
	// start laser
	ScanObj = new lmsScan_t(client, 1);
#endif


	// alloc pose and init values
	MeanPose = gsl_matrix_alloc(3, 1);
	CovPose = gsl_matrix_alloc(3, 3);
	gsl_matrix_set_zero(MeanPose);
	gsl_matrix_set_zero(CovPose);

	// alloc matrix to speed up the process
	Nk = gsl_matrix_alloc(2, 2);
	Fx = gsl_matrix_alloc(3, 3); // 3x2
	FxT = gsl_matrix_alloc(3, 3); // 3x3
	Fu = gsl_matrix_alloc(3, 2); // 3x2
	FuT = gsl_matrix_alloc(2, 3); // 2x3
	Qk = gsl_matrix_alloc(3, 3); //Q(k)
	Qk_temp = gsl_matrix_alloc(3, 2);
	Pk_temp = gsl_matrix_alloc(3, 3); // temp
	res3x3 = gsl_matrix_alloc(3, 3); // temp

	// setup model to use
	MODEL_TYPE = modeltoUse;

	// setup a features extractor object
	setBreakPointMethod(ADPTBREAKMETHOD, 3.0);
	setCornerEstimationMethod(CORNERRAMSAC, 0.025);
	setLineEstimationMethod(LINELEASTSQUAREMETHOD);
	setMinDistanceBtwLines(5);
	setMinLinePoints(5);


	// launch control thread
	Nav_start (0);
}
Nav_MotionEstimator::~Nav_MotionEstimator()
{
	cout << "<TRACE><LOG><Nav_MotionEstimator><Nav_start> Enabling Motors "
			<< "and - Launchin Control Thread " << endl;
}
void Nav_MotionEstimator::Nav_start(int N)
{
	cout << "<TRACE><LOG><Nav_MotionEstimator><Nav_start> Enabling Motors "
			<< "and - Launchin Control Thread " << endl;

    m_Thread = boost::thread(&Nav_MotionEstimator::Nav_RunEFKMethod, this, N);

}
void Nav_MotionEstimator::Nav_join()
{
	m_Thread.join();

	cout
	<< "<TRACE><LOG><Nav_MotionEstimator><Nav_join> Destroy Laser Object "
	<< endl;
	// Disable and delete elements the motor
	delete ScanObj;
	sleep (1);
	cout
	<< "<TRACE><LOG><Nav_MotionEstimator><Nav_join> Disable Motors and close connection to Server "
	<< endl;

#ifdef USEGAZEBO

#elif defined PLAYERPLUGGING
#else
	playerc_position2d_enable (position2d, false);
	playerc_position2d_unsubscribe ( position2d );
	playerc_position2d_destroy ( position2d );
	playerc_client_disconnect(client);
	playerc_client_destroy(client);
#endif

}
void Nav_MotionEstimator::Nav_RunEFKMethod(int N)
{
	list<feature_t>::iterator featIt;

	double elipseUni[3] =
	{ 0, 0, 0 };
	double Vin[2] =
	{ 0, 0 };
	double T = 0.5;
	for (int count = 0; count < 50; count++)
	{
		// setup current velocity commands
		if ( count > 20 ) {
			Vin[0] = 0.3;
			Vin[1] = 0.0;
		}
		// send commands to the simulator/robot
		// this task will wait for the period control to expire
		Nav_CommandRobot(&Vin[0], T);

		// First Step is to update the the sensor measurements
		ScanObj->UpdateScan();

		// Stage 1 Predict the position based on the velocity input
		Nav_EKFStage1MovPredictor( &Vin[0], 0.5, &elipseUni[0]);

		// Stage 1 Get current observation from laset
		Nav_EKFStage1Observation ( );

		// print Observation
		printObservedFeatures("OBSERVED FEATURES");

		// Stage 2 Match current observation with the Map of features present in the complete Map
		Nav_EKFStage2Matching( );

		// Stage 3 of EKF, correction of position based on observation and movement estimation
		Nav_EKFStage3Correction( );

		// Stage 4 of EKF, after the position is corrected
		Nav_EKFStage4MapUpdate( );

		// clear previous observed features
		if ( FeaturesObsLocalFrame.size () > 0)
			FeaturesObsLocalFrame.clear ();
		if ( FeaturesObs.size () > 0)
			FeaturesObs.clear ();
		if ( FeaturesNonMatched.size () > 0 )
			FeaturesNonMatched.clear ();
		if ( FeatMatched.size () > 0 )
			FeatMatched.clear ();

		printgslMatrix(MeanPose,3,1,"<TRACE><LOG><Nav_MotionEstimator><Nav_execution> POSE");
		printgslMatrix(CovPose,3,3,"<TRACE><LOG><Nav_MotionEstimator><Nav_execution> COV");

	}

}
/*****************************************************************/
/*								 								*/
/*		Simulator/Robot Control Methods		 				*/
/*								 */
/*****************************************************************/
void Nav_MotionEstimator::Nav_CommandRobot(double * SpeedInput, double T)
{
	struct timeval Start_t, End_t;
	double mtime, seconds, useconds;
	gettimeofday(&Start_t, NULL);

	// lock position inputs
#ifdef USEGAZEBO
	// lock position inputs
	posIface->Lock(1);
	posIface->data->cmdVelocity.pos.x = SpeedInput[0];
	posIface->data->cmdVelocity.yaw = SpeedInput[1];
	posIface->Unlock();
#elif defined PLAYERPLUGGING
#else
	cout << "V: SpeedInput[0] " << SpeedInput[0] << " I : " << SpeedInput[1] << endl;
	playerc_position2d_set_cmd_vel(position2d, SpeedInput[0], 0, SpeedInput[1], 1);
#endif

	usleep(T * 1000000);
	gettimeofday(&End_t, NULL);
	seconds = End_t.tv_sec - Start_t.tv_sec;
	useconds = End_t.tv_usec - Start_t.tv_usec;
	mtime = (((seconds) * 1000 + useconds / 1000.0) + 0.5) / 1000;

}

/*****************************************************************/
/*								 */
/*		Extended Kalman Filter Functions		 */
/*								 */
/*****************************************************************/
void Nav_MotionEstimator::Nav_EKFStage1MovPredictor( double * SpeedInput, double T, double * Pk_1_elipse)
{
	/**
	 each pose is described by its mean and its covariance, the first stage of the extended kalman filter
	 shares the same theory as the normal kalman filter. Its prediction stage uses the state k, with its mean U(k) and its
	 covariance P(K) to estimate the U(k+1|k) and P(K+1|k)
	 */
	double MeanVect[3] =
	{ gsl_matrix_get(MeanPose, 0, 0), gsl_matrix_get(MeanPose, 1, 0), gsl_matrix_get(
			MeanPose, 2, 0), };

	/** @a Main Kalman Algorithm - Calc of the pose U(k+1|k)*/
	switch (MODEL_TYPE)
	{

	case NOMAD_MODEL:
		try
		{
			Nav_PosePropagator_Nomad200(SpeedInput, T, &MeanVect[0]);

			/** @a Main Kalman Algorithm - Calc of the position and orientation incertanty P(k+1|k) */
			Nav_PKPropagator_Nomad200( SpeedInput, T);
		} catch (std::string e)
		{
			std::cerr << "NOMAD MODEL ERROR: \n" << e << "\n";
			return;
		}
		break;

	case UNICYCLE_MODEL:
		try
		{
			Nav_PosePropagator_Unicycle(SpeedInput, T, &MeanVect[0]);

			/** @a Main Kalman Algorithm - Calc of the position and orientation incertanty P(k+1|k) */
			Nav_PKPropagator_Unicycle(SpeedInput, T);
		} catch (std::string e)
		{
			std::cerr << "UNICYCLE MODEL ERROR: \n" << e << "\n";
			return;
		}
		break;

	default:
		break;

	}

	// update pose mean
	gsl_matrix_set(MeanPose, 0, 0, MeanVect[0]), gsl_matrix_set(MeanPose, 1, 0,
			MeanVect[1]), gsl_matrix_set(MeanPose, 2, 0, MeanVect[2]),

			// this is only to provide a array to plot the uncertanty elipse
			GetUncertantyElipse(CovPose, 3, Pk_1_elipse);
}
void Nav_MotionEstimator::Nav_EKFStage1Observation( )
{

	// first extract all the lines in the samples
	list<line_t> lines;

	features_LineExtractor(ScanObj, &lines);

	list<line_t>::iterator linesIt;
	for (linesIt = lines.begin(); linesIt != lines.end(); linesIt++)
	{
		line_t * newLine = new line_t();
		line_t * newLine1 = new line_t();
		*newLine1 = (*linesIt);
		*newLine = (*linesIt);

		// tbd do id
		newLine->setId(FeaturesObs.size());
		newLine1->setId(FeaturesObs.size());
		FeaturesObs.push_back(newLine);
		FeaturesObsLocalFrame.push_back (newLine1);
		//cout << " (**featItObs). " << typeid(tmp1).name() << endl;
	}
	// extract all circles
	// tbd
	// Transform current observation into the WORLD referetial
	cout << "<TRACE><LOG><Nav_MotionEstimator><Nav_EKFStage1Observation> Lines Nb: "
			<< FeaturesObs.size() << endl;

}
// Match current observation with the Map of features present in the complete Map
void Nav_MotionEstimator::Nav_EKFStage2Matching( )
{
	bool newFeature;

	boost::ptr_list<feature_t>::iterator featItObs;
	boost::ptr_list<feature_t>::iterator featItWorld;

	cout << "MATCHING OBS: " << FeaturesObs.size() << endl;
	cout << "in map: " << MapofFeatures.size() << endl;
	if (FeaturesObs.size() == 0)
	{
		return;
	}
	// go through the current observation and do matching of features
	for (featItObs = FeaturesObs.begin(); featItObs != FeaturesObs.end(); featItObs++)
	{
		// convert to global coordinates
		(*featItObs).toGlobalCordinates(MeanPose, CovPose);
		newFeature = true;
		cout << "NEXT OBSERVATION " << endl;
		featItWorld = MapofFeatures.begin();
		bool FeatFound = false;
		while ( !FeatFound  && featItWorld != MapofFeatures.end() && (MapofFeatures.size() > 0))
		{
			FeaturesPair_C TestPair;
			TestPair.Observed = &*featItObs;
			TestPair.InMapFeat = &*featItWorld;
			// check to see if we have a match
			if ( FeaturesPairMatchCheck( TestPair, CovPose ) )
			{
				// there was a match
				MatchedFeatures_C * NewMatchPair = new MatchedFeatures_C();
				line_t testLine;
				if ( typeid((*TestPair.Observed)).name() == typeid((testLine)).name() ) {

					// increase maturity level
					TestPair.InMapFeat->increaseMaturity();
					TestPair.InMapFeat->setMatchFlag (true);

					// copy to temp lists
					line_t * Observed = new line_t();
					*Observed = *static_cast<line_t*>(&*TestPair.Observed);
					line_t * InMap = new line_t();
					*InMap = *static_cast<line_t*>(&*TestPair.InMapFeat);
					// increase the maturity of the feature
					NewMatchPair->Observed = Observed;
					NewMatchPair->InMapFeat = InMap;
					NewMatchPair->InovVect.insert(NewMatchPair->InovVect.begin(),
							TestPair.Inovation.begin(),
							TestPair.Inovation.end());

					FeatMatched.push_back(NewMatchPair);

				}


				newFeature = false;
				FeatFound = true;
			}
			else
			{
				newFeature = true;
			}
			// go to next feature in the world
			featItWorld++;
		}
		if (newFeature)
		{
			// add feature to temporary list of new features
			line_t Observed1;
			if ( typeid((*featItObs)).name() == typeid((Observed1)).name() ) {
				line_t * Observed = new line_t();
				*Observed = *static_cast<line_t*>(&*featItObs);
				// increase the maturity of the feature
				Observed->increaseMaturity();
				Observed->setMatchFlag ( true );
				FeaturesNonMatched.push_back( Observed );
			}
		}
	}

}
/** @a Main Kalman Algorithm - Final process Update position and update P(k+1|k+1)
 * 	/*  General correction algorithm equations, as per main report document
 *
 *
	Correction equations:

	1) Pose correction
		x(k + 1|k + 1) = x(k + 1|k) + w(k + 1).v(k + 1)   	(5.10)

			 where:
				 x(k + 1|k), predicted pose from Stage 1 of the EKF
				 W(k + 1), Kalman Gain
				 v(k + 1), innovation vector obtained from
						   feature matching stage

			definition of the terms of the equation 5.10:

				w(k + 1) = R(k + 1|k).GgT(k + 1).S^−1(k + 1)			 	(5.11)

				v(k + 1) = [ v1(k+1) ]    									(5.6)
						   [ v2(k+1) ]
							   ...
						   [ vn(k+1) ]
				where:
					R(k + 1|k), pose covariance matrix
					Gg(k + 1), aggregated pose gradient matrix obtained from
							   feature prediction stage
					S(k + 1), aggregated feature covariance

			definition of the terms of the equation 5.11:

					Gg(k + 1) = [ Gg1(k+1) ]    	(5.7)
								[ Gg2(k+1) ]
									...
								[ Ggn(k+1) ]

					S(k + 1) = Gg(k+1).R(k + 1|k).GgT(k+1) + Q(k + 1) (5.8)

						where:
							Q(k + 1), aggregated feature covariace matrix

								Q(k + 1) = [ D1  	0 	  ... 	0  ]
										   [ 0 		D2 	  ... 	0  ]
										   [ ...  	...   ...	...]
										   [ 0  	0     ...	Dn ]

								Dn, diagonal elements of the features covariace

	2) Covariance correction
		R(k + 1|k + 1) = R(k + 1|k) − w(k + 1).S(k + 1).wT(k + 1) 	(5.12)


 **/
void Nav_MotionEstimator::Nav_EKFStage3Correction( )
{
	// features matched during
	boost::ptr_list<MatchedFeatures_C>::iterator MatchedIterator;

	int Featincorrection = 0;

	cout << "FEATURE MATCHED: " << FeatMatched.size() << endl;
	/** Count the number of mature features that can be used in the correction */
	for (MatchedIterator = FeatMatched.begin(); MatchedIterator != FeatMatched.end(); MatchedIterator++)
	{
		if ( (*MatchedIterator).InMapFeat->isMature () ) {
			Featincorrection++;
		}
	}

	cout << " CORRECTION WITH : " << Featincorrection << endl;

	if ( Featincorrection == 0 ) return;

	/* Allocate matrixes */
	gsl_matrix * Vk = gsl_matrix_alloc ( 2*Featincorrection, 1);
	gsl_matrix * Gg = gsl_matrix_alloc ( 2 * Featincorrection, 3);
	gsl_matrix * GgT = gsl_matrix_alloc ( 3, 2 * Featincorrection );
	gsl_matrix * W	= gsl_matrix_alloc ( 3, 2 * Featincorrection );
	gsl_matrix * WT	= gsl_matrix_alloc ( 2 * Featincorrection, 3 );

	gsl_matrix * QK = gsl_matrix_alloc(Featincorrection * 2, Featincorrection * 2);
	gsl_matrix * SK = gsl_matrix_alloc(Featincorrection * 2, Featincorrection * 2);
	gsl_matrix * SKinv = gsl_matrix_alloc(Featincorrection * 2, Featincorrection * 2);

	gsl_matrix * mat2nx3 = gsl_matrix_alloc(Featincorrection * 2, 3);
	gsl_matrix * mat3xn2 = gsl_matrix_alloc(3, Featincorrection * 2);
	gsl_matrix * mat3x3 = gsl_matrix_alloc(3, 3);
	gsl_matrix * mat3x1 = gsl_matrix_alloc(3, 1);

	// fill the matrixes with the relevant data
	int InnovIndex = 0;
	int AggreFeatIndex = 0;
	int FeaturePoseGradIndex = 0;
	for (MatchedIterator = FeatMatched.begin(); MatchedIterator != FeatMatched.end(); MatchedIterator++)
	{
		if ( (*MatchedIterator).InMapFeat->isMature () ) {
			// get position grandient, from observed predicted feature
			(*MatchedIterator).Observed->getPoseGradient(Gg,&FeaturePoseGradIndex);
			// set innovation vector
			for (int i=0;i<(*MatchedIterator).InovVect.size();i++) {
				gsl_matrix_set (Vk, InnovIndex, 0, (*MatchedIterator).InovVect[i] );
				InnovIndex++;
			}
			// aggregated feature covariance matrix
			(*MatchedIterator).Observed->getCovarianceDiagonalElems (QK,&AggreFeatIndex);
		}
	}
	// transpose the matrix GT
	gsl_matrix_transpose_memcpy(GgT, Gg);

	/* Pose Correction of EKF
	 *
	 * S(k + 1) = Gg(k+1).R(k + 1|k).GgT(k+1) + Q(k + 1) (5.8)
	 * */
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Gg, CovPose, 0.0, mat2nx3);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat2nx3, GgT, 0.0, SK);
	gsl_matrix_add(SK,QK);

	printgslMatrix ( Gg, 2*Featincorrection , 3, "Gg");
	printgslMatrix ( QK, 2*Featincorrection , 2*Featincorrection, "QK");
	printgslMatrix ( SK, 2*Featincorrection , 2*Featincorrection, "SK");

	/* matrix inversion Sk */
	CalcInvMatrix(SKinv, SK, Featincorrection * 2 );

	/* Kalman gain
	 * w(k + 1) = R(k + 1|k).GgT(k + 1).S^−1(k + 1)			 	(5.11)
	 * */
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, CovPose, GgT, 0.0, mat3xn2);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat3xn2, SKinv, 0.0, W);

	// transpose the Gain
	gsl_matrix_transpose_memcpy(WT, W);

	/* Pose Correction eq.
	 * x(k + 1|k + 1) = x(k + 1|k) + w(k + 1).v(k + 1)   	(5.10)
	 * */
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, W, Vk, 0.0, mat3x1);
	gsl_matrix_add(MeanPose,mat3x1);

	/* Covariance Correction eq.
	 * R(k + 1|k + 1) = R(k + 1|k) − w(k + 1).S(k + 1).wT(k + 1) 	(5.12)
	 * */
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, W, SK, 0.0, mat3xn2);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, mat3xn2, WT, 0.0, mat3x3);
	gsl_matrix_sub(CovPose,mat3x3);

	// free matrix elements
	gsl_matrix_free(Vk);
	gsl_matrix_free(Gg);
	gsl_matrix_free(GgT);
	gsl_matrix_free(W);
	gsl_matrix_free(WT);
	gsl_matrix_free(QK);
	gsl_matrix_free(SK);
	gsl_matrix_free(SKinv);
	gsl_matrix_free(mat2nx3);
	gsl_matrix_free(mat3xn2);
	gsl_matrix_free(mat3x3);
	gsl_matrix_free(mat3x1);

} //end update position
/** @a Main Kalman Algorithm - Map Update
 * General update method description:
 * 1 - new features, using the corrected pose add to map
 * 2 - matched features, update the map with the new corrected pose
 * */
void Nav_MotionEstimator::Nav_EKFStage4MapUpdate ( ) {

	// features matched during
	boost::ptr_list<MatchedFeatures_C>::iterator MatchPairIt;
	boost::ptr_list<feature_t>::iterator FeatNonMatchIt;
	boost::ptr_list<feature_t>::iterator FeatObsLocalFrameIt;
	bool FeatureFound = false;

	if (FeaturesObsLocalFrame.size() == 0)
	{
		return;
	}
	/* Add new features to map */
	for (FeatNonMatchIt = FeaturesNonMatched.begin();
			FeatNonMatchIt != FeaturesNonMatched.end();
			FeatNonMatchIt++)
	{


		FeatureFound = false;
		FeatObsLocalFrameIt = FeaturesObsLocalFrame.begin();

		while (!FeatureFound){

			if ( (*FeatObsLocalFrameIt).getId() == (*FeatNonMatchIt).getId() ) {
				(*FeatObsLocalFrameIt).toGlobalCordinates(MeanPose, CovPose);
				// add feature to map
				// add feature to temporary list of new features
				AddFeatureToMap ( &*FeatObsLocalFrameIt );
				FeatureFound = true;
			}
			FeatObsLocalFrameIt++;
		}
	}

	/* Update Match Features */
	// go through the current observation and do matching of features
	for (MatchPairIt = FeatMatched.begin(); MatchPairIt != FeatMatched.end(); MatchPairIt++)
	{
		FeatureFound = false;
		FeatObsLocalFrameIt = FeaturesObsLocalFrame.begin();
		while (!FeatureFound){
			if ( (*FeatObsLocalFrameIt).getId() == (*MatchPairIt).Observed->getId() ) {
				// convert observed feature to local map with corrected position
				(*FeatObsLocalFrameIt).toGlobalCordinates(MeanPose, CovPose);
				// merge both observed feature and feature in map
				(*MatchPairIt).InMapFeat->mergeWithFeature ( &(*FeatObsLocalFrameIt) );
				(*MatchPairIt).InMapFeat->increaseMaturity ();
				(*FeatObsLocalFrameIt).setMatchFlag (true);
				FeatureFound = true;
			}
			FeatObsLocalFrameIt++;
		}
	}

	/* Manage the map, bring the most recent matched features to the start of the list */
	// remove false features
	RemoveFalsePositives ();

	// move observed and matched features to the start of the list
	OptimizeListOfFeatures ();

}
/*****************************************************************/
/*								 */
/*		Pose estimation functions			 */
/*								 */
/*****************************************************************/
void Nav_MotionEstimator::Nav_PosePropagator_Unicycle(double * VIn, double T,
		double * PredictedRobotPose)
{
	double v_x = VIn[0];
	double v_yaw = VIn[1];

	/** pose propagation equations for a car like vehicle - approximation */
	PredictedRobotPose[2] = PredictedRobotPose[2] + v_yaw * T; //in rads
	PredictedRobotPose[0] = PredictedRobotPose[0] + v_x * T * cos(
			PredictedRobotPose[2]);
	PredictedRobotPose[1] = PredictedRobotPose[1] + v_x * T * sin(
			PredictedRobotPose[2]);


	if (PredictedRobotPose[2] >= 2 * M_PI)
		PredictedRobotPose[2] = PredictedRobotPose[2] - 2 * M_PI;
	else
	{
		if (PredictedRobotPose[2] < 0.0)
			PredictedRobotPose[2] = 2 * M_PI + PredictedRobotPose[2];
	}

}
void Nav_MotionEstimator::Nav_PosePropagator_Nomad200(double * VIn, double T,
		double * PredictedRobotPose)
{
	double vm_th;
	double vm_theta;

	if (VIn[1] == 0) // x rate equal to zero
	{

		PredictedRobotPose[0] = PredictedRobotPose[0] + VIn[0] * T * cos(
				PredictedRobotPose[2]);
		PredictedRobotPose[1] = PredictedRobotPose[1] + VIn[0] * T * sin(
				PredictedRobotPose[2]);
		PredictedRobotPose[2] = PredictedRobotPose[2];

	}
	else
	{
		if (VIn[0] == 0)
		{ // yaw rate equal to zero
			PredictedRobotPose[0] = PredictedRobotPose[0];
			PredictedRobotPose[1] = PredictedRobotPose[1];
			PredictedRobotPose[2] = PredictedRobotPose[2] + VIn[1] * T;

		}
		else
		{
			double raio = VIn[0] / VIn[1];
			double angle_sum = VIn[1] * T + PredictedRobotPose[2];
			PredictedRobotPose[0] = PredictedRobotPose[0] + raio * (-sin(
					PredictedRobotPose[2]) + sin(angle_sum));
			PredictedRobotPose[1] = PredictedRobotPose[1] - raio * (-cos(
					PredictedRobotPose[2]) + cos(angle_sum));
			PredictedRobotPose[2] = PredictedRobotPose[2] + VIn[1] * T;
		}
	}

	if (PredictedRobotPose[2] >= 2 * M_PI)
		PredictedRobotPose[2] = PredictedRobotPose[2] - 2 * M_PI;
	else
	{
		if (PredictedRobotPose[2] < 0.0)
			PredictedRobotPose[2] = 2 * M_PI + PredictedRobotPose[2];
	}
}

void Nav_MotionEstimator::Nav_PKPropagator_Unicycle( double * Uk, double T)
{
	double thPose = gsl_matrix_get(MeanPose, 2, 0);

	/** todo: move uncertanty values to configuration file */
	double Sig1VtError = 0.01;
	double Sig2VthError = 0.01;
	double Sig3VtError = 0.01;
	double Sig4VthError = 0.01;

	double M11 = Sig1VtError * fabs(Uk[0]) + Sig2VthError * fabs(Uk[1]);
	double M22 = Sig3VtError * fabs(Uk[0]) + Sig4VthError * fabs(Uk[1]); //on the ekf use systematic error

	gsl_matrix_set(Nk, 0, 0, pow(M11, 2));
	gsl_matrix_set(Nk, 0, 1, 0);
	gsl_matrix_set(Nk, 1, 0, 0);
	gsl_matrix_set(Nk, 1, 1, pow(M22, 2));

	/**
	 position gradient
	 Fx =[[ 1	0	-vt*T*sin(theta) ]
	 [ 0        1 	 vt*T*cos(theta) ]
	 [ 0	0 		1	 ]]
	 */
	gsl_matrix_set_identity(Fx);
	gsl_matrix_set(Fx, 0, 2, -Uk[0] * T * sin(thPose));
	gsl_matrix_set(Fx, 1, 2, Uk[0] * T * cos(thPose));
	gsl_matrix_transpose_memcpy(FxT, Fx); //F transpose

	/**
	 velocities gradient

	 Fu = [  [ T*cos(theta)	0 ]
	 [ T*sin(theta)      0 ]
	 [	0		T ]]
	 */
	gsl_matrix_set(Fu, 0, 0, T * cos(thPose));
	gsl_matrix_set(Fu, 0, 1, 0);
	gsl_matrix_set(Fu, 1, 0, T * sin(thPose));
	gsl_matrix_set(Fu, 1, 1, 0);
	gsl_matrix_set(Fu, 2, 0, 0);
	gsl_matrix_set(Fu, 2, 1, T);
	gsl_matrix_transpose_memcpy(FuT, Fu); //F transpose

	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Fu, Nk, 0.0, Qk_temp);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Qk_temp, FuT, 0.0, Qk);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Fx, CovPose, 0.0, Pk_temp);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Pk_temp, FxT, 0.0, res3x3);
	gsl_matrix_set_zero(CovPose);
	gsl_matrix_add(CovPose, Qk);
	gsl_matrix_add(CovPose, res3x3);
}
void Nav_MotionEstimator::Nav_PKPropagator_Nomad200(double * Uk, double T)
{
	double thPose = gsl_matrix_get(MeanPose, 2, 0);
	double angle_diff;
	double vr;

	/** todo: move uncertanty values to configuration file */
	double Sig3VtError = 0.1;
	double Sig4VthError = 2.5;
	double Sig1VtError = 5.5;
	double Sig2VthError = 0.1;
	double M11 = Sig1VtError * fabs(Uk[0]) + Sig2VthError * fabs(Uk[1]);
	double M22 = Sig3VtError * fabs(Uk[0]) + Sig4VthError * fabs(Uk[1]);

	gsl_matrix_set(Nk, 0, 0, pow(M11, 2));
	gsl_matrix_set(Nk, 0, 1, 0);
	gsl_matrix_set(Nk, 1, 0, 0);
	gsl_matrix_set(Nk, 1, 1, pow(M22, 2));

	gsl_matrix_set_identity(Fx);
	gsl_matrix_set_zero(Fu);

	if (Uk[1] == 0)
	{
		// limit Fx, w -> 0, simplify model, gradient of pose and velocity are calculated after that
		/**
		 Fx = [ 1      0	    -V*T*sin(th)
		 0      1     V*T*cos(th)
		 0      0     1 ]
		 */
		gsl_matrix_set(Fx, 0, 2, -Uk[0] * T * sin(thPose));
		gsl_matrix_set(Fx, 1, 2, Uk[0] * T * cos(thPose));
		/**
		 Fu = [ T*cos(th)   0
		 T*sin(th)   0
		 0   	   0 ]
		 */
		gsl_matrix_set(Fu, 0, 0, T * cos(thPose));
		gsl_matrix_set(Fu, 1, 0, T * sin(thPose));
	}
	else
	{
		if (Uk[0] == 0)
		{
			// limit Fx, v -> 0, simplify model, gradient of pose and velocity are calculated after that
			/**
			 Fx = [ 1      0	    0
			 0      1     0
			 0      0     1 ]
			 */
			/**
			 Fu = [ 0   0
			 0   0
			 0   T ]
			 */
			gsl_matrix_set(Fu, 2, 1, T);
		}
		else
		{

			/**
			 velocities gradient
			 Fu = (1/vth)*[ -sin(th)+sin(th+vth*T)	  vt*T*cos(th+vth*T)+(vt/vth)*(sin(th)-sin(th+vth*T))
			 cos(th)-cos(th+vth*T)         vt*T*sin(th+vth*T)+(vt/vth)*(-cos(th)+cos(th+vth*T))
			 0	   			T			    		]

			 */

			vr = Uk[0] / Uk[1];
			double invUk = 1 / Uk[1];
			angle_diff = thPose + Uk[1] * T;

			gsl_matrix_set(Fu, 0, 0, (invUk) * (-sin(thPose) + sin(angle_diff)));
			gsl_matrix_set(Fu, 0, 1, vr * (T * cos(angle_diff)) + (vr * invUk
					* (-sin(thPose) + sin(angle_diff))));
			gsl_matrix_set(Fu, 1, 0, (invUk) * (-cos(thPose) + cos(angle_diff)));
			gsl_matrix_set(Fu, 1, 1, vr * (T * sin(angle_diff)) + (vr * invUk
					* (-cos(thPose) + cos(angle_diff))));
			gsl_matrix_set(Fu, 2, 0, 0);
			gsl_matrix_set(Fu, 2, 1, T);

			/**
			 position gradient
			 Fx = [ 1	0	(vt/vth)*(-cos(th)+cos(th+vth*T))
			 0	1 	(vt/vth)*(-sin(th)+sin(th+vth*T))
			 0	0 		1	]
			 */
			gsl_matrix_set_identity(Fx);
			gsl_matrix_set(Fx, 0, 2, vr * (-cos(thPose) + cos(angle_diff)));
			gsl_matrix_set(Fx, 1, 2, vr * (-sin(thPose) + sin(angle_diff)));

		}
	}

	gsl_matrix_transpose_memcpy(FxT, Fx); //F transpose
	gsl_matrix_transpose_memcpy(FuT, Fu); //F transpose

	/** P(k+1|k) = Fx*Pk*Fx' + Fg*Mt*Fg'*/
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Fu, Nk, 0.0, Qk_temp);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Qk_temp, FuT, 0.0, Qk);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Fx, CovPose, 0.0, Pk_temp);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, Pk_temp, FxT, 0.0, res3x3);
	gsl_matrix_set_zero(CovPose);
	gsl_matrix_add(CovPose, res3x3);
	gsl_matrix_add(CovPose, Qk);

}

void Nav_MotionEstimator::Nav_PositionToSpeed(double * ptrPredD,
		double * ptrTovelInput, double * ptrTopredScanPoseMatch, double * pose,
		double T)
{
	double PoseDiff = pose[2] - ptrPredD[1];
	double vth = ptrPredD[1] / T;
	double vt = 0;
	ptrTopredScanPoseMatch[0] = pose[0] + ptrPredD[0] * cos(PoseDiff);
	ptrTopredScanPoseMatch[1] = pose[1] + ptrPredD[0] * sin(PoseDiff);
	ptrTopredScanPoseMatch[2] = pose[2] - ptrPredD[1];

	if (vth == 0)
	{
		double c = cos(PoseDiff);
		double a = T * cos(pose[2]);
		double d = sin(PoseDiff);
		double b = T * sin(pose[2]);
		double vt2 = (ptrPredD[0] * d) / b;
		vt = (ptrPredD[0] * c) / a;
	}
	else
	{
		double raio = (ptrPredD[0] / 2) / sin((-ptrPredD[1] / 2));
		vt = -raio * vth;
	}

	ptrTovelInput[0] = vt;
	ptrTovelInput[1] = -vth;
}
void Nav_MotionEstimator::Nav_PositionToSpeed(double * PoseNext,
		double * PosePrev, double T, double * outSpeed)
{
	double xPrev = PosePrev[0];
	double yPrev = PosePrev[1];
	double thPrev = PosePrev[2];
	double xNext = PoseNext[0];
	double yNext = PoseNext[1];
	double thNext = PoseNext[2];
	outSpeed[1] = (thNext - thPrev) / T;
	outSpeed[0] = (xNext - xPrev) / (T * cos(thNext));
}

void Nav_MotionEstimator::printObservedFeatures (string id) {
	int i = 0;
	boost::ptr_list<feature_t>::iterator featItObs;
	cout << "PRINTING: " <<  id  << endl;
	for (featItObs = FeaturesObs.begin(); featItObs
	!= FeaturesObs.end(); featItObs++)
	{
		line_t Observed1;
		if ( typeid((*featItObs)).name() == typeid((Observed1)).name() ) {
			line_t * Observed = static_cast<line_t*>(&*featItObs);
			cout << "i: " << i
					<< " r = " << Observed->getLineR()
					<< " alpha = " << Observed->getLineAlpha()
					<< " Maturity: " << (*featItObs).getMaturity ()
					<< " Index End: " << Observed->getEndPointId()
					<< " Index START: " << Observed->getStartPointId() << endl;

			gsl_matrix * Gg = gsl_matrix_alloc ( 2, 3);
			gsl_matrix * QK = gsl_matrix_alloc(2, 2);
			int FeaturePoseGradIndex = 0;
			(*featItObs).getPoseGradient(Gg,&FeaturePoseGradIndex);
			FeaturePoseGradIndex = 0;
			(*featItObs).getCovariance(QK,&FeaturePoseGradIndex);

			printgslMatrix ( Gg, 2 , 3, "Gg");
			printgslMatrix ( QK, 2 , 2 , "QK");

			gsl_matrix_free(Gg);
			gsl_matrix_free(QK);
		}
		++i;
	}

}
