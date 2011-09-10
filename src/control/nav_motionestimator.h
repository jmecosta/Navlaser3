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

#ifndef NAV_MOTIONESTIMATOR_H
#define NAV_MOTIONESTIMATOR_H

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>


#ifdef USEGAZEBO
#include <gazebo/gazebo.h>
#elif defined PLAYERPLUGGING
#else
#include <libplayerc/playerc.h>
#endif

#include "../defs.h"
#include "../sensors/lmsscan_t.h"
#include "../map/features_map_t.h"
#include "../map/grid_map_t.h"
#include "../math/math_lib.h"
#include "../config/config_movement.h"

using namespace std;
using namespace boost;
#ifdef USEGAZEBO
using namespace libgazebo;
#endif

class Nav_MotionEstimator: public grid_map_t,
		public features_map_t,
		public config_movement
{
public:
	Nav_MotionEstimator(int modeltoUse, string dev);
	~Nav_MotionEstimator( );

	// setup of the thread
	void Nav_SetControlObjs();

	// thread run method - start motion estimator thread
	void Nav_RunEFKMethod(int N);
	void Nav_start(int N);

	void Nav_join();

	// for boost thread
	//void start_thread();
	//void thread ();
	//void ~thread ();
private:

	// object for pose estimation
	gsl_matrix * MeanPose;
	gsl_matrix * CovPose;

	// matrixes definition
	gsl_matrix * Nk; // 2x2
	gsl_matrix * Fx; // 3x3
	gsl_matrix * FxT; // 3x3
	gsl_matrix * Fu; // 3x2
	gsl_matrix * FuT; // 2x3
	gsl_matrix * Qk; // 3x3
	gsl_matrix * Qk_temp; // 3x2
	gsl_matrix * Pk_temp; // 3x3
	gsl_matrix * res3x3; // 3x3

	// objects for observation
	laser_t * ScanObj;

	// player/stage/gazebo objects
#ifdef USEGAZEBO
	PositionIface *posIface;
	Client *client;
#elif defined PLAYERPLUGGING
#else
	playerc_client_t * client;
	playerc_position2d_t *position2d;
#endif

	// boost thread need
	boost::thread  m_Thread;

	// features matched during
	boost::ptr_list<MatchedFeatures_C> FeatMatched;
	// non matched features
	boost::ptr_list<feature_t> FeaturesNonMatched;
	// observed features
	boost::ptr_list<feature_t> FeaturesObs;
	void printObservedFeatures (string id);
	// backup observed features
	boost::ptr_list<feature_t> FeaturesObsLocalFrame;

	/*****************************************************************/
	/*								 */
	/*		Robot control methods			 */
	/*								 */
	/*****************************************************************/
	void Nav_CommandRobot(double * SpeedInput, double T);

	/*****************************************************************/
	/*								 */
	/*		Pose estimation functions			 */
	/*								 */
	/*****************************************************************/

	/**
	 * This is the motion propagator, receives a speed input in unicycle model and the current pose,
	 *  using this information estimates the new pose
	 * Additionally to this, if a real position is given it calculates the real position using the same model
	 * @param VIn Current velocity input command vector, xspeed (m/s) and yaw speed (rad/s)
	 * @param T Previous Corrected Pose Using the various Slam algorithms
	 * @param PredictedRobotPose Current Pose, updated pose using speed input
	 */
	void Nav_PosePropagator_Unicycle(double * VIn, double T,
			double * PredictedRobotPose);

	/**
	 * This is the covariance matrix propagator, for Unicycle models, Fx is derivate of the unicycle approximation
	 *
	 *  	R(k + 1|k) = Fx(k).R(k|k).FxT(k) + Fu(k).N(k).FuT(k)
	 *
	 * @param Mean pose
	 * @param Cov covariance matrix
	 * @param Uk speed input
	 * @param T period
	 */
	void Nav_PKPropagator_Unicycle(double * Uk, double T);

	/**
	 * This is the motion propagator, receives a speed input in nomad 200 model and the current pose,
	 *  using this information estimates the new pose
	 * Additionally to this, if a real position is given it calculates the real position using the same model
	 * @param VIn Current velocity input command vector, xspeed (m/s) and yaw speed (rad/s)
	 * @param T Previous Corrected Pose Using the various Slam algorithms
	 * @param PredictedRobotPose IN/OUT (k) Pose, (k+1) updated pose using speed input
	 */
	void Nav_PosePropagator_Nomad200(double * VIn, double T,
			double * PredictedRobotPose);

	/**
	 * This is the covariance matrix propagator, for Nomad 200 models, Fx is derivate of the Nomad 200 transfer function
	 *
	 *  	R(k + 1|k) = Fx(k).R(k|k).FxT(k) + Fu(k).N(k).FuT(k)
	 *
	 * @param Mean pose
	 * @param Cov covariance matrix
	 * @param Uk speed input
	 * @param T period
	 */
	void Nav_PKPropagator_Nomad200(double * Uk, double T);

	/**
	 * Using inverse kinematic information estimate the velocities that were used in the robot movement
	 * @param ptrPredD Previous Pose
	 * @param ptrTovelInput Updated velocities
	 * @param ptrTopredScanPoseMatch Pose Predicted using scan allignent
	 * @param pose Last Pose
	 * @param T Period of time
	 */
	void Nav_PositionToSpeed(double * ptrPredD, double * ptrTovelInput,
			double * ptrTopredScanPoseMatch, double * pose, double T);

	/**
	 * Using inverse kinematic information estimate the velocities that were used in the robot movement
	 * @param PoseNext Previous Pose
	 * @param PosePrev Updated velocities
	 * @param T Pose Predicted using scan allignent
	 * @param outSpeed Last Pose
	 */
	void Nav_PositionToSpeed(double * PoseNext, double * PosePrev, double T,
			double * outSpeed);

	/*****************************************************************/
	/*								 */
	/*		Extended Kalman Filter Functions		 */
	/*								 */
	/*****************************************************************/
	void Nav_EKFStage1MovPredictor(	double * SpeedInput, double T, double * CovElipseOut);
	void Nav_EKFStage1Observation( );
	void Nav_EKFStage2Matching( );
	void Nav_EKFStage3Correction( );
	void Nav_EKFStage4MapUpdate( );

};

#endif // NAV_MOTIONESTIMATOR_H
