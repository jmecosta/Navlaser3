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

#ifndef FEATURES_EXTRACTOR_T_H
#define FEATURES_EXTRACTOR_T_H

//#include <gazebo/gazebo.h>
#include <boost/ptr_container/ptr_list.hpp>
#include <iostream>
#include <list>
#include <vector>
#include <math.h>

#include "../sensors/laser_t.h"
#include "../math/math_lib.h"
#include "line_t.h"
#include "point_t.h"
#include "../config/config_extractor.h"

using namespace std;

class features_extractor_t : public config_extractor
{
private:

    list<line_t> features_LineAdapBreakIterClosestPt ();
    list<line_t> features_LineAdapBreakIterClosestPt ( double lambda, laser_t * ScanObj );

    // complete recursive methods
    bool LineEstimator_RecursiveCornerDetectorSplitEstimateAndMergeLines (  list<line_t> * ExtractedLines,
									    laser_t * ScanObj, 
									    int start_point,
									    int end_point );  // IEPF
    
    // features fit methods
    bool LineEstimator_FitLine ( int start, int end, laser_t * ScanObj, line_t * new_line ); // main method
    bool LineEstimator_EndPointLineFit ( int start, int end, laser_t * ScanObj, line_t * new_line );
    bool LineEstimator_LeastSquareLineFit ( int start, int end, laser_t * ScanObj, line_t * new_line );
    bool LineEstimator_MinimizePointToLineDifferential (int start, int end, laser_t * ScanObj, line_t * new_line );
    bool LineEstimator_VandorpeLineRegression ( int start, int end, laser_t * ScanObj, line_t * new_line );
    
    // line auxiliary methods
    bool LinesEstimator_MergeContiguos( list<line_t> * listofLines, laser_t * ScanObj );
    void LinesEstimator_SetEndPointsAttr ( list<line_t> * listofLines, laser_t * ScanObj );
    
    // rupture points methods
    bool IsABreakPoint ( laser_t * ScanObj, int i ); // main method   
    bool IsABreakPoint_SimpleMethod ( laser_t * ScanObj, int i );    
    bool IsABreakPoint_AdptiveRuptureMethod ( laser_t * ScanObj, int index );
    bool IsABreakPoint_IsThereaBreakPoint  ( laser_t * ScanObj, int start, int end ); // method to check if there are rupture point in sample of poits
    
    // corner detection methods
    bool Corners_LineValidationAndCornerDetection ( laser_t * ScanObj, int start_point, int end_point, list<int> * Corners ); // main method
    bool Corners_2derivativeTestEstimation ( laser_t * ScanObj, int start_point, int end_point, list<int> * Corners );
    bool Corners_RamsacCornerDetection ( laser_t * ScanObj, int start_point, int end_point, list<int> * Corners );
           
public:
    features_extractor_t ( );
    
    // public method to extrat features    
    void features_LineExtractor ( laser_t * XscanVector, list<line_t> * lines );
    list<point_t> features_PointExtractor ( int ExtractMethod, double * XscanVector, double * YscanVector, int sizeOfvector );
            
};

#endif // FEATURES_EXTRACTOR_T_H
