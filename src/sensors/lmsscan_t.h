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

#ifndef LMSSCAN_T_H
#define LMSSCAN_T_H

#ifdef USEGAZEBO
#include <gazebo/gazebo.h>
#elif defined PLAYERPLUGGING
#else
#include <libplayerc/playerc.h>
#endif

#include <iostream>
#include <math.h>
#include "laser_t.h"
#include "../rtk2/mainwnd.h"

#ifdef USEGAZEBO
using namespace libgazebo;
#endif

using namespace std;

class lmsScan_t : public laser_t
{
  private:

#ifdef USEGAZEBO
	LaserIface * lasIface; // gazebo laser iface
#elif defined PLAYERPLUGGING
#else
	playerc_ranger_t * lasIface; // laser interface
	playerc_client_t * client;
    rtk_app_t *app;
    mainwnd *wnd;
    rtk_fig_t **scan_fig;
#endif
        
    double SistError; // error analysis - sistematic error
    int NmbSamples;
    double resolution;
    double max_range;
	double datatime;
	char *drivername;
	double start_angle;
	double end_angle;

    vector<double> SigmaPhi; // point uncertanty angle
    vector<double> SigmaR; // point uncertanty distance
    vector<double> x; // coordinate x of the sample, post processed
    vector<double> y; // coordinate y of the sample, post processed
    vector<double> r; // distance to target
    vector<double> alpha;  // angle that point received makes with x axis of the laser

  public:
#ifdef USEGAZEBO
    lmsScan_t ( Client * Connectedclient, std::string dev );
#elif defined PLAYERPLUGGING
#else
    lmsScan_t ( playerc_client_t * Connectedclient, int index );
#endif
    virtual ~lmsScan_t ();

    bool lmsScan_UpdateScan ();
    bool UpdateScan ();
            
    int  getNmbSamples () { return NmbSamples; };
    double getLaserMaxRange () { return max_range; };
    double getLaserResolution () { return resolution; }
    double getSisError () { return SistError;}
    double getBearings (int i) {return alpha[i];};
    double getRange (int i) {return r[i];};
    double getX (int i)  {return x[i];};
    double getY (int i)  {return y[i];};
    double getSigmaR (int i)  {return SigmaR[i];};
    double getSigmaPhi (int i)  {return SigmaPhi[i];};

};

#endif // LMSSCAN_T_H
