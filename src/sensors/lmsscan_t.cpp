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

#include "lmsscan_t.h"
#include <stdio.h>
#include <stdlib.h>


#ifdef USEGAZEBO
lmsScan_t::lmsScan_t ( Client * Connectedclient, std::string dev ) {

	/// Open the global control stuff
	try
	{
		std::cout << "<TRACE><LOG><lmsScan_t> Creating LMS object " << dev << std::endl;
		lasIface = new LaserIface ();
		lasIface->Open( Connectedclient, dev );

		// setup the laser
		NmbSamples = lasIface->data->range_count;
		resolution = (lasIface->data->max_angle - lasIface->data->min_angle) / NmbSamples;
		max_range = lasIface->data->max_range;

		SigmaPhi.resize (NmbSamples);
		SigmaR.resize(NmbSamples);
		r.resize(NmbSamples);
		alpha.resize(NmbSamples);
		x.resize(NmbSamples);
		y.resize(NmbSamples);

		// fill angles values based in the resultion
		double AddRes = lasIface->data->min_angle;
		for (int i = 0; i < NmbSamples; i++)
		{
			alpha[i] = AddRes;
			AddRes += resolution;
		}

		std::cout << "<TRACE><LOG><lmsScan_t> Samples in Object: " << NmbSamples << std::endl;



	}
	catch (std::string e)
	{
		std::cerr << "Gazebo error: Unable to connect to Specified Laser \n" << e <<  " dev: " << dev << "\n";
		delete lasIface;
		r.clear();
		alpha.clear();
		x.clear();
		y.clear();
		SigmaPhi.clear();
		SigmaR.clear();
		throw e;
	}
}
#elif defined PLAYERPLUGGING
#else
lmsScan_t::lmsScan_t ( playerc_client_t * Connectedclient, int index ) {

	/// Open the global control stuff
	try
	{
		std::cout << "<TRACE><LOG><lmsScan_t> Creating LMS object " << index << std::endl;

		lasIface = playerc_ranger_create (Connectedclient, index);
		if (playerc_ranger_subscribe(lasIface, PLAYER_OPEN_MODE)){
			fprintf(stderr, "error: %s\n", playerc_error_str());
			throw "INVALID";
		}
		// Request the device config for min angle and resolution
		if (playerc_ranger_get_config(lasIface, NULL, NULL, NULL, NULL, NULL, NULL, NULL) != 0)
		{
			fprintf(stderr, "error: %s\n", playerc_error_str());
			resolution = 0.0f;
			start_angle = 0.0f;
		}
		else
		{

			max_range = lasIface->max_range;
			start_angle = lasIface->min_angle;
			end_angle =  lasIface->max_angle;
			playerc_client_read(Connectedclient);
			playerc_client_read(Connectedclient);

			NmbSamples = lasIface->points_count;
			resolution = (lasIface->max_angle - lasIface->min_angle) / NmbSamples;

		}
		std::cout << "<TRACE><LOG><lmsScan_t> DEV INFO " << Connectedclient->devinfos << std::endl;

		// print laser information
		std::cout << "Scan Count: " << lasIface->points_count << std::endl;
		std::cout << "Scan Res: " << lasIface->range_res << std::endl;
		std::cout << "Max Range: " << lasIface->max_range << std::endl;
		std::cout << "MIN ANGLE: " << start_angle << std::endl;
		std::cout << "MAX ANGLE: " << end_angle << std::endl;

		SigmaPhi.resize (NmbSamples);
		SigmaR.resize(NmbSamples);
		r.resize(NmbSamples);
		alpha.resize(NmbSamples);
		x.resize(NmbSamples);
		y.resize(NmbSamples);

		// aux gui stuff, using rtk
		int argc = 0;
		char **argv;
		rtk_init(&argc, &argv);
		// Create gui
		app = rtk_app_create();
		// Create a window for most of the sensor data
		wnd = new mainwnd();
		wnd = wnd->mainwnd_create(app, "localhost", 6560);
		scan_fig = NULL;
		// Start the gui; dont run in a separate thread and dont let it do
		// its own updates.
		rtk_app_main_init(app);
	}
	catch (std::string e)
	{
		std::cerr << "<TRACE><ERROR><lmsScan_t> Unable to connect to Specified Laser "
				<< index <<  " dev: "
				<< Connectedclient->devinfos
				<< std::endl;
		delete lasIface;
		r.clear();
		alpha.clear();
		x.clear();
		y.clear();
		SigmaPhi.clear();
		SigmaR.clear();
		throw e;
	}

	client = Connectedclient;
}
#endif

lmsScan_t::~lmsScan_t()
{
	cout << "LASER DESTROY" << endl;
#ifdef USEGAZEBO
	lasIface->Close();
	delete lasIface;
#elif defined PLAYERPLUGGING
#else
	cout << "STOP LMS SCAN - UNSUBSCRIBE LASET" << endl;
	playerc_ranger_unsubscribe(lasIface);
	playerc_ranger_destroy (lasIface);
#endif
}

bool lmsScan_t::lmsScan_UpdateScan()
{
#ifdef USEGAZEBO
	try {
		for (int i = 0; i < NmbSamples; i++)
		{
			this->SigmaR[i] = 0.005;
			this->SigmaPhi[i] = 0.0;
			if ( lasIface->data->ranges[i] > lasIface->data->max_range ) {
				r[i] = 1000000;
				x[i] = 1000000;
				y[i] = 1000000;
			} else  {
				r[i] = lasIface->data->ranges[i];
				x[i] = r[i] * cos ( alpha[i] );
				y[i] = r[i] * sin ( alpha[i] );
			}
		}
	} catch ( std::string e ) {
		cerr << "Gazebo error: Unable to connect to Specified Laser \n" << e << "\n";
		throw e;
	}
#elif defined PLAYERPLUGGING
#else

	cout << "LASER READ: " << NmbSamples << endl;

	try {
		playerc_client_read(client);
		for (int i = 0; i < NmbSamples; i++)
		{
			this->SigmaR[i] = 0.005;
			this->SigmaPhi[i] = 0.0;

			cout << "Sample: " << i << endl;

			if ( lasIface->ranges[i] > lasIface->max_range ) {
				alpha[i] = lasIface->bearings[i];
				r[i] = 1000000;
				x[i] = 1000000;
				y[i] = 1000000;
			} else  {
				r[i] = lasIface->ranges[i];
				alpha[i] = start_angle + i*resolution;
				x[i] = r[i] * cos ( alpha[i] );
				y[i] = r[i] * sin ( alpha[i] );
			}
		}
	} catch ( std::string e ) {
		cerr << "Stage error: Unable to connect to Specified Laser \n" << e << "\n";
		throw e;
	}
#endif

}

bool lmsScan_t::UpdateScan()
{
	  double points[NmbSamples][2];
	  double point1[2], point2[2];
	  double b, range;

#ifdef USEGAZEBO
	try {
		for (int i = 0; i < NmbSamples; i++)
		{
			this->SigmaR[i] = 0.05;
			this->SigmaPhi[i] = 0.0;
			if ( lasIface->data->ranges[i] > lasIface->data->max_range ) {
				r[i] = 1000000;
				x[i] = 1000000;
				y[i] = 1000000;
			} else  {
				r[i] = lasIface->data->ranges[i];
				x[i] = r[i] * cos ( alpha[i] );
				y[i] = r[i] * sin ( alpha[i] );
			}



		}

	} catch ( std::string e ) {
		cerr << "Gazebo error: Unable to connect to Specified Lase \n" << e << "\n";
		throw e;
	}
#elif defined PLAYERPLUGGING
#else

	int ii = 0;
	// delete any figures in screen
	if ( scan_fig != NULL)
	{
		for (ii = 0; ii < NmbSamples; ii++)
			rtk_fig_destroy(scan_fig[ii]);
		free(scan_fig);
		scan_fig = NULL;
	}

	bool Print = true;
	if ((scan_fig = (rtk_fig_t**)malloc(NmbSamples * sizeof(rtk_fig_t*))) == NULL )
	{
		cerr << "Failed to allocate memory for %d figures to display ranger: " << NmbSamples << endl;
		Print = false;
	}

	try {
		playerc_client_read(client);

		//points = (double**)calloc(NmbSamples + 1, sizeof(double)*2);

		for (int i = 0; i < NmbSamples; i++)
		{
			this->SigmaR[i] = 0.05;
			this->SigmaPhi[i] = 0.0;
			if ( lasIface->ranges[i] > lasIface->max_range ) {
				alpha[i] = lasIface->bearings[i];
				r[i] = 1000000;
				x[i] = 1000000;
				y[i] = 1000000;
			} else  {
				alpha[i] = start_angle + i*resolution;
				r[i] = lasIface->ranges[i];
				x[i] = r[i] * cos ( alpha[i] );
				y[i] = r[i] * sin ( alpha[i] );
			}
	        points[i][0] = x[i];
	        points[i][1] = y[i];
			if ( Print == true ) {
				scan_fig[i] = rtk_fig_create(wnd->canvas, wnd->robot_fig, 1);
				rtk_fig_origin(scan_fig[i],0,0,0);
			}
		}
	} catch ( std::string e ) {
		cerr << "STAGE error: Unable to connect to Specified Lase \n" << e << "\n";
		throw e;
	}
#endif

    // Draw each laser in turn
    rtk_fig_show(scan_fig[0], 1);
    rtk_fig_clear(scan_fig[0]);

    // Draw empty space
    rtk_fig_color_rgb32(scan_fig[0], COLOR_LASER_EMP);

    // draw color scan area
    rtk_fig_polygon(scan_fig[0], 0, 0, 0,NmbSamples, points, 1);

    // Draw occupied space
    //Draw an intensity scan
    rtk_fig_color_rgb32(scan_fig[0], COLOR_LASER_OCC);
    for (ii = 1; ii < NmbSamples; ii++)
    {
      point1[0] = points[ii][0];
      point1[1] = points[ii][1];
      point2[0] = points[ii+1][0];
      point2[1] = points[ii+1][1];
      rtk_fig_line(scan_fig[0], point1[0], point1[1], point2[0], point2[1]);
      rtk_fig_rectangle(scan_fig[0], point1[0], point1[1], 0, 0.05, 0.05, 1);
    }
    // Draw the sensor itself
    rtk_fig_color_rgb32(scan_fig[0], COLOR_LASER);
    rtk_fig_rectangle(scan_fig[0], 0, 0, 0, 0.5, 0.5, 0);

	// update gui
	rtk_app_main_loop(app);
	wnd->mainwnd_update(wnd);

}
