
#include "nav_motionestimator.h"
#include "nav3mainwindow.h"
#include "scene_manager.h"

#include <QtGui/QApplication>

#include <iostream>


using namespace std;

int main(int argc, char **argv) {

	int serverId = 0;
	cout << "<TRACE><LOG><main> Starting Motion Estimator " << endl;


        // Start Main Window
        QApplication app(argc, argv);

        Nav3MainWindow Window;
        Window.show();
        cout << "<TRACE><LOG><main> MAIN WINDOWS LAUNCHED " << endl;
        app.exec();
	// setup the control thread
	Nav_MotionEstimator * PoseEstimatorUnicycle;
	try {
		PoseEstimatorUnicycle = new Nav_MotionEstimator(UNICYCLE_MODEL, "PLAYER");
	} catch (std::string e) {
		std::cerr << "<TRACE><Error><main> Unable to Start Simulation \n" << e
				<< "\n";
		return -1;
	}
	// wait for thread end
	PoseEstimatorUnicycle->Nav_join();

	return 0;

}
