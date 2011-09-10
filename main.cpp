#include <iostream>
#include <time.h>

#include "src/sensors/lmsscan_t.h"
#include "src/control/nav_motionestimator.h"
#include "src/features/features_extractor_t.h"
#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/thread.hpp>
using namespace std;

class counter
{
public:
    counter() : count(0) { }

    int add(int val) {
        boost::recursive_mutex::scoped_lock scoped_lock(mutex);
        count += val;
        return count;
    }
    int increment() {
        boost::recursive_mutex::scoped_lock scoped_lock(mutex);
        return add(1);
    }

private:
    boost::recursive_mutex mutex;
    int count;
};
counter c;
void change_count()
{
    std::cout << "count == " << c.increment() << std::endl;
}
int main(int argc, char **argv) {


	    const int num_threads=4;
	int serverId = 0;
	cout << "<TRACE><LOG><main> Starting Motion Estimator " << endl;

	boost::thread_group threads;
    for (int i=0; i < num_threads; ++i)
        threads.create_thread(&change_count);

    threads.join_all();

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
