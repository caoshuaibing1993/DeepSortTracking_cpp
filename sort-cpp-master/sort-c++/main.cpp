
//372 lines(308 sloc) 10.3 KB
///////////////////////////////////////////////////////////////////////////////
//  SORT: A Simple, Online and Realtime Tracker
//  
//  This is a C++ reimplementation of the open source tracker in
//  https://github.com/abewley/sort
//  Based on the work of Alex Bewley, alex@dynamicdetection.com, 2016

///////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <fstream>
#include <iomanip> // to format image names using setw() and setfill()
#include <io.h>    // to check file existence using POSIX function access(). On Linux include <unistd.h>.
#include <set>


#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "HC_Kfhg_Track.h"
using namespace std;
using namespace cv;



// global variables for counting
#define CNUM 20
int total_frames = 0;
double total_time = 0.0;

int main()
{
	TrackKfhg track;
	TrackingBox ss[10] = {0};
	
	for (int j = 0; j < 100; j++)
	{
		for (int i = 0; i < 10; i++)
		{
			ss[i].sBox.x = i * 11 + 1;
			ss[i].sBox.y = i * 12 + 15;
			ss[i].sBox.width = i * 15 + 9;
			ss[i].sBox.height = i * 14 + 20;
			ss[i].iD = i + 1;
			ss[i].iFrame++;
		}
		
		track.HC_Track_Process(ss, 10);
	}
	
	

	return 0;
}


