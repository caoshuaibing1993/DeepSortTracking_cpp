#ifndef _HC_KFHG_TRACK_H_
#define _HC_KFHG_TRACK_H_
#include <set>
#include "KalmanTracker.h"
#include "Hungarian.h"
//定义结构体

#define UMATDET 32
#define MAXUMATDET 64


class TrackKfhg
{
public:
	TrackKfhg() {
		//参数初始化
		trkNum = 0;
		detNum = 0;
		frame_count = 0;
		min_hits = 3;
		max_age = 1;
		iouThreshold = 0.3;
	
	}
	~TrackKfhg() {}

	int HC_Track_Process(TrackingBox* _pstTrackBox, int _iNum);
	// Computes IOU between two bounding boxes
	double GetIOU(TRACK_OBJ_RECT_S bb_test, TRACK_OBJ_RECT_S bb_gt)
	{
		
		float fMinRight = min((bb_test.x + bb_test.width), (bb_gt.x + bb_gt.width));
		float fMinBottom = min((bb_test.y + bb_test.height), (bb_gt.y + bb_gt.height));
		float fMaxLeft = max(bb_test.x, bb_gt.x);
		float fMaxTop = max(bb_test.y, bb_gt.y);
		
		float in = (fMinRight - fMaxLeft)*(fMinBottom - fMaxTop);
		float un = bb_test.height*bb_test.width + bb_gt.height*bb_gt.width - in;
		if (un < DBL_EPSILON)
			return 0;

		return (double)(in / un);
	}
	
private:
	int frame_count;
	int max_age;
	int min_hits;
	double iouThreshold;
	vector<KalmanTracker> trackers;
								 // variables used in the for-loop
	vector<TRACK_OBJ_RECT_S> predictedBoxes;
	vector<TrackingBox> frameTrackingResult;
	vector<vector<double>> iouMatrix;
	vector<int> assignment;
	TrackingBox sTrackBox[MAXUMATDET];
	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	set<int> allItems;
	set<int> matchedItems;
	vector<cv::Point> matchedPairs;
	
	unsigned int trkNum;
	unsigned int detNum;

};


#endif
