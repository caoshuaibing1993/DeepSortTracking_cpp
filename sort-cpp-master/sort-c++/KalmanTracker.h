/******************************************
作者：cao
功能：
	实现卡尔曼滤波，多目标跟踪功能
******************************************/

#ifndef _KALMANTRACKER_H_
#define _KALMANTRACKER_H_

#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;


typedef struct
{
	float x;
	float y;
	float width;
	float height;
}TRACK_OBJ_RECT_S;



typedef struct TrackingBox
{
	int iFrame;
	int iD;
	TRACK_OBJ_RECT_S sBox;
}TrackingBox;


// This class represents the internel state of individual tracked objects observed as bounding box.
class KalmanTracker
{
public:
	KalmanTracker(TRACK_OBJ_RECT_S _pstInitRect)
	{
		init_kf(_pstInitRect);
		m_time_since_update = 0;
		m_hits = 0;
		m_hit_streak = 0;
		m_age = 0;
		m_id = kf_count;
		kf_count++;
	}

	~KalmanTracker()
	{
		m_history.clear();
	}

	TRACK_OBJ_RECT_S predict();
	void update(TRACK_OBJ_RECT_S* stateMat);
	
	TRACK_OBJ_RECT_S get_state();
	TRACK_OBJ_RECT_S get_rect_xysr(float cx, float cy, float s, float r);

	static int kf_count;

	int m_time_since_update;
	int m_hits;
	int m_hit_streak;
	int m_age;
	int m_id;

private:
	void init_kf(TRACK_OBJ_RECT_S _pstStateMat);

	cv::KalmanFilter kf;
	cv::Mat measurement;

	std::vector<TRACK_OBJ_RECT_S> m_history;
};




#endif