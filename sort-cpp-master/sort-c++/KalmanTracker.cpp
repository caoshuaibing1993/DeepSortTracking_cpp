/*****************************************
作者：cao
功能：	
	卡尔曼滤波源文件
******************************************/

#include "KalmanTracker.h"

int KalmanTracker::kf_count = 0;


// initialize Kalman filter
void KalmanTracker::init_kf(TRACK_OBJ_RECT_S _pstStateMat)
{
	int stateNum = 7;
	int measureNum = 4;
	kf = KalmanFilter(stateNum, measureNum, 0);

	measurement = Mat::zeros(measureNum, 1, CV_32F);

	kf.transitionMatrix = *(Mat_<float>(stateNum, stateNum) <<
		1, 0, 0, 0, 1, 0, 0,
		0, 1, 0, 0, 0, 1, 0,
		0, 0, 1, 0, 0, 0, 1,
		0, 0, 0, 1, 0, 0, 0,
		0, 0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 0, 1, 0,
		0, 0, 0, 0, 0, 0, 1);

	setIdentity(kf.measurementMatrix);
	setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kf.errorCovPost, Scalar::all(1));
	
	// initialize state vector with bounding box in [cx,cy,s,r] style
	kf.statePost.at<float>(0, 0) = _pstStateMat.x + _pstStateMat.width / 2;
	kf.statePost.at<float>(1, 0) = _pstStateMat.y + _pstStateMat.height / 2;
	kf.statePost.at<float>(2, 0) = _pstStateMat.height * _pstStateMat.width;
	kf.statePost.at<float>(3, 0) = _pstStateMat.width / _pstStateMat.height;
}


// Predict the estimated bounding box.
TRACK_OBJ_RECT_S KalmanTracker::predict()
{
	// predict
	Mat p = kf.predict();
	TRACK_OBJ_RECT_S sTrackRect = { 0 };
	m_age += 1;

	if (m_time_since_update > 0)
		m_hit_streak = 0;
	m_time_since_update += 1;

	TRACK_OBJ_RECT_S predictBox = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

	m_history.push_back(predictBox);
	sTrackRect.x = predictBox.x;
	sTrackRect.y = predictBox.y;
	sTrackRect.height = predictBox.height;
	sTrackRect.width = predictBox.width;
	return sTrackRect;
}


// Update the state vector with observed bounding box.
void KalmanTracker::update(TRACK_OBJ_RECT_S* _StateMat)
{
	m_time_since_update = 0;
	m_history.clear();
	m_hits += 1;
	m_hit_streak += 1;

	// measurement
	measurement.at<float>(0, 0) = _StateMat->x + _StateMat->width / 2;
	measurement.at<float>(1, 0) = _StateMat->y + _StateMat->height / 2;
	measurement.at<float>(2, 0) = (_StateMat->width * _StateMat->height);
	measurement.at<float>(3, 0) = _StateMat->width / _StateMat->height;

	// update
	kf.correct(measurement);
}


// Return the current state vector
TRACK_OBJ_RECT_S KalmanTracker::get_state()
{
	Mat s = kf.statePost;
	return get_rect_xysr(s.at<float>(0, 0), s.at<float>(1, 0), s.at<float>(2, 0), s.at<float>(3, 0));
}


// Convert bounding box from [cx,cy,s,r] to [x,y,w,h] style.
TRACK_OBJ_RECT_S KalmanTracker::get_rect_xysr(float cx, float cy, float s, float r)
{
	TRACK_OBJ_RECT_S st = {0};
	float w = sqrt(s * r);
	float h = s / w;
	float x = (cx - w / 2);
	float y = (cy - h / 2);

	if (x < 0 && cx > 0)
		x = 0;
	if (y < 0 && cy > 0)
		y = 0;
	st.x = x;
	st.y = y;
	st.width = w;
	st.height = h;
	return st;
}




// --------------------------------------------------------------------
// Kalman Filter Demonstrating, a 2-d ball demo
// --------------------------------------------------------------------

const int winHeight = 600;
const int winWidth = 800;

Point mousePosition = Point(winWidth >> 1, winHeight >> 1);

// mouse event callback
void mouseEvent(int event, int x, int y, int flags, void *param)
{
	if (event == CV_EVENT_MOUSEMOVE) {
		mousePosition = Point(x, y);
	}
}

void TestKF();

void main0()
{
	TestKF();
}


void TestKF()
{
	int stateNum = 4;
	int measureNum = 2;
	KalmanFilter kf = KalmanFilter(stateNum, measureNum, 0);

	// initialization
	Mat processNoise(stateNum, 1, CV_32F);
	Mat measurement = Mat::zeros(measureNum, 1, CV_32F);

	kf.transitionMatrix = *(Mat_<float>(stateNum, stateNum) <<
		1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1);

	setIdentity(kf.measurementMatrix);
	setIdentity(kf.processNoiseCov, Scalar::all(1e-2));
	setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kf.errorCovPost, Scalar::all(1));

	randn(kf.statePost, Scalar::all(0), Scalar::all(winHeight));

	namedWindow("Kalman");
	setMouseCallback("Kalman", mouseEvent);
	Mat img(winHeight, winWidth, CV_8UC3);

	while (1)
	{
		// predict
		Mat prediction = kf.predict();
		Point predictPt = Point(prediction.at<float>(0, 0), prediction.at<float>(1, 0));

		// generate measurement
		Point statePt = mousePosition;
		measurement.at<float>(0, 0) = statePt.x;
		measurement.at<float>(1, 0) = statePt.y;

		// update
		kf.correct(measurement);

		// visualization
		img.setTo(Scalar(255, 255, 255));
		circle(img, predictPt, 8, CV_RGB(0, 255, 0), -1); // predicted point as green
		circle(img, statePt, 8, CV_RGB(255, 0, 0), -1); // current position as red

		imshow("Kalman", img);
		char code = (char)waitKey(100);
		if (code == 27 || code == 'q' || code == 'Q')
			break;
	}
	destroyWindow("Kalman");
}

