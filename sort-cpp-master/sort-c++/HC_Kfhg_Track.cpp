#include <iostream>
#include <stdio.h>
#include "HC_Kfhg_Track.h"

using namespace std;

int TrackKfhg::HC_Track_Process( TrackingBox* _pstTrackBox, int _iNum)
{
	HungarianAlgorithm HungAlgo;
	frame_count++;

	cout << "Total Frame is " << frame_count << endl;
	//第一帧
	if (trackers.size() == 0)
	{
		// initialize kalman trackers using first detections.
		for (unsigned int i = 0; i < _iNum; i++)
		{
			KalmanTracker trk = KalmanTracker((_pstTrackBox[i].sBox));
			trackers.push_back(trk);
			cout << "[" << _pstTrackBox[i].iFrame << "]" << "    " << "Id is" << _pstTrackBox[i].iD << endl;
		}
		//第一帧处理完跳出
		goto EXIT;

	}

	//3.1卡尔曼滤波预测框的位置，
	predictedBoxes.clear();
	for (auto it = trackers.begin(); it != trackers.end();)
	{
		TRACK_OBJ_RECT_S pBox = (*it).predict();
		if (pBox.x >= 0 && pBox.y >= 0)
		{
			predictedBoxes.push_back(pBox);
			it++;
		}
		else
		{
			it = trackers.erase(it);
			//cerr << "Box invalid at frame: " << frame_count << endl;
		}
	}

	//3.2 associate detections to tracked object (both represented as bounding boxes)
	// dets : detFrameData[fi] 当前检测的目标和上帧轨迹匹配

	//当前帧轨迹的个数
	trkNum = predictedBoxes.size();
	detNum = _iNum;

	iouMatrix.clear();
	iouMatrix.resize(trkNum, vector<double>(detNum, 0));

	for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
	{
		for (unsigned int j = 0; j < detNum; j++)
		{
			// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
			iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], _pstTrackBox[j].sBox);
		}
	}
	// solve the assignment problem using hungarian algorithm.
	// the resulting assignment is [track(prediction) : detection], with len=preNum
	assignment.clear();
	HungAlgo.Solve(iouMatrix, assignment);
	unmatchedTrajectories.clear();
	unmatchedDetections.clear();
	allItems.clear();
	matchedItems.clear();
	//如果检测的个数大于轨迹预测的个数，说明有没有匹配的检测目标
	if (detNum > trkNum) //	there are unmatched detections
	{
		for (unsigned int n = 0; n < detNum; n++)
			allItems.insert(n);

		for (unsigned int i = 0; i < trkNum; ++i)
			matchedItems.insert(assignment[i]);

		/*set_difference(allItems.begin(), allItems.end(),
		matchedItems.begin(), matchedItems.end(),
		insert_iterator<set<int>>(unmatchedDetections, unmatchedDetections.begin()));*/
	}
	else if (detNum < trkNum) // there are unmatched trajectory/predictions
	{
		for (unsigned int i = 0; i < trkNum; ++i)
			if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
				unmatchedTrajectories.insert(i);
	}



	matchedPairs.clear();
	for (unsigned int i = 0; i < trkNum; ++i)
	{
		if (assignment[i] == -1) // pass over invalid values
			continue;
		if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
		{
			unmatchedTrajectories.insert(i);
			unmatchedDetections.insert(assignment[i]);
		}
		else
			matchedPairs.push_back(cv::Point(i, assignment[i]));
	}

	///////////////////////////////////////
	// 3.3. updating trackers更新轨迹
	// update matched trackers with assigned detections.
	// each prediction is corresponding to a tracker

	int detIdx, trkIdx;
	for (unsigned int i = 0; i < matchedPairs.size(); i++)
	{
		trkIdx = matchedPairs[i].x;
		detIdx = matchedPairs[i].y;
		trackers[trkIdx].update(&(_pstTrackBox[i].sBox));

	}
	//针对没有匹配上的检测分配新的轨迹
	// create and initialise new trackers for unmatched detections
	for (auto umd : unmatchedDetections)
	{
		KalmanTracker tracker = KalmanTracker(_pstTrackBox[umd].sBox);
		trackers.push_back(tracker);
	}
	//final get trackers' output
	//得到最后轨迹更新的输出
	frameTrackingResult.clear();
	for (auto it = trackers.begin(); it != trackers.end();)
	{
	
		if (((*it).m_time_since_update < 1) &&
			((*it).m_hit_streak >= min_hits || frame_count <= min_hits))
		{
			TrackingBox res;
			res.sBox = (*it).get_state();
			res.iD = (*it).m_id + 1;
			res.iFrame = frame_count;
			frameTrackingResult.push_back(res);
			it++;
		}
		else
			it++;

		// remove dead tracklet
		if (it != trackers.end() && (*it).m_time_since_update > max_age)
			it = trackers.erase(it);

	}
	for (auto tb : frameTrackingResult)
		cout<< tb.iFrame << "," << tb.iD << "," << tb.sBox.x << "," << tb.sBox.y << "," << tb.sBox.width << "," << tb.sBox.height << ",1,-1,-1,-1" << endl;

EXIT:
	return 0;
}

#if 0
void* HC_Track_Create()
{
	HC_TRACK_S* pstHandle = (HC_TRACK_S*)malloc(sizeof(HC_TRACK_S));

	return pstHandle;
}


int HC_Track_Init(void* _spTrack)
{
	HC_TRACK_S* pstHandle = (HC_TRACK_S*)_spTrack;

//参数初始化
	pstHandle->trkNum = 0;
	pstHandle->detNum = 0;
	pstHandle->frame_count = 0;
	pstHandle->min_hits = 3;
	pstHandle->max_age = 1;
	pstHandle->iouThreshold = 0.3;


	return 0;
}

int HC_Track_Process(void* _spTrack, TrackingBox* _pstTrackBox)
{
	HC_TRACK_S* pstHandle = (HC_TRACK_S*)_spTrack;

	pstHandle->frame_count++;
	cout << "Total Frame is "<<pstHandle->frame_count << endl;
	//第一帧
	if (pstHandle->trackers.size() == 0)
	{
		// initialize kalman trackers using first detections.
		for (unsigned int i = 0; i < _pstTrackBox->iNumber; i++)
		{
			KalmanTracker trk = KalmanTracker(&(_pstTrackBox->sBox[i]));
			pstHandle->trackers.push_back(trk);
			cout << "[" << _pstTrackBox->iFrame << "]" << "    " << "Id is" << _pstTrackBox->iD << endl;
		}
		//第一帧处理完跳出
		goto EXIT;

	}
	
	//3.1卡尔曼滤波预测框的位置，
	pstHandle->predictedBoxes.clear();
	for (auto it = pstHandle->trackers.begin(); it != pstHandle->trackers.end();)
	{
		TRACK_OBJ_RECT_S pBox = (*it).predict();
		if (pBox.x >= 0 && pBox.y >= 0)
		{
			pstHandle->predictedBoxes.push_back(pBox);
			it++;
		}
		else
		{
			it = pstHandle->trackers.erase(it);
			//cerr << "Box invalid at frame: " << frame_count << endl;
		}
	}

	//3.2 associate detections to tracked object (both represented as bounding boxes)
		// dets : detFrameData[fi] 当前检测的目标和上帧轨迹匹配

	//当前帧轨迹的个数
	pstHandle->trkNum = pstHandle->predictedBoxes.size();




EXIT:
	return 0;
}
#endif