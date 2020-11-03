

#include "FindFarestPoints.hpp"


using namespace cv;
using namespace std;

std::vector<cv::Point> FindFar(std::vector<cv::Point> &pts, int cx, int cy, double theta, cv::Mat &img)
{
	//vector<cv::Point> vec;
	std::vector<cv::Point> vecxy;
	std::vector<cv::Point> vecxy1;
	std::vector<std::pair<int, int> > vec;
	std::vector<std::pair<int, int> > vec1;
	//Construct a buffer used by the pca analysis
   	int sz = static_cast<int>(pts.size());
    	Mat data_pts = Mat(sz, 4, CV_64F);
    	Mat ang_mat = Mat(sz, 1, CV_64F);
    	Mat ang_mat1 = Mat(sz, 1, CV_64F);	

    	for (int i = 0; i < pts.size(); i++)
    	{
        	int x = pts[i].x;
        	int y = pts[i].y;
		double dis = pow((x-cx),2)+pow((y-cy),2);
		double diff_angle=atan2(cy-y, cx-x) -theta;
		double diff_angle1=atan2(cy-y, cx-x) -theta+3.14;

		data_pts.at<double>(i, 0) = pts[i].x;
        	data_pts.at<double>(i, 1) = pts[i].y;
        	data_pts.at<double>(i, 2) = dis;
        	data_pts.at<double>(i, 3) = diff_angle;

        	ang_mat.at<double>(i, 0) = diff_angle;
        	ang_mat1.at<double>(i, 0) = diff_angle1;

		if (abs(diff_angle)<0.174)
		{
			vecxy.push_back({x,y});
			vec.push_back(make_pair(dis,diff_angle));
		}

		if (abs(diff_angle1)<0.174)
		{
			vecxy1.push_back({x,y});
			vec1.push_back(make_pair(dis,diff_angle1));
		}

    	}
	
	// find the min and max angle
	
	std::vector<cv::Point> vec_far;
	
	cv::Point minLoc;
	cv::Point maxLoc;
	double min, max;
	//cv::minMaxLoc(ang_mat, &min, &max, &minLoc, &maxLoc);
	//cout<<"angle: min="<<min<<",minLoc="<<minLoc.y<<",max="<<max<<",maxLoc="<<maxLoc.y<<endl;

	//----orientation1------
    	Mat dist_mat = Mat(vecxy.size(), 1, CV_64F);
	for (int i=0; i< vecxy.size(); i++)
	{
		dist_mat.at<double>(i, 0) = abs(vec[i].first);	
	}
	// find the min and max dispc
	cv::minMaxLoc(dist_mat, &min, &max, &minLoc, &maxLoc);
	cout<<"dist: min="<<min<<",minLoc="<<minLoc.y<<",max="<<max<<",maxLoc="<<maxLoc.y<<endl;
	if (maxLoc.y>=0) 
	{
		vec_far.push_back({vecxy[maxLoc.y].x,vecxy[maxLoc.y].y});

		// plot the farest point in principle orientation on image
		cv::circle(img, cv::Point(vecxy[maxLoc.y].x,vecxy[maxLoc.y].y), 1, cv::Scalar(255,255,255), 5, 8, 0);
	}

	//----orientation2------
    	Mat dist_mat1 = Mat(vecxy1.size(), 1, CV_64F);
	for (int i=0; i< vecxy1.size(); i++)
	{
		dist_mat1.at<double>(i, 0) = abs(vec1[i].first);	
	}
	// find the min and max dispc
	cv::minMaxLoc(dist_mat1, &min, &max, &minLoc, &maxLoc);
	cout<<"dist: min="<<min<<",minLoc="<<minLoc.y<<",max="<<max<<",maxLoc="<<maxLoc.y<<endl;
	if (maxLoc.y>=0) 
	{
		vec_far.push_back({vecxy1[maxLoc.y].x,vecxy1[maxLoc.y].y});

		// plot the farest point in principle orientation on image
		cv::circle(img, cv::Point(vecxy1[maxLoc.y].x,vecxy1[maxLoc.y].y), 1, cv::Scalar(255,255,255), 5, 8, 0);
	}
	
	return vec_far;
}
