//#ifndef MEANSHIFT_MEANSHIFT_HPP
//#define MEANSHIFT_MEANSHIFT_HPP
#ifndef MEANSHIFT_H_
#define MEANSHIFT_H_

#include <ros/ros.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <vector>

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>

#include "Point.hpp"
#define MAX_ITERATIONS 100


#define DVSW 240
#define DVSH 180


namespace dvs_test
{

class Meanshift {
public:
  Meanshift(ros::NodeHandle & nh);//, ros::NodeHandle nh_private);
  //virtual ~Meanshift();

private:
  ros::NodeHandle nh_;

  void eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg);

  ros::Subscriber event_sub_;
	ros::Publisher center_orien_pub;

	int height,width;
	double timestamp;
	int events_size;
	std::vector<double> point;
	std::vector<double> ts;
	std::vector<Point> points;
	std::vector<int> cluster_ID;
	std::vector<Point> cluster_center;	
	std::vector<cv::Point> points_vector;
	std::vector<cv::Point> points_ID0;
	std::vector<std::vector<cv::Point> > points_cluster;

	std::vector<Point > points3d;
	cv::Mat sparseMatrix;
	std::vector<int> events_num;
	std::vector<bool> p;

  	int numRows, numCols, maxIterNum;
	float timeDivider;
	std::vector<Point> pointsp;

	double angle;
	double grasping_angle;

	//frame of times
	cv::Mat BGAFframe;
	
};

	//std::vector<Cluster> meanShift(const std::vector<Point> &points, float bandwidth);//, int events_num);



}//namespace


#endif // MEANSHIFT_H_

