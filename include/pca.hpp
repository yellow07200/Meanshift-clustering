#ifndef PCA_HPP
#define PCA_HPP

#include <vector>

#include "Point.hpp"

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>


#define MAX_ITERATIONS 100

#define DVSW 240
#define DVSH 180

void drawAxis(cv::Mat& img, cv::Point p, cv::Point q, cv::Scalar colour, const float scale = 0.2);
double getOrientation(std::vector<cv::Point> &pts, cv::Mat &img);

#endif PCA_HPP//MEANSHIFT-GAUSSIAN_HPP
