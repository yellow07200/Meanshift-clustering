#ifndef FAR_HPP
#define FAR_HPP

#include <vector>
#include <utility>

#include "Point.hpp"

#include <ros/ros.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>

#include <iostream>

#include "pca.hpp"


std::vector<cv::Point> FindFar(std::vector<cv::Point> &pts, int cx, int cy, double theta, cv::Mat &img);

#endif FAR_HPP
