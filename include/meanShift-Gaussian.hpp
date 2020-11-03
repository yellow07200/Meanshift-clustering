#ifndef MEANSHIFT-GAUSSIAN_HPP
#define MEANSHIFT-GAUSSIAN_HPP

#include <vector>

#include "Point.hpp"

#include <ros/ros.h>
#include <opencv2/core/core.hpp>


#define MAX_ITERATIONS 100

#define DVSW 240
#define DVSH 180

void meanshiftCluster_Gaussian(cv::Mat dataPts, std::vector<double> *clusterCenterX, std::vector<double> *clusterCenterY, std::vector<double> *clusterCenterZ, std::vector<int> *point2Clusters, double bandwidth, cv::Mat TclusterVotes);

void findClosestCluster(std::vector<int> *assign_matches, std::vector<double> clustCentX, std::vector<double> clustCentY, std::vector<double> prev_clustCentX, std::vector<double> prev_clustCentY);

#endif //MEANSHIFT-GAUSSIAN_HPP
