#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <chrono>

#include "meanShift-huang.hpp"
#include "meanShift-Gaussian.hpp"
#include "Point.hpp"
#include "pca.hpp"
#include "FindFarestPoints.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include "opencv2/imgproc/imgproc_c.h"
#include <opencv/highgui.h>
//#include "opencv2/imgproc/imgproc.hpp"

#include <complex>
#include <omp.h>


#include <cv_bridge/cv_bridge.h>

#include<std_msgs/Float32MultiArray.h>



#define min(x,y) ((x)<(y)?(x):(y))

#define Num_Cluster 7

#define bandwidth 0.17//0.15

int iter=0;
uint64_t first_timestamp;
double final_timestamp;

bool firstevent = true;
double firsttimestamp;

std::vector<Point> centroid;
std::vector<Point> centroid_filter;
std::vector<Point> centroid_filter_prev;

std::vector<cv::Point> cluster_points;
std::vector<std::vector<cv::Point> > clusters_test;

std::vector<Point> centroid_final;
std::vector<Point> centroid_prev;
std::vector<std::vector<int> > centroid_traj_x;
std::vector<std::vector<int> > centroid_traj_y;
std::vector<std::vector<Point> > centroid_traj;
std::vector<Point> original_centroid;

std::vector<int> events_num;

std::vector<int> a;

int last_size;

float elapsedTime;
int t_count;	

	int LBI;
	int TP;
	int GT;

namespace dvs_test {

Meanshift::Meanshift(ros::NodeHandle & nh) : nh_(nh) //, ros::NodeHandle nh_private//
{
	numRows=DVSH; numCols=DVSW;
  	event_sub_=nh_.subscribe("/dvs/events",1,&Meanshift::eventsCallback_simple,this);///dvs/events ///feature_events
	center_orien_pub = nh_.advertise<std_msgs::Float32MultiArray>("center_info", 1);
}

void Meanshift::eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg)
{
	points.clear();
	centroid.clear();
	centroid_filter.clear();
	centroid_final.clear();
	points_cluster.clear();
	points_vector.clear();
	points_ID0.clear(); // points in the first cluster;
	
	//iter+=1;
	std::cout<<"iter="<<iter<<std::endl;

	height=msg->height; //180
	width=msg->width; //240
	events_size=msg->events.size();
	//events_num.push_back(events_size);

	sparseMatrix=cv::Mat::zeros(height,width,CV_8UC1);

	ROS_INFO("events_size: [%d]",events_size);
	//ROS_INFO("total_events_size: [%d]",events_num.size());
	//std::cout<<"total_events_size="<<event_num[0]<<std::endl;
		
	// store in mat
	cv_bridge::CvImage cv_image;
        cv_image.image = cv::Mat(msg->height, msg->width, CV_8UC3);//CV_8U-->mono
        cv_image.image = cv::Scalar(128, 128, 128);//cv::Scalar(128);

	cv_bridge::CvImage cv_image1;
        cv_image1.image = cv::Mat(msg->height, msg->width, CV_8UC3);//CV_8U-->mono
        cv_image1.image = cv::Scalar(128, 128, 128);//cv::Scalar(128);

	/*cv_bridge::CvImage cv_imageID;
        cv_imageID.image = cv::Mat(msg->height, msg->width, CV_8UC3);//CV_8U-->mono
        cv_imageID.image = cv::Scalar(128, 128, 128);//cv::Scalar(128);*/

	cv::Mat imageID;
        imageID = cv::Mat(msg->height, msg->width, CV_8UC3);//CV_8U-->mono
        imageID = cv::Scalar(128,128,128);

	//cv::Mat timestamp_matrix = cv::Mat(numRows, numCols, CV_64F);
	//timestamp_matrix = cv::Scalar(0.);
	//cv::Mat timestamp_matrix_out;

	cv::Mat clusters=cv::Mat(2, events_size, CV_64F, cv::Scalar::all(0.));
        //#pragma omp parallel for
	for (int i = 0; i < events_size; ++i)
	{

        	const int x = msg->events[i].x;
        	const int y = msg->events[i].y;

        	cv_image.image.at<cv::Vec3b>(cv::Point(x, y)) = ( \
			msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));

        	//cv_image1.image.at<cv::Vec3b>(cv::Point(x, y)) = ( \
			msg->events[i].polarity == true ? cv::Vec3b(255, 0, 0) : cv::Vec3b(0, 0, 255));

		clusters.at<double>(cv::Point(i, 0))= x;
		clusters.at<double>(cv::Point(i, 1))= y;
		//clusters.at<double>(cv::Point(i, 2))= 0;

	}

if (events_size>60)
{
	iter+=1;
	// count events per pixels with polarity
	//cv::Mat data=cv::Mat(3, events_size, CV_64F, cv::Scalar::all(0.));

	uint64_t first_timestamp = msg->events[0].ts.toNSec();
	double final_timestamp =  (1E-6*(double)(msg->events[(msg->events.size())-1].ts.toNSec()-first_timestamp));
	std::vector<bool> activeEvents(msg->events.size());

	// packets
	int counterIn = 0;
	int counterOut = 0;

	int beginEvent = 0;
	int packet;

	 if(DVSW==240)
		packet = 1800;
	 else
		packet = 1500;

	//packet = 100;
	int ClusterID=0;
	
	while(beginEvent < events_size )
	{
		  counterIn = 0;
		  counterOut = 0;

		cv::Mat data=cv::Mat(3, min(packet, events_size-beginEvent), CV_64F, cv::Scalar::all(0.));
		cv::Mat data_cluster=data;
		if(firstevent)
		{
			firsttimestamp = (1E-6*(double)(msg->events[10].ts.toNSec()));
			firstevent = false;
		}

		//double maxTs;
		//int posx, posy;
		//double usTime = 10.0;
		double ts;
		int temp=min(beginEvent+packet, events_size);
		//events_num.push_back(temp);
		for (int i = beginEvent; i < min(beginEvent+packet, events_size); i++)//
		{
			  //SELECT SMALL PACKETS OF MAXIMUM 1000 events
			  const int x = msg->events[counterIn].x;
			  const int y = msg->events[counterIn].y;

			  double event_timestamp =  (1E-6*(double)(msg->events[counterIn].ts.toNSec()-first_timestamp));//now in usecs 

			 ts = (1E-6*(double)(msg->events[i].ts.toNSec())) - firsttimestamp;

			data.at<double>(cv::Point(counterOut, 0))= (double)x/numCols;
			data.at<double>(cv::Point(counterOut, 1))= (double)y/numRows;
			//data.at<double>(cv::Point(counter, 2))= event_timestamp/final_timestamp;//normalized
			data.at<double>(cv::Point(counterOut, 2))= event_timestamp/final_timestamp;//normalized
			double tau = 10000;
			//data.at<double>(cv::Point(counterOut, 2))= exp(-(final_timestamp-event_timestamp)/tau);//normalized
			activeEvents.at(counterIn)=true;
	
			data_cluster.at<int>(cv::Point(counterOut, 0))= x;
			data_cluster.at<int>(cv::Point(counterOut, 1))= y;
			data_cluster.at<int>(cv::Point(counterOut, 2))= 0;


			p.push_back(msg->events[i].polarity);

			sparseMatrix.at<uchar>(y,x)=255;

			//ts.push_back(timestamp);
			point.push_back(x);
                	point.push_back(y);
			points.push_back({x,y});
			//points3d.push_back({x,y,timestamp});			

			counterIn++;
			counterOut++;

		}
		double last_timestamp =  (1E-6*(double)(msg->events[counterIn-1].ts.toNSec()));//now in usecs

		cv::Mat clusterCenters;
		cv::Mat segmentation=cv::Mat(numRows, numCols, CV_8UC3);
		segmentation = cv::Scalar(128,128,128);

		cv::Mat traj = cv::Mat(numRows, numCols, CV_8UC3);
		traj = cv::Scalar(128,128,128);

		std::vector<double> clustCentX, clustCentY, clustCentZ;
		std::vector<double> prev_clustCentX, prev_clustCentY;

		std::vector<int> point2Clusters;
		std::vector<int> positionClusterColor;
		std::vector<int> assign_matches;

		cv::Mat TclusterVotes;

		//double bandwidth = 0.15; // 0.16;//0.15;
		std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
		meanshiftCluster_Gaussian(data, &clustCentX, &clustCentY, &clustCentZ, &point2Clusters, bandwidth, TclusterVotes);
 		std::chrono::high_resolution_clock::time_point end =std::chrono::high_resolution_clock::now();
   		elapsedTime += std::chrono::duration_cast<std::chrono::duration<float>>(end - start).count();
		t_count++;
		std::cout << "Elapsed time: " << elapsedTime << " s, count=" <<t_count<< std::endl;

		std::cout<<"clustCentX.size="<<clustCentX.size()<<",clustCentX[0]="<<clustCentX[0]<<std::endl;
		std::cout<<"point2Clusters.size="<<point2Clusters.size()<<",point2Clusters0="<<point2Clusters[0]<<std::endl;
		//cluster_points.clear();
		//std::vector<std::vector<cv::Point> > clusters_test;
		clusters_test.clear();
		//findClosestCluster(&assign_matches, clustCentX, clustCentY, prev_clustCentX, prev_clustCentY);


		beginEvent +=packet;

		int inter=0;
		float xx,yy;
		std::vector<Point> centroid;
		cv::Point cluster_point_cv;
		for (int i=0; i<clustCentX.size();i++)//auto
		{
			//std::vector<std::vector<double> > cluster_point = i->getCentroid();
			inter++;
			int co=inter*30;
			//Point cluster_point = i->getCentroid();
			cluster_points.clear();
			int count_points=0;
			//std::cout<<"points_num="<<point2Clusters[i]<<std::endl;
			//if (point2Clusters[i]<=0) break;
			//if (inter==1) 
			//{
				xx=std::floor(clustCentX[i]*240);//cluster_point[0];//-----
				yy=std::floor(clustCentY[i]*180);//cluster_point[1];//------
				centroid.push_back({xx,yy});
				cluster_point_cv=cv::Point(xx,yy);//yy,xx//------
				//cv::circle(cv_image.image, cluster_point_cv, 1, cv::Scalar(255,255,255), 5, 8, 0);//--------	

				// ------merge close centroids of data packet------------
				for (int j = beginEvent; j < min(beginEvent+packet, events_size); j++)
				{
					const int x = msg->events[j].x;
					const int y = msg->events[j].y;
					//float dis2=pow((x-centroid[i][0]),2)+pow((y-centroid[i][1]),2);
					float dis2=pow((x-xx),2)+pow((y-yy),2);
					if (dis2<(bandwidth/2)*(bandwidth/2)*240*180)
					{
						std::cout<<"dis2="<<dis2<<std::endl;
						cluster_points.push_back({x,y});
						//cluster.addPoint({x,y});
						//cluster.addPoint({x,y});			
						//clusters.push_back({x,y});
						count_points++;	
					}

				}
				clusters_test.push_back(cluster_points);
				std::cout<<"count_points="<<count_points<<std::endl;
				// ------filter events noise----------------------------
				if (count_points>0) //50 for raw events //4 for corner events //40 for last working file
				{
					//ClusterID++;
					//centroid_filter.push_back({xx,yy});//{ID,}
					cv::circle(cv_image.image, cv::Point(xx,yy), 1, cv::Scalar(255,255,255), 5, 8, 0);//--------
					//cv::circle(cv_image1.image, cluster_point_cv, 1, cv::Scalar(255,255,255), 5, 8, 0);//------------
				}
				//centroid_filter_prev=centroid_filter;
				centroid_filter.push_back({xx,yy});
		}


		//if (events_size-beginEvent < packet )
		std::cout<<"cluster_size="<<clustCentX.size()<<",ClusterID="<<ClusterID<<",centroid_filter="<<centroid_filter.size()<<std::endl;

	}//while packet
	std::cout<<"xxx1"<<std::endl;
	// merge close centroids of all data
	int sign=0;
	if (centroid_filter.size()>0)
	{
		for(int i=0;i<centroid_filter.size();i++)
		{
			int x1=centroid_filter[i][0];
			int y1=centroid_filter[i][1];
			for(int j=i;j<centroid_filter.size();j++)
			{
				int x2=centroid_filter[j][0];
				int y2=centroid_filter[j][1];
				int dis=pow((x1-x2),2)+pow((y1-y2),2);
	
				if (dis<(bandwidth/2)*(bandwidth/2)*240*180)// && x1!=0 && y1!=0)//800)// 400 last working //i!=j && 
				{
					x1=	(x1+centroid_filter[j][0])/2;	//x1=centroid_filter[i][0];
					y1=	(y1+centroid_filter[j][1])/2;	//y1=centroid_filter[i][1];
					centroid_filter[j][0]=0;centroid_filter[j][1]=0;	
					//cv::circle(cv_image.image, cv::Point(x1,y1), 1, cv::Scalar(0,255,0), 5, 8, 0);
					sign++;
				}
			}
			if (x1!=0 && y1!=0)
			{
				ClusterID++;
				centroid_final.push_back({x1,y1});//ClusterID,<<centroid_final[1][1]
				cv::circle(cv_image.image, cv::Point(x1,y1), 1, cv::Scalar(0,255,0), 5, 8, 0);//------------------
			}
		}
	}
	std::cout<<"centroid_filter.size="<<centroid_filter.size()<<",centroid_final.size="<<centroid_final.size()<<",ClusterID="<<ClusterID<<std::endl;
	int cs=centroid_final.size();
	for (int i=0; i<cs; i++)
	{
		std::cout<<"points=("<<centroid_final[i][0]<<","<<centroid_final[i][1]<<")"<<std::endl;
	}

	//if (centroid_final.size()>0) // remove the situation that nothing detected //
	//{
	// get initial centroid 
	std::vector<Point> temp_center=cluster_center;//centroid_final;
	if (iter==1) temp_center=centroid_final;
	std::vector<Point> curr_center=centroid_final; 
	cluster_center.clear();
	
	int vector_x=0,vector_y=0;
	int ii_t=0; int vector_i=0;

	// track the centroid----------------------------------
	//if (iter<10) cluster_center=centroid_final;//iter==1
	//else
	//{
	if (iter>0)
	{ 
		LBI+=centroid_final.size(); 
		GT+=7;
		//if (centroid_final.size()<=7) TP+=centroid_final.size();
		std::cout<<"cluster_center.size()="<<cluster_center.size()<<",temp_center.size()="<<temp_center.size()<<std::endl;
		for (int cc=0; cc<temp_center.size(); cc++) // last iteration
		{
			int ii=0; 	std::vector<Point> center_ii;
			int xii=0, yii=0; //int vector_i=0;
			int xii_bar=0, yii_bar=0;
			int ccx=temp_center[cc][0];
			int ccy=temp_center[cc][1];
			for (int tc=0; tc<centroid_final.size(); tc++) 
			{
				int tcx=centroid_final[tc][0]; 
				int tcy=centroid_final[tc][1]; 		
				double disTwoCen=pow((ccx-tcx),2)+pow((ccy-tcy),2);
				if (disTwoCen<(bandwidth*bandwidth/4*180*240)) 
				{
					ii++; 
					center_ii.push_back({tcx,tcy});
					//std::cout<<"I find "<< cc <<"th center!!!"<<",center_ii.size()="<<center_ii.size()<<std::endl;
					curr_center[tc][0]=0; curr_center[tc][1]=0; 
					ii_t++;
				}
				else
				{
					curr_center[tc][0]=tcx; curr_center[tc][1]=tcy; // new center remained 
				} 
			}

			//if (ii>0) TP++;
			//std::cout<<"ii="<<ii<<",TP="<<TP<<std::endl;

			//---check the repeated centers and get the final centers of current iteration----------
			if (ii>0)//(center_ii.size()>1) 
			{
				vector_i++; TP++;
				for (int j=0; j<center_ii.size(); j++)
				{
					xii+=center_ii[j][0];
					yii+=center_ii[j][1];
					std::cout<<"ii>0, point("<<center_ii[j][0]<<","<<center_ii[j][1]<<")"<<std::endl;
				}
				xii/=center_ii.size();
				yii/=center_ii.size();
				cluster_center.push_back({xii,yii});
				vector_x=vector_x+xii-ccx; vector_y=vector_x+yii-ccy;
				std::cout<<"I find "<< cc <<"th center!!!---new"<<",TP="<<TP<<std::endl;
			}
			else if (vector_i!=0)
			{
				cluster_center.push_back({ccx+vector_x/vector_i,ccy+vector_y/vector_i});
				std::cout<<"I find "<< cc <<"th center!!!---dis"<<std::endl;
			}
			else 
			{
				cluster_center.push_back({ccx,ccy});
				std::cout<<"I find "<< cc <<"th center!!!---dis"<<std::endl;
			}
			center_ii.clear();
		}

		std::vector<Point> cluster_center_remain;

		for (int cc=0; cc<curr_center.size(); cc++)
		{
			int it=0;
			if (curr_center[cc][0]!=0 && curr_center[cc][1]!=0)
			{
				it++;
				//cluster_center_remain[it][0]=curr_center[cc][0]; 
				//cluster_center_remain[it][1]=curr_center[cc][1];
				cluster_center_remain.push_back({curr_center[cc][0],curr_center[cc][0]});
			}
		}
	
		if (cluster_center_remain.size()==1) 
		{
			cluster_center.push_back({cluster_center_remain[0][0],cluster_center_remain[0][0]});
			std::cout<<"I find new center!!!---dis"<<std::endl;
		}
		else if ( cluster_center_remain.size()>0)
		{//cluster_center_remain.size>1;
			for (int cci=0; cci<cluster_center_remain.size(); cci++)
			{
				std::vector<Point> center_ii;
				int ii=0;
				int ccx=cluster_center_remain[cci][0];
				int ccy=cluster_center_remain[cci][1];
				for (int ccj=cci+1; ccj<cluster_center_remain.size(); ccj++)
				{
					int tcx=cluster_center_remain[ccj][0];
					int tcy=cluster_center_remain[ccj][1];
					double disTwoCen=pow((ccx-tcx),2)+pow((ccy-tcy),2);
				if (disTwoCen<(bandwidth*bandwidth/4*180*240)) 
				{
					ii++; 
					center_ii.push_back({tcx,tcy});
					//std::cout<<"I find "<< cc <<"th center!!!"<<",center_ii.size()="<<center_ii.size()<<std::endl;
					curr_center[ccj][0]=0; curr_center[ccj][1]=0; 
					ii_t++;
				}
				else
				{
					curr_center[ccj][0]=tcx; curr_center[ccj][1]=tcy;
				} 
				}
				if (ii>0)
				{
					int sumx=0, sumy=0;
					for (int cck=0; cck<center_ii.size(); cck++)
					{
						sumx+=center_ii[cck][0]; sumy+=center_ii[cck][1];
					}
			
				}
			}//end-for
			cluster_center_remain.clear();

			std::cout<<"ii_t="<<ii_t<<",cluster_center.size="<<cluster_center.size()<<std::endl;
			//}// end-else
			//}// end-ifif (centroid_final.size()>3)
		} //end-else if ( cluster_center_remain.size()>0)
	} //end-if iter>1

	//-----find the cluster points--------------
	cv::RNG rng(12345);
	std::vector<std::vector<int> > points_cluster_x;
	std::vector<cv::Point> row;
	double angle_ID0=0;
	//std::vector<std::vector<int> > points_cluster_x(2, std::vector<int> (2, 0));
	//std::vector<std::vector<cv::Point> > points_cluster_x(2, std::vector<cv::Point> (2, {0,0}));

	//if (cluster_center.size()==0) 

	std::cout<<"000cluster_center.size()="<<cluster_center.size()<<std::endl;
	for (int w=0;w<cluster_center.size();w++)//centroid_final //cluster_center
	{
		int ID=w+1;
		int ID_color=ID*10;

		bool sign=0;
		
		cv::Mat clustersID=cv::Mat(2, events_size, CV_64F, cv::Scalar::all(0.));

		cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
		for (int k=0;k<events_size;k++)
		{
			int x_mat=clusters.at<double>(cv::Point(k, 0));
			int y_mat=clusters.at<double>(cv::Point(k, 1));
			//std::cout<<"clusters.at(k, 0))="<<clusters.at<double>(cv::Point(k, 0))<<std::endl;
			double distp=pow((x_mat-cluster_center[w][0]),2)+pow((y_mat-cluster_center[w][1]),2);
			cluster_ID.push_back(ID);//w
			if (distp<(bandwidth/2)*(bandwidth/2)*240*180)
			{	//points_vector.push_back(points[k]);
				//clusters.at<double>(cv::Point(k, 3))=ID;
				cluster_ID.pop_back();
				cluster_ID.push_back(ID);
				sign=1;
				//if (ID==3) 
					points_vector.push_back({x_mat,y_mat});
					//points_cluster.emplace_back(std::vector<cv::Point> {{x_mat,y_mat},{x_mat,y_mat}}); // correct expression
				//std::cout<<"x_mat="<<x_mat<<",y_mat="<<y_mat<<std::endl;
				cv::circle(cv_image1.image, cv::Point(x_mat,y_mat), 0.2, color, 0.2, 1, 0);//cv::Scalar(ID_color,ID_color,ID_color)
				if (w==0)
				{
					//---record the first cluster-----------------------
					std::vector<std::vector<cv::Point> > contourID;
        				imageID.at<cv::Vec3b>(cv::Point(x_mat,y_mat))= cv::Vec3b(255, 0, 0);
					points_ID0.push_back({x_mat,y_mat});
				}
			}
			
		}
		if (sign) 
		{
			angle = getOrientation(points_vector, cv_image1.image); //orientation in radians //-3.14/2;
			std::cout<<"ID="<<ID<<",angle="<<angle<<",cluster_center.size()="<<cluster_center.size()<<std::endl;//",size="<<points_cluster[0].size()<<std::endl;
		}
		if (w==0) angle_ID0=angle; // record the principle axis orientation of 1st cluster //+3.14/2

		//clusters.at<double>(cv::Point(i, 0))
		//points_cluster.push_back(points_vector);
		points_vector.clear();
		
		//row.push_back(points_cluster[0][0]);
	}

	std::cout<<"points_ID0.size()="<<points_ID0.size()<<std::endl;
	//----find the farest points in the principle axis from the centroid------------

	std::vector<cv::Point> vec_far;
	if (cluster_center.size()>0) 
		vec_far=FindFar(points_ID0, cluster_center[0][0], cluster_center[0][1], angle_ID0, imageID);

	//std::cout<<"points_cluster.size="<<points_cluster.size()<<",points_vector.size="<<points_vector.size()<<std::endl;
	
	std::cout<<"cluster_size="<<cluster_points.size()<<",centroid_final.size="<<centroid_final.size()<<",centroid_prev="<<centroid_prev.size()<<std::endl;
	std::cout<<"points_size="<<points.size()<<std::endl;
	//float Recall=TP/GT*100;
	std::cout<<"TP="<<TP<<",LBI="<<LBI<<",GT="<<GT<<std::endl;
	//std::cout<<"Recall="<<Recall<<"%"<<std::endl;

	//	last_size=centroid_final.size();
	//	centroid_prev=centroid_final;

	std_msgs::Float32MultiArray cen_msg;

	if (cluster_center.size()>0) 
	{
	cen_msg.data.push_back(angle);
	cen_msg.data.push_back(cluster_center[0][0]);
	cen_msg.data.push_back(cluster_center[0][1]);
	for (int i=0; i<vec_far.size(); i++)
	{
		cen_msg.data.push_back(vec_far[i].x);
		cen_msg.data.push_back(vec_far[i].y);
	}
	}
	else 
	{
		cen_msg.data.push_back(-1);
		cen_msg.data.push_back(-1);
		cen_msg.data.push_back(-1);
	}
	center_orien_pub.publish(cen_msg);
 
	//---------------Evaluation-----------------
	//if (centroid_final.size()<=Num_Cluster) int TP_t=centroid_final.size();

	/*cv::namedWindow("sparseMatrix", CV_WINDOW_AUTOSIZE);
	cv::imshow("sparseMatrix", sparseMatrix);
	cv::waitKey(1);*/

	cv::namedWindow("image", CV_WINDOW_AUTOSIZE);//CV_WINDOW_AUTOSIZE
	cv::imshow("image", cv_image.image);
	cv::waitKey(1);

	cv::namedWindow("image1", CV_WINDOW_AUTOSIZE);
	cv::imshow("image1", cv_image1.image);
	cv::waitKey(1);

	cv::namedWindow("imageID", CV_WINDOW_AUTOSIZE);
	cv::imshow("imagID", imageID);
	cv::waitKey(1);

	/*cv::namedWindow("imaget", CV_WINDOW_AUTOSIZE);
	cv::imshow("imaget", temporalIm);
	cv::waitKey(1);*/
} //end-if(event_size>60)
} //end-void



} //namespace
