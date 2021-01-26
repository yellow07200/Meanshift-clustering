#include <time.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <chrono>

#include "meanShift-huang-3D.hpp"
#include "meanShift-Gaussian-3D.hpp"
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

#define bandwidth 0.5//0.25

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

std::vector<Point> cluster_center_final;
std::vector<int> cluster_center_final_x;
std::vector<int> cluster_center_final_y;
std::vector<cv::Point> cv_cluster_center_final;

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
	//cam_info_subs_ = nh_.subscribe("/dvs/camera_info", 1, &Meanshift::CameraInfoCallback, this);
	pcl_sub_=nh_.subscribe("/point_cloud_Clustering", 1, &Meanshift::PointCloud2Callback, this);
	center_orien_pub = nh_.advertise<std_msgs::Float32MultiArray>("center_info", 1);
}
void Meanshift::PointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& pcl_info)
{
	ROS_INFO("PCl Info Received=%d",pcl_info->data);       
	 
}
/* void Meanshift::CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& camera_info)
{
	ROS_INFO("Camera Info Received");        
} */

void Meanshift::eventsCallback_simple(const dvs_msgs::EventArray::ConstPtr& msg)
{
	ROS_INFO("Next call_back");
	points.clear();
	centroid.clear();
	centroid_filter.clear();
	centroid_final.clear();
	points_cluster.clear();
	points_vector.clear();
	points_ID0.clear(); // points in the first cluster;
	ROS_INFO("Next call_back_000");
	//iter+=1;
	//std::cout<<"iter="<<iter<<std::endl;

	height=msg->height; //180
	width=msg->width; //240
	events_size=msg->events.size();
	//events_num.push_back(events_size);

	sparseMatrix=cv::Mat::zeros(height,width,CV_8UC1);

	//ROS_INFO("events_size: [%d]",events_size);
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

	cv::Mat clusters=cv::Mat( 2, events_size, CV_64F, cv::Scalar::all(0.));
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

	if (iter==900)//(GraspID!=prev_GraspID)
	{
		iter=0; 
		cluster_center.clear();
	}

	ROS_INFO("Next call_back_111");
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
			//std::cout<<"loop in while"<<std::endl;
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
			std::cout<<"size="<<temp-beginEvent<<std::endl;
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

			//std::cout<<"clustCentX.size="<<clustCentX.size()<<",clustCentX[0]="<<clustCentX[0]<<std::endl;
			//std::cout<<"point2Clusters.size="<<point2Clusters.size()<<",point2Clusters0="<<point2Clusters[0]<<std::endl;
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
					float dis2=0;
					int x_size=0;
					//std::cout<<"beginEvent"<<beginEvent<<",min="<<min(beginEvent+packet, events_size)<<std::endl;
					// ------merge close centroids of data packet------------
					for (int j = 0; j < min(beginEvent+packet, events_size); j++)
					{
						const int x = msg->events[j].x;
						const int y = msg->events[j].y;
						//float dis2=pow((x-centroid[i][0]),2)+pow((y-centroid[i][1]),2);
						dis2=pow((x-xx),2)+pow((y-yy),2);
						if (dis2<((bandwidth/2)*(bandwidth/2)*240*180*1.2))
						{
							//std::cout<<"dis2="<<dis2<<std::endl;
							cluster_points.push_back({x,y});
							//cluster.addPoint({x,y});
							//cluster.addPoint({x,y});			
							//clusters.push_back({x,y});
							count_points++;	
						}
						x_size++;
						//std::cout<<"x_size="<<x_size<<std::endl;
					}
					clusters_test.push_back(cluster_points);
					//std::cout<<"count_points="<<count_points<<",dis2="<<dis2<<",dis1="<<(bandwidth/2)*(bandwidth/2)*240*180*10<<",xx.size="<<clustCentX.size()<<",x.size="<<x_size<<std::endl;
					// ------filter events noise----------------------------
					if (count_points>10) //50 for raw events //4 for corner events //40 for last working file
					{
						//ClusterID++;
						centroid_filter.push_back({xx,yy});//{ID,}
						cv::circle(cv_image.image, cv::Point(xx,yy), 1, cv::Scalar(255,255,255), 5, 8, 0);//--------
						//cv::circle(cv_image1.image, cluster_point_cv, 1, cv::Scalar(255,255,255), 5, 8, 0);//------------
					}
					//centroid_filter_prev=centroid_filter;
					//centroid_filter.push_back({xx,yy});
			}


			//if (events_size-beginEvent < packet )
			//std::cout<<"cluster_size="<<clustCentX.size()<<",ClusterID="<<ClusterID<<",centroid_filter="<<centroid_filter.size()<<std::endl;

		//}//while packet----------------------------------------------------------
		//std::cout<<"xxx1"<<std::endl;
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
		// points groups of centroid_final
		cv::Mat centroid_final_mat=cv::Mat(3, events_size, CV_64F, cv::Scalar::all(255));//cv::Mat(numRows, numCols, CV_8UC3);
		//centroid_final_mat = cv::Scalar(128,128,128);
		int cfs=0;
		cv::Mat clusters_copy=clusters;	
	 /* if (centroid_final.size()>0)
		{
		for (int m=0; m<centroid_final.size();m++)
		{
			int cfx=centroid_final[m][0];
			int cfy=centroid_final[m][1];
			for (int n=0;n<events_size;n++)
			{
				int x_mat=clusters_copy.at<double>(cv::Point(n, 0));
				int y_mat=clusters_copy.at<double>(cv::Point(n, 1));
				double distance=pow((cfx-x_mat),2)+pow((cfy-y_mat),2);
				if (distance<(bandwidth/2)*(bandwidth/2)*240*180)
				{
					cfs++;
					clusters_copy.at<double>(cv::Point(n, 0))=0;
					clusters_copy.at<double>(cv::Point(n, 1))=0;
					//std::cout<<"cfs="<<cfs<<",distance="<<distance<<std::endl;
					//clusters.at<double>(cv::Point(i, 0))= x;
					centroid_final_mat.at<double>(cv::Point(cfs, 0))=x_mat;
					centroid_final_mat.at<double>(cv::Point(cfs, 1))=y_mat;
					centroid_final_mat.at<double>(cv::Point(cfs, 2))=m;
				}
			}
		}
		}  */
		//if (centroid_final.size()>0) // remove the situation that nothing detected //
		//{
		// get initial centroid 
		std::vector<Point> temp_center=cluster_center;//centroid_final;
		if (iter==11) temp_center=centroid_final; 
		std::vector<Point> curr_center=centroid_final; 
		cluster_center.clear();
		cluster_center_final.clear();
		cluster_center_final_x.clear();
		cluster_center_final_y.clear();
		cv_cluster_center_final.clear();
		int vector_x=0,vector_y=0;
		int ii_t=0; int vector_i=0;

		// track the centroid----------------------------------
		//if (iter<10) cluster_center=centroid_final;//iter==1
		//else
		//{
		if (iter>10)
		{ 
			LBI+=centroid_final.size(); 
			GT+=7;
			std::vector<Point> center_found;
			//cv::Mat center_found_mat=cv::Mat(2, centroid_final.size(), CV_64F, cv::Scalar::all(0));
			//if (centroid_final.size()<=7) TP+=centroid_final.size();
			//std::cout<<"cluster_center.size()="<<cluster_center.size()<<",temp_center.size()="<<temp_center.size()<<std::endl;
			for (int cc=0; cc<temp_center.size(); cc++) // last iteration
			{
				int elseif=0; int elseif_x=0, elseif_y=0;
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
					if (disTwoCen<(bandwidth*bandwidth/4*180*240)) // /2
					{
						ii++; 
						center_ii.push_back({tcx,tcy});
						center_found.push_back({tcx,tcy});
						//std::cout<<"I find "<< cc <<"th center!!!"<<",center_ii.size()="<<center_ii.size()<<std::endl;
						curr_center[tc][0]=0; curr_center[tc][1]=0; 
						/* center_found_mat.at<double>(cv::Point(0, cc))=0;
						center_found_mat.at<double>(cv::Point(1, cc))=0; */
						ii_t++;
					}
					else
					{
						curr_center[tc][0]=tcx; curr_center[tc][1]=tcy; // new center remained 
						/* center_found_mat.at<double>(cv::Point(0, cc))=tcx;
						center_found_mat.at<double>(cv::Point(1, cc))=tcy; */
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
						//std::cout<<"ii>0, point("<<center_ii[j][0]<<","<<center_ii[j][1]<<")"<<std::endl;
					}
					xii/=center_ii.size();
					yii/=center_ii.size();
					cluster_center.push_back({xii,yii});
					vector_x=vector_x+xii-ccx; vector_y=vector_y+yii-ccy;
					//std::cout<<"I find "<< cc <<"th center!!!---new"<<",TP="<<TP<<std::endl;
					//std::cout<<"ii>0 ,point("<<xii<<","<<yii<<")"<<std::endl;
				}
				else if (vector_i!=0)
				{
					cluster_center.push_back({ccx+vector_x/vector_i,ccy+vector_y/vector_i});
					//std::cout<<"else if ,point("<<ccx+vector_x/vector_i<<","<<ccy+vector_y/vector_i<<")"<<std::endl;
				}
				else 
				{
					cluster_center.push_back({ccx,ccy});
					//std::cout<<"else ,point("<<ccx<<","<<ccy<<")"<<std::endl;
				}
				/* else if (vector_i!=0 )//&& ii==0) // if points disappeared, estimate it
				{
					//elseif++;
					if (elseif==1)
					{
						cluster_center.push_back({ccx+vector_x/vector_i,ccy+vector_y/vector_i});
						elseif_x=ccx+vector_x/vector_i; elseif_y=ccx+vector_y/vector_i;
						std::cout<<"else if ==1,point("<<ccx+vector_x/vector_i<<","<<ccy+vector_y/vector_i<<")"<<std::endl;
					}
					else
					{
						double elseif_dis=pow((elseif_x-vector_x/vector_i),2)+pow((elseif_y-vector_y/vector_i),2);
						if (elseif_dis>900 && vector_x/vector_i<240 && vector_y/vector_i <180)
						{
							cluster_center.push_back({ccx+vector_x/vector_i,ccy+vector_y/vector_i});
							elseif_x=ccx+vector_x/vector_i; elseif_y=ccx+vector_y/vector_i;
							std::cout<<"elseif_x="<<elseif_x<<",elseif_y="<<elseif_y<<std::endl;
							std::cout<<"else if >900,point("<<ccx+vector_x/vector_i<<","<<ccy+vector_y/vector_i<<")"<<std::endl;
						}
					}
					//std::cout<<"I find "<< cc <<"th center!!!---dis"<<std::endl;
					//std::cout<<"else if ,point("<<ccx+vector_x/vector_i<<","<<ccy+vector_y/vector_i<<")"<<std::endl;
				} */
				/* else 
				{
					cluster_center.push_back({ccx,ccy});
					//std::cout<<"I find "<< cc <<"th center!!!---dis"<<std::endl;
					std::cout<<"else ,point("<<ccx<<","<<ccy<<")"<<std::endl;
				} */

				center_ii.clear();

			}

			// clear repeated centers in cluster_center 
			cv::Mat cluster_center_mat=cv::Mat(2, cluster_center.size(), CV_64F, cv::Scalar::all(0));
			for (int tc=0; tc<cluster_center.size(); tc++)
			{
				int ccx=cluster_center[tc][0];
				int ccy=cluster_center[tc][1];
				//int count_repeat=0;
				for (int tcf=tc; tcf<cluster_center.size(); tcf++)
				{
					int ccx1=cluster_center[tcf][0];
					int ccy1=cluster_center[tcf][1];
					int count_repeat=0;
					double dd=pow((ccx-ccx1),2)+pow((ccy-ccy1),2);
					if (dd>900 || dd==0)
					{
						count_repeat++;
						cluster_center_mat.at<double>(cv::Point(0, tcf))=ccx;
						cluster_center_mat.at<double>(cv::Point(1, tcf))=ccy;
					}
					else if (dd<=900 || count_repeat>1)
					{
						cluster_center_mat.at<double>(cv::Point(0, tcf))=0;
						cluster_center_mat.at<double>(cv::Point(1, tcf))=0;
					}
				}
			}
			std::cout<<"saved in mat, cluster_center.size()="<<cluster_center.size()<<std::endl;

			// save in vector 
			for (int tc=0; tc<cluster_center.size(); tc++)
			{
				int xxx=0; int yyy=0;
				//std::cout<<"tc="<<tc<<std::endl;
				int ccx=cluster_center_mat.at<double>(cv::Point(0, tc));//cluster_center[tc][0];
				int ccy=cluster_center_mat.at<double>(cv::Point(1, tc));//cluster_center[tc][1];
				//std::cout<<"ccx="<<ccx<<",ccy="<<ccy<<std::endl;
				if (ccx!=0)
				{
					//std::cout<<"ccx!=0"<<std::endl;
					//Point temp_point=Point({ccx,ccy}); 
					//std::cout<<"ccx!=1"<<std::endl;
					xxx=ccx; yyy=ccy;
					//std::cout<<"ccx!=2"<<std::endl;
					/* cluster_center_final_x.push_back(ccx);
					std::cout<<"ccx!=2"<<std::endl;
					cluster_center_final_y.push_back(ccy); */
					//cluster_center_final.push_back(temp_point);//{ccx,ccy});
					cv_cluster_center_final.push_back(cv::Point(xxx,yyy));
					std::cout<<"cluster_center_final=("<<ccx<<","<<ccy<<")"<<std::endl;
				}
				/* if (xxx!=0)
				{
					std::cout<<"ccx!=3"<<std::endl;
					//cluster_center_final.push_back({xxx,yyy});
					

					std::cout<<"ccx!=4"<<std::endl;
				} */
				//std::cout<<"ccx!=5"<<std::endl;
			}
			std::cout<<"saved in cluster_center_final"<<std::endl;
			std::cout<<"centroid_final.size="<<centroid_final.size()<<", cv_cluster_center_final.size="<<cluster_center_final.size()<<std::endl;
			// add new points detected 
			if (centroid_final.size()>cv_cluster_center_final.size())
			{
				for (int tf=0; tf<centroid_final.size(); tf++)
				{
					int cfx=centroid_final[tf][0];
					int cfy=centroid_final[tf][1];
					int sign_repeat=0;
					//std::cout<<"sign_repeat="<<sign_repeat<<std::endl;
					for (int tc=0; tc<cv_cluster_center_final.size(); tc++)
					{
						int ccx=cv_cluster_center_final[tc].x;//cluster_center_final[tc][0];
						int ccy=cv_cluster_center_final[tc].y;//cv_cluster_center_final[tc][1];
						double dd=pow((cfx-ccx),2)+pow((cfy-ccy),2);
						if (dd>50) 
						{
							sign_repeat++;
						}
					}
					if (sign_repeat==cv_cluster_center_final.size()) 
					{
						//cluster_center_final.push_back({cfx,cfy});
						cv_cluster_center_final.push_back(cv::Point(cfx,cfy));
						/* cluster_center_final_x.push_back(cfx);
						cluster_center_final_y.push_back(cfy); */
						std::cout<<"new point=("<<cfx<<","<<cfy<<")"<<std::endl;
					}
					//std::cout<<"sign_repeat="<<sign_repeat<<std::endl;
					//std::cout<<"tf="<<tf<<std::endl;
				}
				//std::cout<<"iter>10"<<std::endl;
			}

			/* for (int tc=0; tc<centroid_final.size(); tc++)
			{
				int x_found=center_found_mat.at<double>(cv::Point(0, tc));
				int y_found=center_found_mat.at<double>(cv::Point(1, tc));
				if (x_found!=0 && y_found!=0)
				{
					//cluster_center.push_back({x_found,y_found});
					std::cout<<"new,point("<<x_found<<","<<y_found<<")"<<std::endl;
				}
			} */

			/* std::vector<Point> cluster_center_remain;

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

				//std::cout<<"ii_t="<<ii_t<<",cluster_center.size="<<cluster_center.size()<<std::endl;
				//}// end-else
				//}// end-ifif (centroid_final.size()>3)
			} //end-else if ( cluster_center_remain.size()>0) */
		} //end-if iter>1
		//std::cout<<"endif"<<std::endl;
		}//while packet -------------------------------------------------------------------------------------
		//std::cout<<"end while packet"<<std::endl;
		/* for (int t=0;t<cluster_center.size();t++)
		{
			std::cout<<"cluster_center=("<<cluster_center[t][0]<<","<<cluster_center[t][1]<<")"<<std::endl;
		} */

		//-----find the cluster points--------------
		cv::RNG rng(12345);
		std::vector<std::vector<int> > points_cluster_x;
		std::vector<cv::Point> row;
		double angle_ID0=0;
		
		//std::vector<std::vector<int> > points_cluster_x(2, std::vector<int> (2, 0));
		//std::vector<std::vector<cv::Point> > points_cluster_x(2, std::vector<cv::Point> (2, {0,0}));

		//if (cluster_center.size()==0) 
		std::vector<cv::Point> vec_far;
		if (cv_cluster_center_final.size()>0)
		{
			//std::cout<<"000cluster_center.size()="<<cluster_center.size()<<std::endl;
			//ROS_INFO("Cluster_Center.size=%d, Cluster_Center_final.size=%d",cluster_center.size(),cv_cluster_center_final.size());
			//std::cout<<"end-1"<<std::endl;
			int cv_cluster_center_final_size=cv_cluster_center_final.size();
			//std::cout<<"end-int"<<std::endl;
			//ROS_INFO("cv_cluster_center_final_size=%d",cv_cluster_center_final_size);
			for (int w=0;w<cv_cluster_center_final_size;w++)//centroid_final //cluster_center
			{
				//std::cout<<"w="<<w<<std::endl;
				int xw=cv_cluster_center_final[w].x;
				int yw=cv_cluster_center_final[w].y;
				//std::cout<<"x="<<cv_cluster_center_final[w].x<<",y="<<cv_cluster_center_final[w].y<<std::endl;
				int ID=w+1;
				int ID_color=ID*10;

				bool sign=0;
				
				//cv::Mat clustersID=cv::Mat(2, events_size, CV_64F, cv::Scalar::all(0.));
				// store points belonging to one cluster
				//std::cout<<"2w"<<w<<std::endl;
				cv::Scalar color = cv::Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
				//std::cout<<"3w"<<w<<std::endl;
				for (int k=0;k<events_size;k++)
				{
					//std::cout<<"before-before"<<std::endl;
					int x_mat=clusters.at<double>(cv::Point(k,0));
					int y_mat=clusters.at<double>(cv::Point(k,1));
					//std::cout<<"clusters.at(k, 0))="<<clusters.at<double>(cv::Point(k, 0))<<std::endl;
					//double distp=pow((x_mat-cluster_center_final[w][0]),2)+pow((y_mat-cluster_center_final[w][1]),2);
					//std::cout<<"before"<<std::endl;
					double distp=pow((x_mat-xw),2)+pow((y_mat-yw),2);
					//std::cout<<"after"<<std::endl;
					cluster_ID.push_back(ID);//w
					//std::cout<<"after-1"<<std::endl;
					if (distp<(bandwidth/2)*(bandwidth/2)*240*180*2)
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
						//std::cout<<"after-2"<<std::endl;
						if (w==0)
						{
							//---record the first cluster-----------------------
							std::vector<std::vector<cv::Point> > contourID;
							imageID.at<cv::Vec3b>(cv::Point(x_mat,y_mat))= cv::Vec3b(255, 0, 0);
							points_ID0.push_back({x_mat,y_mat});
						}
					}
					//std::cout<<"after-after"<<std::endl;
					
				}
				//std::cout<<"w="<<w<<std::endl;
				//ROS_INFO("points_ID0.size0=%d",points_ID0.size());
				if (sign) 
				{
					angle = getOrientation(points_vector, cv_image1.image); //orientation in radians //-3.14/2;
					//std::cout<<"111"<<std::endl;
					//double angle0 = getOrientation(points_ID0, imageID);
					//std::cout<<"222"<<std::endl;
					//std::cout<<"ID="<<ID<<",angle="<<angle<<",cluster_center.size()="<<cluster_center.size()<<std::endl;//",size="<<points_cluster[0].size()<<std::endl;
				}
				std::cout<<"333"<<std::endl;
				if (w==0) angle_ID0=angle; // record the principle axis orientation of 1st cluster //+3.14/2
				//std::cout<<"angle_ID0="<<angle_ID0<<std::endl;
				//clusters.at<double>(cv::Point(i, 0))
				//points_cluster.push_back(points_vector);
				//std::cout<<"444"<<std::endl;
				points_vector.clear();
				
				//row.push_back(points_cluster[0][0]);
			}

			//std::cout<<"points_ID0.size()="<<points_ID0.size()<<std::endl;
			ROS_INFO("points_ID0.size=%d",points_ID0.size());
			//----find the farest points in the principle axis from the centroid------------
			
			//std::vector<cv::Point> vec_far;
			//if (cv_cluster_center_final.size()>0) 
				vec_far=FindFar(points_ID0, cv_cluster_center_final[0].x, cv_cluster_center_final[0].y, angle_ID0, imageID);//cv_image1.image);//
		}
		//std::cout<<"points_cluster.size="<<points_cluster.size()<<",points_vector.size="<<points_vector.size()<<std::endl;
		
		//std::cout<<"cluster_size="<<cluster_points.size()<<",centroid_final.size="<<centroid_final.size()<<",centroid_prev="<<centroid_prev.size()<<std::endl;
		//std::cout<<"points_size="<<points.size()<<std::endl;
		//float Recall=TP/GT*100;
		//std::cout<<"TP="<<TP<<",LBI="<<LBI<<",GT="<<GT<<std::endl;
		//std::cout<<"Recall="<<Recall<<"%"<<std::endl;

		//	last_size=centroid_final.size();
		//	centroid_prev=centroid_final;
		ROS_INFO("angle=%f",angle_ID0);

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
			cen_msg.data.push_back(-1);
			cen_msg.data.push_back(-1);
		}
		//std::cout<<"555"<<std::endl;
		center_orien_pub.publish(cen_msg);
		ROS_INFO("xxx1");
		//---------------Evaluation-----------------
		//if (centroid_final.size()<=Num_Cluster) int TP_t=centroid_final.size();

		/*cv::namedWindow("sparseMatrix", CV_WINDOW_AUTOSIZE);
		cv::imshow("sparseMatrix", sparseMatrix);
		cv::waitKey(1);*/
		//}//while packet
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
		ROS_INFO("Iter=%d",iter);
	} //end-if(event_size>60)
	//std::cout<<"666"<<std::endl;
} //end-void

void CameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &camera_info)
{
	ROS_INFO("Camera Info Received");        
}


} //namespace
