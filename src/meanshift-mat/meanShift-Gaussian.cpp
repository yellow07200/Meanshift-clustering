#include <cstring>
#include <cmath>
#include <cstdlib>
#include "meanShift-Gaussian.hpp"

//#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define max(x,y) ((x)>(y)?(x):(y))
#define min(x,y) ((x)<(y)?(x):(y))


#define _2D_to_linear(row, col, maxCol) ((row)*(maxCol)+(col))

const int MAX_DISTANCE = 500;
cv::Mat weight0;

void meanshiftCluster_Gaussian(cv::Mat dataPts, std::vector<double> *clusterCenterX, std::vector<double> *clusterCenterY, std::vector<double> *clusterCenterZ, std::vector<int> *point2Clusters, double bandwidth, cv::Mat TclusterVotes)
{
	//Initialization
	int numPts = dataPts.cols;
	int numDim = dataPts.rows;
	int numClust = 0;
	double bandSq = bandwidth*bandwidth;
	cv::Range rng = cv::Range(0,numPts-1);

	cv::Mat onesAux = cv::Mat::ones(1, dataPts.cols, dataPts.type());

	std::vector<int> initPtInds;
	for (int i = rng.start; i <= rng.end; i++) initPtInds.push_back(i);
	double stopThresh = 1E-3*bandwidth; 						//when mean has converged
	int numInitPts = numPts; 								//track if a points been seen already
	cv::Mat beenVisitedFlag = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); 	//number of points to posibaly use as initilization points
	cv::Mat clusterVotes; 		//used to resolve conflicts on cluster membership

	double lambda = 10.;
	double myMeanX, myMeanY, myMeanZ, myOldMeanX, myOldMeanY, myOldMeanZ, totalWeightX, totalWeightY, totalWeightZ;
	double temp1, temp2, temp3, myMeanX1, myMeanY1, myMeanZ1;
	double temp11, temp22, temp33;
	double dev_myMeanX, dev_myMeanY, dev_myMeanZ, dev_totalWeightX, dev_totalWeightY, dev_totalWeightZ;	

	int stInd;

	int tempInd;

	while (numInitPts>0)
	{

		tempInd = (rand()%numInitPts);	 //pick a random seed point
		//ROS_ERROR_STREAM("numInitPts ="<<numInitPts);

		stInd = initPtInds[tempInd];	//use this point as start of mean
		myMeanX = dataPts.at<double>(cv::Point(stInd, 0));//intilize mean to this points location
		myMeanY = dataPts.at<double>(cv::Point(stInd, 1));
		myMeanZ = dataPts.at<double>(cv::Point(stInd, 2));
		cv::Mat thisClusterVotes = cv::Mat(1, numPts, CV_32S, cv::Scalar::all(0)); //used to resolve conflicts on cluster membership
		//ROS_ERROR_STREAM("Before getting into while myMean = ["<<myMeanX<<", "<<myMeanY<<", "<<myMeanZ<<"]");
		cv::Mat_<double> myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);


		while(true)
		{
			cv::Mat myDataPts; //3xcols;
			dataPts.copyTo(myDataPts);

			cv::Mat diff = myDataPts - myMean*onesAux; //3xcols; myMean:3x1; //myMean*onesAux: after shift;
			cv::Mat diffdiff = diff.mul(diff); //3xcols;
			cv::Mat sqDistToAll = diffdiff.row(0) +diffdiff.row(1) + diffdiff.row(2);//1xcols;	//dist squared from mean to all points still active
			//x1+y1+z1-x2-y2-z2;
			double dev_diff=(double)(cv::sum(diff.row(0)+diff.row(1)+diff.row(2))[0]);//sum(x-x')

			cv::Mat inInds = (sqDistToAll < bandSq); // Puts 255 wherever it is true
			cv::add(thisClusterVotes, cv::Scalar(1), thisClusterVotes, inInds); //add a vote for all the in points belonging to this cluster //thisClusterVotes: 1xcols; // if (inInds==1), thisClusterVotes= 1 + thisClusterVotes;
			cv::Mat mask3 = cv::repeat(inInds==0, 3, 1); // CV_EXPORTS Mat repeat(const Mat& src, int ny, int nx);--> 3xcols;

			myDataPts.setTo(0, mask3);
			double numOfOnes = (double) (cv::sum(inInds)[0])/255; //inInds is active if inInds[i]==255

			myOldMeanX = myMeanX; myOldMeanY = myMeanY; myOldMeanZ = myMeanZ;

			myMeanX = cv::sum(myDataPts.row(0))[0]/numOfOnes;
			myMeanY = cv::sum(myDataPts.row(1))[0]/numOfOnes;
			myMeanZ = cv::sum(myDataPts.row(2))[0]/numOfOnes;

			myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);

			//ROS_ERROR_STREAM("MIDDLE myMean = "<<myMean);

			diff = myDataPts - myMean*onesAux;
			diffdiff = diff.mul(diff);

			cv::Mat weight;
			cv::exp(diffdiff/lambda, weight);
			//weight=0.9*weight+0.1*weight0;   //-------------------------learning rate for updating weight

			// gradient;
			cv::Mat dev_weight;
			dev_weight=weight.mul(dev_diff)*(2);//(dev_diff);
			//std::cout<<"weights="<<(double) (cv::sum(weight)[0])<<" dweights="<<(double) (cv::sum(dev_weight)[0])<<std::endl;

			weight.setTo(0., mask3); //Again, because the exp and the x - u make invalid values non-zero (inInds)
			dev_weight.setTo(0., mask3);
			cv::Mat wData = weight.mul(myDataPts);

			cv::Mat dev_wData = dev_weight.mul(myDataPts);

			totalWeightX = cv::sum(weight.row(0))[0];
			totalWeightY = cv::sum(weight.row(1))[0];
			totalWeightZ = cv::sum(weight.row(2))[0];
			myMeanX = cv::sum(wData.row(0))[0]/totalWeightX;
			myMeanY = cv::sum(wData.row(1))[0]/totalWeightY;
			myMeanZ = cv::sum(wData.row(2))[0]/totalWeightZ;

			dev_totalWeightX = cv::sum(dev_weight.row(0))[0];
			dev_totalWeightY = cv::sum(dev_weight.row(1))[0];
			dev_totalWeightZ = cv::sum(dev_weight.row(2))[0];
			//dev_myMeanX = (dev_totalWeightX==0) ?0:sum(dev_wData.row(0))[0]/dev_totalWeightX;//-myOldMeanX;
			//dev_myMeanY = (dev_totalWeightY==0) ?0:sum(dev_wData.row(1))[0]/dev_totalWeightY;//-myOldMeanY;
			//dev_myMeanZ = (dev_totalWeightZ==0) ?0:sum(dev_wData.row(2))[0]/dev_totalWeightZ;//-myOldMeanZ;
			dev_myMeanX = cv::sum(dev_wData.row(0))[0]/dev_totalWeightX;//-myOldMeanX;
			dev_myMeanY = cv::sum(dev_wData.row(1))[0]/dev_totalWeightY;//-myOldMeanY;
			dev_myMeanZ = cv::sum(dev_wData.row(2))[0]/dev_totalWeightZ;//-myOldMeanZ;

			temp1 = myMeanX+0.5*(myMeanX-myOldMeanX);//dev_myMeanX; //0.5*(myMeanX-myOldMeanX);
			temp2 = myMeanY+0.5*(myMeanY-myOldMeanY);//dev_myMeanY; //0.5*(myMeanY-myOldMeanY);
			temp3 = myMeanZ+0.5*(myMeanZ-myOldMeanZ);//dev_myMeanZ; //0.5*(myMeanZ-myOldMeanZ);

			temp11 = dev_myMeanX+0.1*(dev_myMeanX-myOldMeanX);//dev_myMeanX; //0.5*(myMeanX-myOldMeanX);
			temp22 = dev_myMeanY+0.1*(dev_myMeanY-myOldMeanY);//dev_myMeanY; //0.5*(myMeanY-myOldMeanY);
			temp33 = dev_myMeanZ+0.1*(dev_myMeanZ-myOldMeanZ);//dev_myMeanZ; //0.5*(myMeanZ-myOldMeanZ);

			myMeanX=temp11;
			myMeanY=temp22;
			myMeanZ=temp33;

			/*temp1 = min(temp1,1);
			temp2 = min(temp2,1);
			temp3 = min(temp3,1);
*/
			//std::cout<<"mean="<<myMeanX<<",temp="<<temp1<<std::endl;

			myMeanX1 = myMeanX+0.5*dev_myMeanX; 
			myMeanY1 = myMeanY+0.5*dev_myMeanY;
			myMeanZ1 = myMeanZ+0.5*dev_myMeanZ;

			//std::cout<<"mean="<<myMeanX<<",temp="<<temp1<<",dev="<<myMeanX1<<std::endl;//dev_myMeanX

			//std::cout<<"wData="<<sum(wData.row(0))[0]<<",dev_wData="<<sum(dev_wData.row(0))[0]<<std::endl;//-------------
			//std::cout<<"totalWeightX="<<totalWeightX<<",dev_totalWeightX="<<dev_totalWeightX<<std::endl;
			//std::cout<<"x="<<myMeanX<<"y="<<myMeanY<<"z="<<myMeanZ<<std::endl;
			//std::cout<<"dev1="<<dev_myMeanX<<"dev2="<<dev_myMeanY<<"dev3="<<dev_myMeanZ<<std::endl;

			//myMean = ( cv::Mat_<double>(3, 1) << myMeanX, myMeanY, myMeanZ);
			//myMean = ( cv::Mat_<double>(3, 1) << dev_myMeanX, dev_myMeanY, dev_myMeanZ);
			//myMean = ( cv::Mat_<double>(3, 1) << temp1, temp2, temp3);
			myMean = ( cv::Mat_<double>(3, 1) << temp11, temp22, temp33);
			//myMean = ( cv::Mat_<double>(3, 1) << myMeanX1, myMeanY1, myMeanZ1);
			//ROS_ERROR_STREAM("AFTER: myMean = "<<myMeanX<<","<<myMeanY<<","<<myMeanZ);
			//exit(0);

			beenVisitedFlag.setTo(1, inInds);
			// if mean doesn't move much stop this cluster
			if((myMeanX-myOldMeanX)*(myMeanX-myOldMeanX) + (myMeanY-myOldMeanY)*(myMeanY-myOldMeanY) + (myMeanZ-myOldMeanZ)*(myMeanZ-myOldMeanZ) < stopThresh*stopThresh)
			{
				//check for merge posibilities
				//ROS_ERROR_STREAM("Dentro!! ");
				int mergeWith = -1;
				double distToOther;
				for(int cN = 0; cN<numClust; cN++) //Careful!! cN goes from 1 to numClust!!!
				{
					double distToOther = (myMeanX - (*clusterCenterX)[cN])*(myMeanX - (*clusterCenterX)[cN]) + (myMeanY - (*clusterCenterY)[cN])*(myMeanY - (*clusterCenterY)[cN]) + (myMeanZ - (*clusterCenterZ)[cN])*(myMeanZ - (*clusterCenterZ)[cN]); //distance from posible new clust max to old clust max

					if(distToOther < (bandwidth/2)*(bandwidth/2))  //if its within bandwidth/2 merge new and old
					{
						mergeWith = cN;
						break;
					}
				}

				if(mergeWith > -1)
				{
					(*clusterCenterX)[mergeWith] = 0.5*(myMeanX+(*clusterCenterX)[mergeWith]);
					(*clusterCenterY)[mergeWith] = 0.5*(myMeanY+(*clusterCenterY)[mergeWith]);
					(*clusterCenterZ)[mergeWith] = 0.5*(myMeanZ+(*clusterCenterZ)[mergeWith]);

					cv::Mat newClusterVotes = cv::Mat(clusterVotes.row(mergeWith)) + thisClusterVotes;
					newClusterVotes.copyTo(clusterVotes.row(mergeWith));
				}
				else
				{
					(*clusterCenterX).push_back(myMeanX);    //record the mean value
					(*clusterCenterY).push_back(myMeanY);
					(*clusterCenterZ).push_back(myMeanZ);
					numClust = numClust+1;                   //increment clusters

					clusterVotes.push_back(thisClusterVotes);  // add the new row for the new cluster

				}
				break;
			} // moving distance < threshold
			weight0=weight.clone();
		} // while (true)

		std::vector<int> newInitPtInds;	//points not yet visited
		for(int i=0; i<numPts; i++)
			if (beenVisitedFlag.at<int>(cv::Point(i, 0)) == 0)
				newInitPtInds.push_back(i);
		initPtInds = newInitPtInds;
		numInitPts = initPtInds.size(); //number of active points in set
	}// while (numInitPts>0)

	TclusterVotes = clusterVotes.t(); //TclusterVotes=[cluster_num x points_num in each cluster]
	cv::Point minLoc;
	cv::Point maxLoc;
	double min, max;
		std::cout<<"TclusterVotes(i).size="<<TclusterVotes.rows<<std::endl;
	for(int i=0; i<TclusterVotes.rows; i++)
	{
		cv::minMaxLoc(cv::Mat(TclusterVotes.row(i)), &min, &max, &minLoc, &maxLoc);
		(*point2Clusters).push_back(maxLoc.x);
		/*std::cout<<"TclusterVotes"<<i<<"=";
		for (int j=0; j<TclusterVotes.cols; j++)	
		{
			//if (TclusterVotes.at<double>(i,j)<0.001) TclusterVotes.at<double>(i,j)=0;
			std::cout<<TclusterVotes.at<double>(i,j)<<",";
		}
		std::cout<<"."<<std::endl;*/
	}
	ROS_ERROR_STREAM("Number of clusters " <<numClust<<" and "<<(*clusterCenterX).size());
	ROS_ERROR_STREAM("Tclusters rows " <<TclusterVotes.rows<<" cols "<<TclusterVotes.cols<<",TclusterVotes.size="<<TclusterVotes.size()<<",clusterVotes.size="<<clusterVotes.size()<<",point2Clusters.size="<<point2Clusters->size()<<","<<TclusterVotes.at<double>(1,6));
	

}

