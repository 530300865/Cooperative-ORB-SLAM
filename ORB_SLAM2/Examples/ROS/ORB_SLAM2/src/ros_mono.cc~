/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "std_msgs/String.h"

//lcm 头文件
//#include<iostream>

#include<lcm/lcm-cpp.hpp>


#include "lcmPose/lcmMapPointWorldPose.hpp"
#include "lcmPose/lcmVectorMapPoint.hpp"
#include "lcmRefPose/lcmRefMapPointWorldPose.hpp"
#include "lcmRefPose/lcmVectorRefMapPoint.hpp"
#include "lcmDrawKeyFrame/lcmDrawCovGraph.hpp"
#include "lcmDrawKeyFrame/lcmDrawGraph.hpp"
#include "lcmDrawKeyFrame/lcmDrawKF.hpp"
#include "lcmDrawKeyFrame/lcmDrawKFSize.hpp"
#include "lcmDrawKeyFrame/lcmDrawSpanningTree.hpp"
#include "lcmDrawKeyFrame/lcmDrawLoop.hpp"
#include "lcmDrawKeyFrame/lcmDrawKFVector.hpp"
#include "lcmDrawKeyFrame/lcmDrawKFGraph.hpp"
#include "lcmDrawKeyFrame/lcmDrawCovGraphTotal.hpp"

#include "lcmFrame/lcmFrameInfo.hpp"//  #include "lcmFrame" yu zhe ge tou wen jian chong tu
#include "lcmFrame/lcmFrameInfoVector.hpp"
#include "lcmFrame/lcmORBextractorAndParameter.hpp"
#include <map>

#include "KeyFrame.h"
#include "ORBextractor.h"

using namespace std;
using namespace cv;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM2::System* mpSLAM;
};

std::vector<cv::Point3f> Receive_mapPointVector;//存放传输过来的mapPoint世界坐标
std::vector<cv::Point3f> Receive_RefmapPointVector;//存放传输过来的reference mappoint的世界坐标
float KFSize[3];//w,h,z
std::vector<cv::Mat> Receive_DrawKFVector;//存放传输过来的keyframe的twc（mat矩阵)

//cov graph
//std::vector<cv::Point3f> Receive_CovGraphVector_OW;//CovGraph OW
//std::vector<cv::Point3f> Receive_CovGraphVector_OW2;//CovGraph OW2
std::vector<std::vector<cv::Point3f> > Receive_CovGraphVector_OWVector;//record ow,
std::vector<std::vector<cv::Point3f> > Receive_CovGraphVector_OW2Vector;//record ow,
std::vector<std::vector<std::vector<cv::Point3f>>> Received_keyFrame_CovGraph;
std::vector<std::vector<std::vector<std::vector<cv::Point3f>>>> Received_keyFrame;

std::vector<cv::Point3f> Receive_SpanTreeVector_OW;
std::vector<cv::Point3f> Receive_SpanTreeVector_OW2;

std::vector<cv::Point3f> Receive_LoopVector_OW;
std::vector<cv::Point3f> Receive_LoopVector_OW2;

std::vector<int> mnIdVector;
std::map<int, double> mTimeStampVector;
std::map<int, cv::Mat> mImGrayVector;
std::map<int, cv::Mat> mTcw;

int nFeatures;
float fScaleFactor;
int nLevels ;
int fIniThFAST ;
int fMinThFAST;

float mbf ;
float mThDepth;
cv::Mat mK(3,3,CV_32F);
cv::Mat mDistCoef(4,1,CV_32F);



class Handler 
    {


        public:
            ~Handler() {}
	    
	     //std::vector<cv::Point3f>mapPointVector;
            
   	 void handleMessage_ORBextractorAndParameters(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmFrame::lcmORBextractorAndParameter *sendlcmparameters)
		{
			printf("Received message on channel \"%s\":\n", chan.c_str());
			   nFeatures = sendlcmparameters->nFeatures;
		
			   fScaleFactor = sendlcmparameters->fScaleFactor;
			
   			   nLevels = sendlcmparameters->nLevels;
   		
    			   fIniThFAST = sendlcmparameters->fIniThFAST;
    		
 			   fMinThFAST = sendlcmparameters->fMinThFAST;
 		
 			   mbf = sendlcmparameters->mbf;
 			
 			   mThDepth= sendlcmparameters->mThDepth;
 		

 			  
 			  // cv:Mat mK;
 			   //cv:Mat mDistCoef;
 			   /*
 			  cout<<"sendlcmparameters->mDistCoef[0][0]"<< sendlcmparameters->mDistCoef[0][0] <<endl;
 			  cout<<"sendlcmparameters->mDistCoef[1][0]"<< sendlcmparameters->mDistCoef[1][0] <<endl;
 			  cout<<"sendlcmparameters->mDistCoef[2][0]"<< sendlcmparameters->mDistCoef[2][0] <<endl;
 			  cout<<"sendlcmparameters->mDistCoef[3][0]"<< sendlcmparameters->mDistCoef[3][0] <<endl;
 			
 			  cout<<"sendlcmparameters->mDistCoef[0][0]"<< sendlcmparameters->mDistCoef[0][0] <<endl;
 			  cout<<"sendlcmparameters->mDistCoef[0][1]"<< sendlcmparameters->mDistCoef[0][1] <<endl;
 			  cout<<"sendlcmparameters->mDistCoef[0][2]"<< sendlcmparameters->mDistCoef[0][2] <<endl;
 			  cout<<"sendlcmparameters->mDistCoef[0][3]"<< sendlcmparameters->mDistCoef[0][3] <<endl;


 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[0][0] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[0][1] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[0][2] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[1][0] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[1][1] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[1][2] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[2][0] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[2][1] <<endl;
 			 cout<<"sendlcmparameters->mk"<< sendlcmparameters->mk[2][2] <<endl;
 			*/

 			   for(int i=0; i<4; i++){
		        		for(int j=0; j<1; j++){
		  			mDistCoef.at<float>(i,j) = sendlcmparameters->mDistCoef[i][j] ;
		        		}
		        	}

		
		        	for(int i=0;i<3;i++){
		        		for(int j=0;j<3;j++){
		            			mK.at<float>(i,j) = sendlcmparameters->mk[i][j] ;
		        		}
		        	}

		        	 cout<< "nFeatures" <<  nFeatures <<endl;
 			 cout<< "fScaleFactor" <<  fScaleFactor <<endl;
 			 cout<< "nLevels" <<  nLevels <<endl;
 			 cout<< "fIniThFAST" <<  fIniThFAST <<endl;
 			 cout<< "fMinThFAST" <<  fMinThFAST <<endl;
 			 cout<< "mk" << mK<<endl;
 			 cout<< "mDistCoef" << mDistCoef<<endl;
			// this funcition had bugs because  mk and mDistCoef should define as cv::Mat mK(3,3,CV_32F); cv::Mat mDistCoef(4,1,CV_32F); not cv::mat mk , cv::mat mDistCoef

		}

	    void handleMessage_MapPoint(const lcm::ReceiveBuffer* rbuf,
                    const std::string& chan, 
                    const lcmPose::lcmVectorMapPoint *sendVectorMapPoint)
		{
			std::vector<cv::Point3f> newemptyvector;

			printf("Received message on channel \"%s\":\n", chan.c_str());
			//mapPointVector.assign(sendVectorMapPoint->VectorMapPoint.begin(),sendVectorMapPoint->VectorMapPoint.end());
			
			//std::cout << "mapPoint size is "<<mapPointVector.size() << endl ;
			 for(size_t i=0, iend=sendVectorMapPoint->VectorMapPoint.size(); i<iend;i++)
   			 {
    			    float a =sendVectorMapPoint->VectorMapPoint[i].poseX;
     			    float b =sendVectorMapPoint->VectorMapPoint[i].poseY;
			    float c =sendVectorMapPoint->VectorMapPoint[i].poseZ;
			
			    int p = sendVectorMapPoint->VectorMapPoint[i].IfPoseClear;
  			    if(sendVectorMapPoint->num_point==0 || p==1)
			    {
				std::cout << "p: "<< p << endl ;
				std::cout << "mapPoint:"<<Receive_mapPointVector.size() << endl ;
				Receive_mapPointVector.swap(newemptyvector);

			    }
			    Receive_mapPointVector.push_back(cv::Point3f(a,b,c));
			 }
				
			/*
			if(sendlcmMapPointWorldPose->IfPoseClear==1)
			{
				mapPointVector.swap(newemptyvector);
				std::cout << "mapPoint size is "<<mapPointVector.size() << endl ;
				std::cout << "newemptyvector size is "<<newemptyvector.size() << endl ;
			}	
			
			mapPointVector.push_back(cv::Point3f(sendlcmMapPointWorldPose->poseX,
			sendlcmMapPointWorldPose->poseY,
			sendlcmMapPointWorldPose->poseZ));
			*/
			//std::cout << "IfPoseClear is "<<sendlcmMapPointWorldPose->IfPoseClear << endl ;
			//std::cout << "mapPoint x is "<<sendlcmMapPointWorldPose->poseX << endl ;
			//std::cout << "mapPoint y is "<<sendlcmMapPointWorldPose->poseY << endl ;
			//std::cout << "mapPoint z is "<<sendlcmMapPointWorldPose->poseZ << endl ;

			//std::cout << "mapPoint size is "<<mapPointVector.size() << endl ;
			//std::cout << "newemptyvector size is "<<newemptyvector.size() << endl ;
		}

	 void handleMessage_RefMapPoint(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmRefPose::lcmVectorRefMapPoint *sendVectorRefMapPoint)
		{
			printf("Received message on channel \"%s\":\n", chan.c_str());
			
			std::vector<cv::Point3f> newemptyRefvector;
			
			std::cout << "number of RefmapPoint:"<<sendVectorRefMapPoint->VectorRefMapPoint.size() << endl ;
			for(size_t i=0, iend=sendVectorRefMapPoint->VectorRefMapPoint.size(); i<iend;i++)
   			{
    			 	float a =sendVectorRefMapPoint->VectorRefMapPoint[i].poseX;
     			 	float b =sendVectorRefMapPoint->VectorRefMapPoint[i].poseY;
			 	float c =sendVectorRefMapPoint->VectorRefMapPoint[i].poseZ;
			
			 	int p = sendVectorRefMapPoint->VectorRefMapPoint[i].IfPoseClear;
  			 	if(sendVectorRefMapPoint->num_point==0 || p==1)
			    	{
					std::cout << "p: "<< p << endl ;
					std::cout << "RefmapPoint:"<<Receive_RefmapPointVector.size() << endl ;
					Receive_RefmapPointVector.swap(newemptyRefvector);
	
			   	 }
			    	Receive_RefmapPointVector.push_back(cv::Point3f(a,b,c));
			}
		}

		void handleMessage_DrawKFSize(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmDrawKeyFrame::lcmDrawKFSize *sendlcmDrawKFSize)
		{
			printf("Received message on channel \"%s\":\n", chan.c_str());
			KFSize[0] = sendlcmDrawKFSize->w;
			KFSize[1] = sendlcmDrawKFSize->h;
			KFSize[2] = sendlcmDrawKFSize->z;
		}

		void handleMessage_DrawKF(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const lcmDrawKeyFrame::lcmDrawKFVector *sendlcmDrawKFVector)
		{
			printf("Received message on channel \"%s\":\n", chan.c_str());
			
			//std::vector<cv::Point3f> newemptKFvector;
			cout << "sendlcmDrawKFVector->VectorDrawKF.size()" << sendlcmDrawKFVector->VectorDrawKF.size()<< endl;
			for(size_t i=0, iend=sendlcmDrawKFVector->VectorDrawKF.size(); i<iend;i++)
   			{
				//利用数组初始化mat矩阵
    			    	//float A[4][4]={0};
				//cv::Mat Twc = Mat(4,4,CV_32F,A);
				//"‘Mat’ was not declared in this scope" 需要加 using namespace cv;
				cv::Mat Twc = Mat(4,4,CV_32F);
				for(int m=0;m<4;m++)
	    			{	
					for(int n=0;n<4;n++)
					{	
						float a=sendlcmDrawKFVector->VectorDrawKF[i].Twc[m][n];
						Twc.at<float>(m,n) = a;
					}
				}

				//cout << "Twc "<< Twc <<endl;
				//cout << "Twc "<< Twc .size()<<endl;

			    	Receive_DrawKFVector.push_back(Twc);
			}
		}

		void handleMessage_DrawGraph(const lcm::ReceiveBuffer* rbuf, const std::string& chan,  const lcmDrawKeyFrame::lcmDrawGraph *sendlcmDrawGraph)
		{
			printf("Received message on channel \"%s\":\n", chan.c_str());

	
			//cov graph'
			//cout<< "sendlcmDrawGraph->VectorCovGraphVector.size()" <<    sendlcmDrawGraph->VectorCovGraphVector.size()<<endl;
			//the covgraph only  a liitle, the first several keyfarmes have the covgraph, the followed keyframe do not have.
			cout<<"VectorCovGraphVector" <<sendlcmDrawGraph->VectorCovGraphVector.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[0].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[1].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[2].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[3].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[4].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[5].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[6].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[7].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[8].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[9].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[10].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[11].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[12].VectorCovGraph.size() <<endl;
			cout<<"VectorCovGraph" <<sendlcmDrawGraph->VectorCovGraphVector[13].VectorCovGraph.size() <<endl;




			for(size_t m = 0, mend = sendlcmDrawGraph->VectorCovGraphVector.size(); m<mend; m++)
			{
				
			 	std::vector<cv::Point3f> Receive_CovGraphVector_OW;//CovGraph OW
				std::vector<cv::Point3f> Receive_CovGraphVector_OW2;//CovGraph OW2

				cout<< " m  "<< m <<endl;
			 		
			 	for(size_t i=0, iend=sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph.size(); i<iend;i++)
    			 	{
    			 		cout<<"i" << i <<endl;
    			 		float a_1 =sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph[i].pose1_X;
     			 		float b_1 =sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph[i].pose1_Y;
			 		float c_1 =sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph[i].pose1_Z;

			 		float a_2 =sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph[i].pose2_X;
     			 		float b_2 =sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph[i].pose2_Y;
			 		float c_2 =sendlcmDrawGraph->VectorCovGraphVector[m].VectorCovGraph[i].pose2_Z;
					/*
			 		cout<<"a_1" << a_1<<endl;
			 		cout<<"b_1" << b_1<<endl;
					cout<<"c_1" << c_1<<endl;
					cout<<"a_2" << a_2<<endl;
			 		cout<<"b_2" << b_2<<endl;
					cout<<"c_2" << c_2<<endl;
					*/
					Receive_CovGraphVector_OW.push_back(cv::Point3f(a_1,b_1,c_1));
			    		Receive_CovGraphVector_OW2.push_back(cv::Point3f(a_2,b_2,c_2));
			    		/*
			    		cout<< "Receive_CovGraphVector_OW.size()  " <<Receive_CovGraphVector_OW.size() <<endl;
			    		cout<< "Receive_CovGraphVector_OW[0].x " <<Receive_CovGraphVector_OW[0].x <<endl;
			    		cout<< "Receive_CovGraphVector_OW[0].y " <<Receive_CovGraphVector_OW[0].y <<endl;
			    		cout<< "Receive_CovGraphVector_OW[0].z " <<Receive_CovGraphVector_OW[0].z <<endl;
			    		cout<< "Receive_CovGraphVector_OW[1].x " <<Receive_CovGraphVector_OW[1].x <<endl;
			    		cout<< "Receive_CovGraphVector_OW[1].y " <<Receive_CovGraphVector_OW[1].y <<endl;
			    		cout<< "Receive_CovGraphVector_OW[1].z " <<Receive_CovGraphVector_OW[1].z <<endl;
			    		cout<< "Receive_CovGraphVector_OW[2].x " <<Receive_CovGraphVector_OW[2].x <<endl;
			    		cout<< "Receive_CovGraphVector_OW[2].y " <<Receive_CovGraphVector_OW[2].y <<endl;
			    		cout<< "Receive_CovGraphVector_OW[2].z " <<Receive_CovGraphVector_OW[2].z <<endl;
			    		cout<< "Receive_CovGraphVector_OW[3].x " <<Receive_CovGraphVector_OW[3].x <<endl;
			    		cout<< "Receive_CovGraphVector_OW[3].y " <<Receive_CovGraphVector_OW[3].y <<endl;
			    		cout<< "Receive_CovGraphVector_OW[3].z " <<Receive_CovGraphVector_OW[3].z <<endl;
			    		cout<< "Receive_CovGraphVector_OW[4].x " <<Receive_CovGraphVector_OW[4].x <<endl;
			    		cout<< "Receive_CovGraphVector_OW[4].y " <<Receive_CovGraphVector_OW[4].y <<endl;
			    		cout<< "Receive_CovGraphVector_OW[4].z " <<Receive_CovGraphVector_OW[4].z <<endl;
			    		cout<< "Receive_CovGraphVector_OW[5].x " <<Receive_CovGraphVector_OW[5].x <<endl;
			    		cout<< "Receive_CovGraphVector_OW[5].y " <<Receive_CovGraphVector_OW[5].y <<endl;
			    		cout<< "Receive_CovGraphVector_OW[5].z " <<Receive_CovGraphVector_OW[5].z <<endl;
			    		*/
			    	}

			    	//cout<< "Receive_CovGraphVector_OW.size()  " <<Receive_CovGraphVector_OW.size() <<endl;
				Receive_CovGraphVector_OWVector.push_back(Receive_CovGraphVector_OW);
				Receive_CovGraphVector_OW2Vector.push_back(Receive_CovGraphVector_OW2);

				//cout<<"Receive_CovGraphVector_OWVector.size()" <<Receive_CovGraphVector_OWVector.size() <<endl;
				//cout<<"Receive_CovGraphVector_OWVector[j]" <<Receive_CovGraphVector_OWVector[j] <<endl;
			}

		
			//cout<< "Received_keyFrame.size() " << Received_keyFrame.size()<<endl;
			//cout<< "Received_keyFrame[i].size() " <<Received_keyFrame[0].size() <<endl;
			//cout<< " Received_keyFrame[i][j][0].size() " <<  Received_keyFrame[0][0][0].size()<<endl;
			//cout<< " Received_keyFrame[i][j][1].size() " <<  Received_keyFrame[0][0][1].size()<<endl;

			//cout<< "Received_keyFrame_CovGraph size "<< Received_keyFrame_CovGraph.size()<<endl;//82,    41 keyframes
			//cout<< "Received_keyFrame_CovGraph size "<< Received_keyFrame_CovGraph[0].size()<<endl;//5
			//cout<< "Received_keyFrame_CovGraph size "<< Received_keyFrame_CovGraph[0][0]<<endl;//[0.0376789, -0.0116325, 0.0529988]
			//cout<< "ow size()" << Receive_CovGraphVector_OW.size()<<endl;
			//cout<< "ow2 size()"<<Receive_CovGraphVector_OW2.size()<<endl;
			//cout<< "ow " << Receive_CovGraphVector_OW<<endl;

			//cout<< "ow2 "<<Receive_CovGraphVector_OW2 <<endl;
			/*
			//spanning tree
			for(size_t i=0, iend=sendlcmDrawGraph->VectorSpanningTree.size(); i<iend;i++)
    			 {		
    			 	float a_1 =sendlcmDrawGraph->VectorSpanningTree[i].pose1_X;
     			 	float b_1 =sendlcmDrawGraph->VectorSpanningTree[i].pose1_Y;
			 	float c_1 =sendlcmDrawGraph->VectorSpanningTree[i].pose1_Z;

			 	float a_2 =sendlcmDrawGraph->VectorSpanningTree[i].pose2_X;
     			 	float b_2 =sendlcmDrawGraph->VectorSpanningTree[i].pose2_Y;
			 	float c_2 =sendlcmDrawGraph->VectorSpanningTree[i].pose2_Z;
				
			 	
			    	Receive_SpanTreeVector_OW.push_back(cv::Point3f(a_1,b_1,c_1));
			    	Receive_SpanTreeVector_OW2.push_back(cv::Point3f(a_2,b_2,c_2));
			 }
			//loop

			 for(size_t i=0, iend=sendlcmDrawGraph->VectorLoopVector.size(); i<iend;i++)
   			{
   				for(size_t j=0, iend=sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector.size(); j<iend;j++)
    			 	{	
    			 		float a_1 =sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector[j].pose1_X;
     			 		float b_1 =sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector[j].pose1_Y;
			 		float c_1 =sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector[j].pose1_Z;

			 		float a_2 =sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector[j].pose2_X;
     			 		float b_2 =sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector[j].pose2_Y;
			 		float c_2 =sendlcmDrawGraph->VectorLoopVector[i].DrawLoopVector[j].pose2_Z;
				
					Receive_LoopVector_OW.push_back(cv::Point3f(a_1,b_1,c_1));
			    		Receive_LoopVector_OW2.push_back(cv::Point3f(a_2,b_2,c_2));
				}
			 	
			    	
			}
			*/
		}
		void handleMessage_FrameInfo(const lcm::ReceiveBuffer* rbuf, const std::string& chan,  const lcmFrame::lcmFrameInfoVector *sendlcmFrameInfoVector)
		{
			cout<<"received frame size " <<sendlcmFrameInfoVector->FrameVector.size()<<endl;

			for(size_t i=0, iend=sendlcmFrameInfoVector->FrameVector.size(); i<iend;i++)
    			{	
    			 	
    				int mnId = sendlcmFrameInfoVector->FrameVector[i].mnId;
    			
    				double mTimeStamp = sendlcmFrameInfoVector->FrameVector[i].timestamp;
    			
    				cv::Mat Im(480,640, CV_32FC1);
    			
    				 for(int a=0; a<480; ++a)
			                 {
			                        for(int b =0; b<640; ++b)
			                       {
			                            	 Im.at<uchar>(a,b)= sendlcmFrameInfoVector->FrameVector[i].ImGray[a][b] ;
			                       }
                  		  	}
                  		  
                  		  	cv::Mat Tcw(4,4, CV_32F);
                  		  
    				 for(int a=0; a<4; ++a)
			                 {
			                        for(int b =0; b<4; ++b)
			                       {
			                            	 Tcw.at<uchar>(a,b)= sendlcmFrameInfoVector->FrameVector[i].mTcw[a][b] ;
			                       }
                  		  	}
                  		  	
                  		  	
                  		  	
                  		  	mnIdVector.push_back(mnId);
                  		  	mTimeStampVector.insert(pair<int, double>(mnId,mTimeStamp));
                  		  	mImGrayVector.insert(pair<int, cv::Mat>(mnId,Im));
                  		  	mTcw.insert(pair<int, cv::Mat>(mnId,Tcw));

                  		  	cout<< "mnIdVector" <<mnIdVector.size() <<endl;
                  		  	cout<<"mTimeStamp" << mTimeStampVector.size() <<endl;
                  		  	cout<<"mImGrayVector" << mImGrayVector.size() << endl;
                  		  	cout<< "mTcw" << mTcw.size()<<endl;

                  		  	//mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
                  		  	//mpVocabulary = new ORBVocabulary();  system.cc, created when the system was created. It can be used directly
                  		  	//mpORBextractorLeft =new  ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
                  		  	//mCurrentFrame = new Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
                  		  	//KeyFrame *pKF =  new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


			}
			cout<<  " frame send finished" <<endl;
		}

     };


/*
void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Mono path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);


    lcm::LCM lcm;
   // lcm::LCM lcmMapPoint;

      if(!lcm.good())
	{	
		cout << "lcm is not good "<< endl;
	}

     Handler handlerObject;
     //lcm.subscribe("sendlcmFrame", &Handler::handleMessage2, &handlerObject);
    
	//lcm.subscribe("sendlcmVectorMapPoint", &Handler::handleMessage_MapPoint, &handlerObject);
     
	//lcm.subscribe("sendlcmVectorRefMapPoint", &Handler::handleMessage_RefMapPoint, &handlerObject);
     
	lcm.subscribe("sendlcmDrawKFSize", &Handler::handleMessage_DrawKFSize, &handlerObject);

	lcm.subscribe("sendlcmDrawKF", &Handler::handleMessage_DrawKF, &handlerObject);

	lcm.subscribe("sendlcmDrawGraph", &Handler::handleMessage_DrawGraph, &handlerObject);
	//decoding error "",意味着lcm有关的代码错误，缺少变量，变量名错误，变量赋值错误等等
	lcm.subscribe("sendlcmFrameInfo", &Handler::handleMessage_FrameInfo, &handlerObject);
	lcm.subscribe("sendlcmORBextractorAndParameters", &Handler::handleMessage_ORBextractorAndParameters, &handlerObject);

    	 while(0 == lcm.handle());

/*
                mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
                mpVocabulary = new ORBVocabulary();  system.cc, created when the system was created. It can be used directly
                mpORBextractorLeft =new  ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
                mCurrentFrame = new Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
                KeyFrame *pKF =  new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
*/
/*
    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;
cout << "1 "<< endl;
    ros::Subscriber sub = nodeHandler.subscribe("/usb_cam/image_raw", 1, &ImageGrabber::GrabImage,&igb);
cout << "2 "<< endl;
   // ros::Subscriber sub2 = nodeHandler.subscribe("chatter", 1000, chatterCallback,ros::TransportHints().tcpNoDelay(true));
*/
/*
      lcm::LCM lcm;
      if(!lcm.good())
	{	
		cout << "lcm is not good "<< endl;
	}
    
      Handler handlerObject;
      lcm.subscribe("sendlcmORBextractor", &Handler::handleMessage, &handlerObject);

      while(0 == lcm.handle());
*/
	


    ros::spin();



    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());
}
///////////////////////////////////////////////////////////////////////////////////////////


/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)), mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";



    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    if(bUseViewer)
    {
        mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);
    }

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    return Tcw;
}

void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    if(mpViewer)
        pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

} //namespace ORB_SLAM
/////////////////////////////////////////////////////////////////////////////////
/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include "Tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"FrameDrawer.h"
#include"Converter.h"
#include"Map.h"
#include"Initializer.h"

#include"Optimizer.h"
#include"PnPsolver.h"

#include<iostream>

#include<mutex>


using namespace std;

namespace ORB_SLAM2
{

Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor):
    mState(NO_IMAGES_YET), mSensor(sensor), mbOnlyTracking(false), mbVO(false), mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)), mpSystem(pSys), mpViewer(NULL),
    mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file

    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    float fps = fSettings["Camera.fps"];
    if(fps==0)
        fps=30;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = fps;

    cout << endl << "Camera Parameters: " << endl;
    cout << "- fx: " << fx << endl;
    cout << "- fy: " << fy << endl;
    cout << "- cx: " << cx << endl;
    cout << "- cy: " << cy << endl;
    cout << "- k1: " << DistCoef.at<float>(0) << endl;
    cout << "- k2: " << DistCoef.at<float>(1) << endl;
    if(DistCoef.rows==5)
        cout << "- k3: " << DistCoef.at<float>(4) << endl;
    cout << "- p1: " << DistCoef.at<float>(2) << endl;
    cout << "- p2: " << DistCoef.at<float>(3) << endl;
    cout << "- fps: " << fps << endl;


    int nRGB = fSettings["Camera.RGB"];
    mbRGB = nRGB;

    if(mbRGB)
        cout << "- color order: RGB (ignored if grayscale)" << endl;
    else
        cout << "- color order: BGR (ignored if grayscale)" << endl;

    // Load ORB parameters

    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::STEREO)
        mpORBextractorRight = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    if(sensor==System::MONOCULAR)
        mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    cout << endl  << "ORB Extractor Parameters: " << endl;
    cout << "- Number of Features: " << nFeatures << endl;
    cout << "- Scale Levels: " << nLevels << endl;
    cout << "- Scale Factor: " << fScaleFactor << endl;
    cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
    cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

    if(sensor==System::STEREO || sensor==System::RGBD)
    {
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
    }

    if(sensor==System::RGBD)
    {
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
            mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }

}

void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

void Tracking::SetViewer(Viewer *pViewer)
{
    mpViewer=pViewer;
}


cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp)
{
    mImGray = imRectLeft;
    cv::Mat imGrayRight = imRectRight;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGR2GRAY);
        }
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
        {
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_RGBA2GRAY);
        }
        else
        {
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
            cvtColor(imGrayRight,imGrayRight,CV_BGRA2GRAY);
        }
    }

    mCurrentFrame = Frame(mImGray,imGrayRight,timestamp,mpORBextractorLeft,mpORBextractorRight,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
{
    mImGray = imRGB;
    cv::Mat imDepth = imD;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
        imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);

    mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

    Track();

    return mCurrentFrame.mTcw.clone();
}


cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double &timestamp)
{
    mImGray = im;

    if(mImGray.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    if(mState==NOT_INITIALIZED || mState==NO_IMAGES_YET)
        mCurrentFrame = Frame(mImGray,timestamp,mpIniORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
    else
        mCurrentFrame = Frame(mImGray,timestamp,mpORBextractorLeft,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);

/*
    lcm::LCM lcm;

      if(!lcm.good())
	{	
		cout << "lcm is not good "<< endl;
	}

      Handler handlerObject;
      lcm.subscribe("sendlcmORBextractor", &Handler::handleMessage, &handlerObject);
      lcm.subscribe("sendlcmFrame", &Handler::handleMessage2, &handlerObject);
      lcm.subscribe("sendlcmMapPointWorldPose", &Handler::handleMessage3, &handlerObject);

     while(0 == lcm.handle());
	
    */  
      

    Track();

    return mCurrentFrame.mTcw.clone();
}

void Tracking::Track()
{
    if(mState==NO_IMAGES_YET)
    {
        mState = NOT_INITIALIZED;
    }

    mLastProcessedState=mState;

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

    if(mState==NOT_INITIALIZED)
    {
        if(mSensor==System::STEREO || mSensor==System::RGBD)
            StereoInitialization();
        else
            MonocularInitialization();

        mpFrameDrawer->Update(this);

        if(mState!=OK)
            return;
    }
    else
    {
        // System is initialized. Track Frame.
        bool bOK;

        // Initial camera pose estimation using motion model or relocalization (if tracking is lost)
        if(!mbOnlyTracking)
        {
            // Local Mapping is activated. This is the normal behaviour, unless
            // you explicitly activate the "only tracking" mode.

            if(mState==OK)
            {
                // Local Mapping might have changed some MapPoints tracked in last frame
                CheckReplacedInLastFrame();

                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame();
                }
                else
                {
                    bOK = TrackWithMotionModel();
                    if(!bOK)
                        bOK = TrackReferenceKeyFrame();
                }
            }
            else
            {
                bOK = Relocalization();
            }
        }
        else
        {
            // Localization Mode: Local Mapping is deactivated

            if(mState==LOST)
            {
                bOK = Relocalization();
            }
            else
            {
                if(!mbVO)
                {
                    // In last frame we tracked enough MapPoints in the map

                    if(!mVelocity.empty())
                    {
                        bOK = TrackWithMotionModel();
                    }
                    else
                    {
                        bOK = TrackReferenceKeyFrame();
                    }
                }
                else
                {
                    // In last frame we tracked mainly "visual odometry" points.

                    // We compute two camera poses, one from motion model and one doing relocalization.
                    // If relocalization is sucessfull we choose that solution, otherwise we retain
                    // the "visual odometry" solution.

                    bool bOKMM = false;
                    bool bOKReloc = false;
                    vector<MapPoint*> vpMPsMM;
                    vector<bool> vbOutMM;
                    cv::Mat TcwMM;
                    if(!mVelocity.empty())
                    {
                        bOKMM = TrackWithMotionModel();
                        vpMPsMM = mCurrentFrame.mvpMapPoints;
                        vbOutMM = mCurrentFrame.mvbOutlier;
                        TcwMM = mCurrentFrame.mTcw.clone();
                    }
                    bOKReloc = Relocalization();

                    if(bOKMM && !bOKReloc)
                    {
                        mCurrentFrame.SetPose(TcwMM);
                        mCurrentFrame.mvpMapPoints = vpMPsMM;
                        mCurrentFrame.mvbOutlier = vbOutMM;

                        if(mbVO)
                        {
                            for(int i =0; i<mCurrentFrame.N; i++)
                            {
                                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                                {
                                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                                }
                            }
                        }
                    }
                    else if(bOKReloc)
                    {
                        mbVO = false;
                    }

                    bOK = bOKReloc || bOKMM;
                }
            }
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(!mbOnlyTracking)
        {
            if(bOK)
                bOK = TrackLocalMap();
        }
        else
        {
            // mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
            // a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
            // the camera we will use the local map again.
            if(bOK && !mbVO)
                bOK = TrackLocalMap();
        }

        if(bOK)
            mState = OK;
        else
            mState=LOST;

        // Update drawer
        mpFrameDrawer->Update(this);

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->Observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    }
            }

            // Delete temporal MapPoints
            for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
            {
                MapPoint* pMP = *lit;
                delete pMP;
            }
            mlpTemporalPoints.clear();

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==LOST)
        {
            if(mpMap->KeyFramesInMap()<=5)
            {
                cout << "Track lost soon after initialisation, reseting..." << endl;
                mpSystem->Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = Frame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
        mlbLost.push_back(mState==LOST);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlFrameTimes.push_back(mlFrameTimes.back());
        mlbLost.push_back(mState==LOST);
    }

}


void Tracking::StereoInitialization()
{
    if(mCurrentFrame.N>500)
    {
        // Set Frame pose to the origin
        mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));

        // Create KeyFrame
        KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

        // Insert KeyFrame in the map
        mpMap->AddKeyFrame(pKFini);

        // Create MapPoints and asscoiate to KeyFrame
        for(int i=0; i<mCurrentFrame.N;i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                pNewMP->AddObservation(pKFini,i);
                pKFini->AddMapPoint(pNewMP,i);
                pNewMP->ComputeDistinctiveDescriptors();
                pNewMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pNewMP);

                mCurrentFrame.mvpMapPoints[i]=pNewMP;
            }
        }

        cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

        mpLocalMapper->InsertKeyFrame(pKFini);

        mLastFrame = Frame(mCurrentFrame);
        mnLastKeyFrameId=mCurrentFrame.mnId;
        mpLastKeyFrame = pKFini;

        mvpLocalKeyFrames.push_back(pKFini);
        mvpLocalMapPoints=mpMap->GetAllMapPoints();
        mpReferenceKF = pKFini;
        mCurrentFrame.mpReferenceKF = pKFini;

        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        mpMap->mvpKeyFrameOrigins.push_back(pKFini);

        mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

        mState=OK;
    }
}

void Tracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference Frame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = Frame(mCurrentFrame);
            mLastFrame = Frame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,1.0,200);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }

        // Find correspondences
        ORBmatcher matcher(0.9,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,100);

        // Check if there are enough correspondences
        if(nmatches<100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set Frame Poses
            mInitialFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            cv::Mat Tcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.SetPose(Tcw);

            CreateInitialMapMonocular();
        }
    }
}

void Tracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap,mpKeyFrameDB);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);


    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);

        pKFini->AddMapPoint(pMP,i);
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);

        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);

        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->AddMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // Bundle Adjustment
    cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth<0 || pKFcur->TrackedMapPoints(1)<100)
    {
        cout << "Wrong initialization, reseting..." << endl;
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.SetPose(pKFcur->GetPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    mvpLocalMapPoints=mpMap->GetAllMapPoints();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = Frame(mCurrentFrame);

    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

    mpMap->mvpKeyFrameOrigins.push_back(pKFini);

    mState=OK;
}

void Tracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        MapPoint* pMP = mLastFrame.mvpMapPoints[i];

        if(pMP)
        {
            MapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}


bool Tracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.7,true);
    vector<MapPoint*> vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.SetPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void Tracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    KeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.SetPose(Tlr*pRef->GetPose());

    if(mnLastKeyFrameId==mLastFrame.mnId || mSensor==System::MONOCULAR || !mbOnlyTracking)
        return;

    // Create "visual odometry" MapPoints
    // We sort points according to their measured depth by the stereo/RGB-D sensor
    vector<pair<float,int> > vDepthIdx;
    vDepthIdx.reserve(mLastFrame.N);
    for(int i=0; i<mLastFrame.N;i++)
    {
        float z = mLastFrame.mvDepth[i];
        if(z>0)
        {
            vDepthIdx.push_back(make_pair(z,i));
        }
    }

    if(vDepthIdx.empty())
        return;

    sort(vDepthIdx.begin(),vDepthIdx.end());

    // We insert all close points (depth<mThDepth)
    // If less than 100 close points, we insert the 100 closest ones.
    int nPoints = 0;
    for(size_t j=0; j<vDepthIdx.size();j++)
    {
        int i = vDepthIdx[j].second;

        bool bCreateNew = false;

        MapPoint* pMP = mLastFrame.mvpMapPoints[i];
        if(!pMP)
            bCreateNew = true;
        else if(pMP->Observations()<1)
        {
            bCreateNew = true;
        }

        if(bCreateNew)
        {
            cv::Mat x3D = mLastFrame.UnprojectStereo(i);
            MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);

            mLastFrame.mvpMapPoints[i]=pNewMP;

            mlpTemporalPoints.push_back(pNewMP);
            nPoints++;
        }
        else
        {
            nPoints++;
        }

        if(vDepthIdx[j].first>mThDepth && nPoints>100)
            break;
    }
}

bool Tracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);

    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

    // Project points seen in previous frame
    int th;
    if(mSensor!=System::STEREO)
        th=15;
    else
        th=7;
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==System::MONOCULAR);

    // If few matches, uses a wider window search
    if(nmatches<20)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,mSensor==System::MONOCULAR);
    }

    if(nmatches<20)
        return false;

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];

                mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                nmatchesMap++;
        }
    }    

    if(mbOnlyTracking)
    {
        mbVO = nmatchesMap<10;
        return nmatches>20;
    }

    return nmatchesMap>=10;
}

bool Tracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                if(!mbOnlyTracking)
                {
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        mnMatchesInliers++;
                }
                else
                    mnMatchesInliers++;
            }
            else if(mSensor==System::STEREO)
                mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);

        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool Tracking::NeedNewKeyFrame()
{
    if(mbOnlyTracking)
        return false;

    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
        return false;

    const int nKFs = mpMap->KeyFramesInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    if(nKFs<=2)
        nMinObs=2;
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;
    if(mSensor!=System::MONOCULAR)
    {
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
            {
                if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                    nTrackedClose++;
                else
                    nNonTrackedClose++;
            }
        }
    }

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    if(mSensor==System::MONOCULAR)
        thRefRatio = 0.9f;

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c =  mSensor!=System::MONOCULAR && (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
            if(mSensor!=System::MONOCULAR)
            {
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
    }
    else
        return false;
}

void Tracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);

    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    if(mSensor!=System::MONOCULAR)
    {
        mCurrentFrame.UpdatePoseMatrices();

        // We sort points by the measured depth by the stereo/RGBD sensor.
        // We create all those MapPoints whose depth < mThDepth.
        // If there are less than 100 close points we create the 100 closest.
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(!vDepthIdx.empty())
        {
            sort(vDepthIdx.begin(),vDepthIdx.end());

            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;

                bool bCreateNew = false;

                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }

                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);

                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                    nPoints++;
                }
                else
                {
                    nPoints++;
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)
                    break;
            }
        }
    }

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}

void Tracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<MapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        MapPoint* pMP = *vit;
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills MapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }

    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        if(mSensor==System::RGBD)
            th=3;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void Tracking::UpdateLocalMap()
{
    // This is for visualization
    mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        KeyFrame* pKF = *itKF;
        const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

        for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            MapPoint* pMP = *itMP;
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}


void Tracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    map<KeyFrame*,int> keyframeCounter;
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());

    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        KeyFrame* pKF = it->first;

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(it->first);
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        if(mvpLocalKeyFrames.size()>80)
            break;

        KeyFrame* pKF = *itKF;

        const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            KeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<KeyFrame*> spChilds = pKF->GetChilds();
        for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            KeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        KeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}

bool Tracking::Relocalization()
{
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<PnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;

    for(int i=0; i<nKFs; i++)
    {
        KeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            PnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                set<MapPoint*> sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void Tracking::Reset()
{

    cout << "System Reseting" << endl;
    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            usleep(3000);
    }

    // Reset Local Mapping
    cout << "Reseting Local Mapper...";
    mpLocalMapper->RequestReset();
    cout << " done" << endl;

    // Reset Loop Closing
    cout << "Reseting Loop Closing...";
    mpLoopClosing->RequestReset();
    cout << " done" << endl;

    // Clear BoW Database
    cout << "Reseting Database...";
    mpKeyFrameDB->clear();
    cout << " done" << endl;

    // Clear Map (this erase MapPoints and KeyFrames)
    mpMap->clear();

    KeyFrame::nNextId = 0;
    Frame::nNextId = 0;
    mState = NO_IMAGES_YET;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlFrameTimes.clear();
    mlbLost.clear();

    if(mpViewer)
        mpViewer->Release();
}

void Tracking::ChangeCalibration(const string &strSettingPath)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(mK);

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = fSettings["Camera.k1"];
    DistCoef.at<float>(1) = fSettings["Camera.k2"];
    DistCoef.at<float>(2) = fSettings["Camera.p1"];
    DistCoef.at<float>(3) = fSettings["Camera.p2"];
    const float k3 = fSettings["Camera.k3"];
    if(k3!=0)
    {
        DistCoef.resize(5);
        DistCoef.at<float>(4) = k3;
    }
    DistCoef.copyTo(mDistCoef);

    mbf = fSettings["Camera.bf"];

    Frame::mbInitialComputations = true;
}

void Tracking::InformOnlyTracking(const bool &flag)
{
    mbOnlyTracking = flag;
}



} //namespace ORB_SLAM

////////////////////////////////////////////////////////////////////////////////////////////////


/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{


MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

}

void MapDrawer::DrawMapPoints()
{

   // mapPointVector;

    if(Receive_mapPointVector.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=Receive_mapPointVector.size(); i<iend;i++)
    {
        glVertex3f(Receive_mapPointVector[i].x,Receive_mapPointVector[i].y,Receive_mapPointVector[i].z);//传输过来的地图点的世界坐标x,y,z存在mapPointVector[]里，在这里画出来
	//glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
/*
    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }
*/
    for(size_t i=0, iend=Receive_RefmapPointVector.size(); i<iend;i++)
    {
        glVertex3f(Receive_RefmapPointVector[i].x,Receive_RefmapPointVector[i].y,Receive_RefmapPointVector[i].z);//传输过来的参考地图点的世界坐标x,y,z存在RefmapPointVector[]里，在这里画出来
	//glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

/*
    const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
        return;

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(0.0,0.0,0.0);

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
            continue;
        cv::Mat pos = vpMPs[i]->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
    }
    glEnd();

    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad())
            continue;
        cv::Mat pos = (*sit)->GetWorldPos();
        glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

    }

    glEnd();
*/
}
/*
float KFSize[3];//w,h,z
std::vector<cv::Mat> Receive_DrawKFVector;//存放传输过来的keyframe的twc（mat矩阵)

std::vector<cv::Point3f> Receive_CovGraphVector_OW;//CovGraph OW
std::vector<cv::Point3f> Receive_CovGraphVector_OW2;//CovGraph OW2

std::vector<cv::Point3f> Receive_SpanTreeVector_OW;
std::vector<cv::Point3f> Receive_SpanTreeVector_OW2;

std::vector<cv::Point3f> Receive_LoopVector_OW;
std::vector<cv::Point3f> Receive_LoopVector_OW2;
*/


void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
   	const float w =  KFSize[0];
    	const float h = w*0.75;
    	const float z = w*0.6;
	
    	if(bDrawKF)
    	{
		///cout << "Receive_DrawKFVector.size()" <<Receive_DrawKFVector.size()<<endl;

        		for(size_t i=0; i<Receive_DrawKFVector.size(); i++)
        		{

            			cv::Mat Twc = Receive_DrawKFVector[i];
			
			//cout << " Twc" << Twc << endl;
           		 	glPushMatrix();

            			glMultMatrixf(Twc.ptr<GLfloat>(0));

            			//cout<< " KF vector size is " <<Receive_DrawKFVector.size() <<endl; 
            			//cout << "i is "  <<  i<< endl;
            			glLineWidth(mKeyFrameLineWidth);
            			
	         		 glColor3f(0.0f,0.0f,1.0f);
	            		glBegin(GL_LINES);
	            		glVertex3f(0,0,0);
	            		glVertex3f(w,h,z);
	            		glVertex3f(0,0,0);
	            		glVertex3f(w,-h,z);
	            		glVertex3f(0,0,0);
	            		glVertex3f(-w,-h,z);
	            		glVertex3f(0,0,0);
	            		glVertex3f(-w,h,z);

            			glVertex3f(w,h,z);
            			glVertex3f(w,-h,z);

            			glVertex3f(-w,h,z);
            			glVertex3f(-w,-h,z);

            			glVertex3f(-w,h,z);
            			glVertex3f(w,h,z);

            			glVertex3f(-w,-h,z);
            			glVertex3f(w,-h,z);
            			//cout << "3"  << endl;
            			glEnd();

            			glPopMatrix();
            			//cout << "4"  << endl;
       	 	}
    	}
    	
    	//每次到传输的时间点之后，就会出现，段错误，然后orbslam2就崩溃退出，把下面这段代码注释之后，就没问题了，需要探索一下为什么
    	//Receive_CovGraphVector_OW.size为0,而且lcmgraph出现的decoding -1，所有是lcm除了问题
	//检查之后，发现是传输的slam里，loop那部分lcm的赋值有问题，因为在检测到loop前，loop那部分set里的值一直为空，无法赋值，导致lcm的变量赋值不完全，所有decoding -1
    	



    	if(bDrawGraph)
    	{
        		glLineWidth(mGraphLineWidth);
        		glColor4f(0.0f,1.0f,0.0f,0.6f);
        		glBegin(GL_LINES);
		//这里是每一个keyframe进一次循环去画图，Receive_DrawKFVector的数量刚好是keyframe的数量，所以这里用的是这个变量
	
        		for(size_t i=0, iend = Receive_CovGraphVector_OWVector.size(); i<iend; i++)//number of keyframe
		{
			for(size_t j =0, jend = Receive_CovGraphVector_OWVector[i].size(); j<jend; j++)
				{
					glVertex3f(Receive_CovGraphVector_OWVector[i][j].x,Receive_CovGraphVector_OWVector[i][j].y,Receive_CovGraphVector_OWVector[i][j].z);
					glVertex3f(Receive_CovGraphVector_OW2Vector[i][j].x,Receive_CovGraphVector_OW2Vector[i][j].y,Receive_CovGraphVector_OW2Vector[i][j].z);
				
					//cout<< "Receive_CovGraphVector_OWVector[i][j].x " <<Receive_CovGraphVector_OWVector[i][j].x <<endl;
					//cout<< "Receive_CovGraphVector_OWVector[i][j].y " <<Receive_CovGraphVector_OWVector[i][j].y <<endl;
					//cout<< "Receive_CovGraphVector_OWVector[i][j].z " <<Receive_CovGraphVector_OWVector[i][j].z <<endl;
				}
		}
        		/*
        		for(size_t i=0, iend = Received_keyFrame.size(); i<iend; i++)//number of keyframe
		{
			for(size_t j=0, jend= Received_keyFrame[i].size(); j<jend; j++)//numer of cov graph in one keyframe
			{
				for(size_t m =0, mend = Received_keyFrame[i][j][0].size(); m<mend; m++)//Received_keyFrame_CovGraph[i][j][0]: vector of ow; Received_keyFrame_CovGraph[i][j][1]: vector of ow2
				{
					glVertex3f(Received_keyFrame[i][j][0][m].x,Received_keyFrame[i][j][0][m].y,Received_keyFrame[i][j][0][m].z);
					glVertex3f(Received_keyFrame[i][j][1][m].x,Received_keyFrame[i][j][1][m].y,Received_keyFrame[i][j][1][m].z);
				}
			}
		}
		*/

		/*
        		for(size_t i=0; i<Receive_DrawKFVector.size(); i++)//Receive_DrawKFVector.size == vpKFs.size, thus, here use it
        		{
            			// Covisibility Graph
        			//cout<<"	Receive_CovGraphVector_OW.size()	"<< Receive_CovGraphVector_OW.size()	<< endl;
        			
        			
        	 		for(size_t i=0, iend=Receive_CovGraphVector_OW.size(); i<iend;i++)
    			{
      				glVertex3f(Receive_CovGraphVector_OW[i].x,Receive_CovGraphVector_OW[i].y,Receive_CovGraphVector_OW[i].z);
      				glVertex3f(Receive_CovGraphVector_OW2[i].x,Receive_CovGraphVector_OW2[i].y,Receive_CovGraphVector_OW2[i].z);
  	 		}
  	 		

        			
        			//for(size_t i=0, iend = Received_keyFrame_CovGraph.size(); i<iend; i++)
        			//{
        			//		glVertex3f(Received_keyFrame_CovGraph[i][0].x,Received_keyFrame_CovGraph[i][0].y,Received_keyFrame_CovGraph[i][0].z);
      			//		glVertex3f(Received_keyFrame_CovGraph[i][1].x,Received_keyFrame_CovGraph[i][1].y,Received_keyFrame_CovGraph[i][1].z);
        			//}
      			
           
  	 		
            			// Spanning tree
  	 		//this is the wrong place
           			
               			glVertex3f(Receive_SpanTreeVector_OW[i].x ,Receive_SpanTreeVector_OW[i].y,Receive_SpanTreeVector_OW[i].z);
               			glVertex3f(Receive_SpanTreeVector_OW2[i].x,Receive_SpanTreeVector_OW2[i].y,Receive_SpanTreeVector_OW2[i].z);
			

            			// Loops
            			//cout<<"	Receive_LoopVector_OW.size()	"<< Receive_LoopVector_OW.size()	<< endl;
        	 		for(size_t i=0, iend=Receive_LoopVector_OW.size(); i<iend;i++)
    			{
      				glVertex3f(Receive_LoopVector_OW[i].x,Receive_LoopVector_OW[i].y,Receive_LoopVector_OW[i].z);
      				glVertex3f(Receive_LoopVector_OW2[i].x,Receive_LoopVector_OW2[i].y,Receive_LoopVector_OW2[i].z);
  			}

  			
        		}
		*/
        		glEnd();
    	}
    	
    	
}

void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

} //namespace ORB_SLAM

