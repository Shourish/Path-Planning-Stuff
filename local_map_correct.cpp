#include <iostream>
#include <vector>
#include <utility>
#include <cmath>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Int8MultiArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"


#include <opencv2/core/core.hpp>     // opencv libraries
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

const float ResAngLidar = 0.25;                            //lidar angular resolution in degrees
const int  NumLidarRays = 1080;                            //number of angular lidar beams 

int LaneMapW, LaneMapH;                                    //lane map size
int RoadMapW, RoadMapH;                                    //road map size

const float MapRes     = 0.1;                              //map resolution (m/cell)
const float FinMapRes  = 0.1;                              //map resolution (m/cell)
const float ResConv    = FinMapRes/MapRes;                //for converting map to final map

const int MapHeightM   = 40;                               //map height in meters
const int MapWidthM    = 40;                               //map width in meters
const int MapHeight    = MapHeightM/MapRes;                //map height in pixel
const int MapWidth     = MapWidthM/MapRes;                 //map width in pixel
const int FinMapHeight = MapHeightM/FinMapRes;             //final map height in pixel
const int FinMapWidth  = MapWidthM/FinMapRes;              //final map width in pixel
const int MapRangeM    = 10;                               //range for mapping in meters 
const int MapRange     = MapRangeM/MapRes;                 //range for mapping in pixel 

const int LaneSafeRadiiCM = 40;                            //radius around lane for path planning in cm
const int LidarDistCM     = 0;                             //distance of lidar from center of car in cm
const int LidarDist       = LidarDistCM*0.01/MapRes;       //distance of lidar from center of car in cm
const int CamLidarDistCM  = 200;                           //distance between camera and lidar in cm
const int CamDistCM       = LidarDistCM - CamLidarDistCM;  //distance of cam from center of car in cm
const int CamDist         = CamDistCM*0.01/MapRes;         //distance of cam from center of car in pixel
const int LaneShiftCM     = 0;                             //width error in lane in cm
const int LaneShift       = LaneShiftCM*0.01/MapRes;       //width error in lane in pixel

const int CarCenX  = MapWidth/2;                           //center of car with respect to map
const int CarCenY  = 0;                                    //center of car with respect to map

const int ObstCost = 100;                                  //cost of obstacle
const int LaneCost = 80;                                   //cost of lane
const int FinObstCost = 18;                                //final cost of obstacle
const int FinLaneCost = 110;                               //final cost of lane
// ye bakchodi kyu hai mujhe bhi nahi pata

float obs_wgt=0.5,lane_wgt=0.5;

std_msgs::Int8MultiArray LaneMap, RoadMap;
ros::Publisher pub_local_map, pub_cost_map, pub_blown_local_map;

pair<float,float> convToCart(int i,float r) //convert from polar to cartesian co-ordinates
{
	float ang = i*ResAngLidar*(M_PI/180.0);
	float x   =    r*cos(ang);
	float y   = -1*r*sin(ang);
	return make_pair(x,y);
}

void LaneCallback(nav_msgs::OccupancyGrid msg)
{
	if(!msg.data.empty())
	{
		int i,j;
		LaneMapW= msg.info.width;
		LaneMapH = msg.info.height;

		for(i=0; i<LaneMapH; i++)//initialising map
		{
			for(j=0;j<LaneMapW; j++)
			{
				LaneMap.data.push_back(0);
			}
		}

		for(i=LaneMapH-1;i>=0;i--)//putting values
		{
			for(j=0;j<LaneMapW;j++)
			{
				int i1;
				i1=LaneMapH-1-i;
				LaneMap.data[i1*LaneMapW+j] = msg.data[i*LaneMapW+j];
			}
		}
	}
}

void RoadCallback(nav_msgs::OccupancyGrid msg)
{
	if(!msg.data.empty())
	{
		int i,j;
		RoadMapW = msg.info.width;
		RoadMapH = msg.info.height;

		for(i=0; i<RoadMapH; i++)//initialising map
		{
			for(j=0;j<RoadMapW; j++)
			{
				RoadMap.data.push_back(0);
			}
		}

		for(i=RoadMapH-1;i>=0;i--)//putting values
		{
			for(j=0;j<RoadMapW;j++)
			{
				int i1;
				i1=LaneMapH-1-i;
				if(msg.data[i*RoadMapW+j]==2)
				{
					RoadMap.data[i1*RoadMapW+j] = msg.data[i*RoadMapW+j];
				}
			}
		}
	}
}

void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	nav_msgs::OccupancyGrid GridMap;
	GridMap.header.stamp = ros::Time::now();
	GridMap.header.frame_id = "/map";
	GridMap.info.resolution = MapRes;
	GridMap.info.origin.position.x = 0.0;
	GridMap.info.origin.position.y = 0.0;
	GridMap.info.origin.position.z = 0.0;
	GridMap.info.origin.orientation.x = 0.0;
	GridMap.info.origin.orientation.y = 0.0;
	GridMap.info.origin.orientation.z = 0.0;
	GridMap.info.origin.orientation.w = 1.0;
	GridMap.info.width = MapWidth;
	GridMap.info.height = MapHeight;
	GridMap.info.map_load_time = ros::Time::now();

	nav_msgs::OccupancyGrid GCostMap;
	GCostMap.header.stamp = ros::Time::now();
	GCostMap.header.frame_id = "/map";
	GCostMap.info.resolution = MapRes;
	GCostMap.info.origin.position.x = 0.0;
	GCostMap.info.origin.position.y = 0.0;
	GCostMap.info.origin.position.z = 0.0;
	GCostMap.info.origin.orientation.x = 0.0;
	GCostMap.info.origin.orientation.y = 0.0;
	GCostMap.info.origin.orientation.z = 0.0;
	GCostMap.info.origin.orientation.w = 1.0;
	GCostMap.info.width  = MapWidth;
	GCostMap.info.height = MapHeight;
	GCostMap.info.map_load_time = ros::Time::now();

	nav_msgs::OccupancyGrid BlownGridMap;
	BlownGridMap.header.stamp = ros::Time::now();
	BlownGridMap.header.frame_id = "/map";
	BlownGridMap.info.resolution = MapRes;
	BlownGridMap.info.origin.position.x = 0.0;
	BlownGridMap.info.origin.position.y = 0.0;
	BlownGridMap.info.origin.position.z = 0.0;
	BlownGridMap.info.origin.orientation.x = 0.0;
	BlownGridMap.info.origin.orientation.y = 0.0;
	BlownGridMap.info.origin.orientation.z = 0.0;
	BlownGridMap.info.origin.orientation.w = 1.0;
	BlownGridMap.info.width  = MapWidth;
	BlownGridMap.info.height = MapHeight;
	BlownGridMap.info.map_load_time = ros::Time::now();

	nav_msgs::OccupancyGrid FinGridMap;
	FinGridMap.header.stamp = ros::Time::now();
	FinGridMap.header.frame_id = "/map";
	FinGridMap.info.resolution = FinMapRes;
	FinGridMap.info.origin.position.x = 0.0;
	FinGridMap.info.origin.position.y = 0.0;
	FinGridMap.info.origin.position.z = 0.0;
	FinGridMap.info.origin.orientation.x = 0.0;
	FinGridMap.info.origin.orientation.y = 0.0;
	FinGridMap.info.origin.orientation.z = 0.0;
	FinGridMap.info.origin.orientation.w = 1.0;
	FinGridMap.info.width  = FinMapWidth;
	FinGridMap.info.height = FinMapHeight;
	FinGridMap.info.map_load_time = ros::Time::now();

	nav_msgs::OccupancyGrid FinGCostmap;
	FinGCostmap.header.stamp = ros::Time::now();
	FinGCostmap.header.frame_id = "/map";
	FinGCostmap.info.resolution = FinMapRes;
	FinGCostmap.info.origin.position.x = 0.0;
	FinGCostmap.info.origin.position.y = 0.0;
	FinGCostmap.info.origin.position.z = 0.0;
	FinGCostmap.info.origin.orientation.x = 0.0;
	FinGCostmap.info.origin.orientation.y = 0.0;
	FinGCostmap.info.origin.orientation.z = 0.0;
	FinGCostmap.info.origin.orientation.w = 1.0;
	FinGCostmap.info.width  = FinMapWidth;
	FinGCostmap.info.height = FinMapHeight;
	FinGCostmap.info.map_load_time = ros::Time::now();

	vector<pair<float,float>> LidarXY(NumLidarRays,make_pair(0,0));
	for(int i=179;i<NumLidarRays-180;i++)
	{
		{
			//cartesian[i] = convToCart((i-num_values/2),msg->ranges[(i-180)]);
			LidarXY[i] = convToCart((i-NumLidarRays/2),msg->ranges[NumLidarRays - (i)]);
			LidarXY[i].first += LidarDistCM*0.01;  
		}
	}

	for(int i=0;i<MapHeight;i++)
	{
		for(int j=0;j<MapWidth;j++)
		{
			GridMap.data.push_back(0);
			GCostMap.data.push_back(0);
			BlownGridMap.data.push_back(0);
		}
	}
	for(int i=0;i<MapHeight;i++)
	{
		for(int j=0;j<MapWidth;j++)
		{
			FinGridMap.data.push_back(0);
			FinGCostmap.data.push_back(0);
		}
	}

	for(int i=0;i<LaneMapH;i++)//adding lane data 
	{
		for(int j=0;j<LaneMapW;j++)
		{
			if(i >= CarCenY && i <= CarCenY + MapRange)//limiting obstacle detection to only 10m
				if(j - CarCenX <= MapRange && j - CarCenX >= -MapRange)
					if(LaneMap.data[(i)*LaneMapW+j]==2)
						GridMap.data[(i+CamDist)*MapWidth+j+CarCenX-LaneMapW/2-LaneShift] = LaneCost;
		}
	}
	for(int i=0;i<NumLidarRays;i++)
	{
		int Xco = round(LidarXY[i].first/MapRes);
		int Yco = round(LidarXY[i].second/MapRes);
		if(Xco >= CarCenY && Xco <= CarCenY + MapRange)//limiting obstacle detection to only 10m
		{
			if(Yco -CarCenX <= MapRange && Yco - CarCenX >= -MapRange)
			{
				if(!(Xco==CarCenX+LidarDist && Yco==CarCenY))
					GridMap.data[(Xco+LidarDist)*MapWidth+Yco+CarCenX-LaneShift] =  ObstCost;
			} 
		}
	}
	Mat FinLocalMap     = Mat::zeros(FinMapHeight,FinMapWidth,CV_8UC1);
	Mat FinLocalCostMap = Mat::zeros(MapHeight,MapWidth,CV_8UC1);
	Mat LocalMap        = Mat::zeros(MapHeight,MapWidth,CV_8UC1);
	Mat BlownLocalMap   = Mat::zeros(MapHeight,MapWidth,CV_8UC1);
	Mat ObstMat         = Mat::zeros(MapHeight,MapWidth,CV_8UC1); //obstacle matrix
	Mat LaneMat         = Mat::zeros(MapHeight,MapWidth,CV_8UC1); //lane matrix

	for(int i=0;i<MapHeight;i++)
	{
		for(int j=0;j<MapWidth;j++)
		{
			int i_= MapHeight - i;
			LocalMap.at<uchar>(i,j) = GridMap.data[i_*MapWidth+j]*2.55;
			BlownLocalMap.at<uchar>(i,j) = BlownGridMap.data[i_*MapWidth+j]*2.55;
			if (GridMap.data[i_*MapWidth+j] == ObstCost)
				ObstMat.at<uchar>(i,j) = 0;
			else
				ObstMat.at<uchar>(i,j) = 200;
			if(GridMap.data[i_*MapWidth+j] == LaneCost)
				LaneMat.at<uchar>(i,j) = 0;
			else
				LaneMat.at<uchar>(i,j) = 200;
		}
	}
	for(int i=0;i<FinMapHeight;i++)
	{
		for(int j=0;j<FinMapWidth;j++)
		{
			int i_= FinMapHeight - i;
			FinLocalMap.at<uchar>(i,j) = FinGridMap.data[i_*FinMapWidth+j]*0.5*255;
		}
	}

	distanceTransform (ObstMat, ObstMat, CV_DIST_L2, 5);
	distanceTransform (LaneMat, LaneMat, CV_DIST_L2, 5);
	normalize (ObstMat, ObstMat, 0.0, 1.0, NORM_MINMAX);
	normalize (LaneMat, LaneMat, 0.0, 1.0, NORM_MINMAX);
	subtract(1,ObstMat,ObstMat);
	subtract(1,LaneMat,LaneMat);
	lane_wgt = (float)LaneCost/100.0 ; 
	obs_wgt = (float)ObstCost/100.0;
	addWeighted( ObstMat, obs_wgt, LaneMat, lane_wgt, 0.0, FinLocalCostMap);

	for(int i=0;i<MapHeight;i++)
	{
		for(int j=0;j<MapWidth;j++)
		{
			int i_= MapHeight - i;
			GCostMap.data[i_*MapWidth+j] = (FinLocalCostMap.at<float>(i,j))*100;
		}
	}
	for(int i=0;i<MapHeight;i++)//initialising map
	{
		for(int j=0;j<MapWidth;j++)
		{
			if(GridMap.data[i*MapWidth+j]!=0)
				FinGridMap.data[(int)(i/ResConv)*FinMapWidth+(int)(j/ResConv)] = GridMap.data[i*MapWidth+j];
			if(GCostMap.data[i*MapWidth+j]!=0)
				FinGCostmap.data[(int)(i/ResConv)*FinMapWidth+(int)(j/ResConv)] = GCostMap.data[i*MapWidth+j];
		}
	}
	pub_local_map.publish(FinGridMap);
	pub_blown_local_map.publish(GridMap);
	pub_cost_map.publish(FinGCostmap);

	FinGridMap.data.clear();
	FinGCostmap.data.clear();
	LaneMap.data.clear();
	RoadMap.data.clear();
	GridMap.data.clear();
	GCostMap.data.clear();
	BlownGridMap.data.clear();

}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"local_map");

	ros::NodeHandle LRSubNode, LidarSubNode, MapPubNode, BlownPubNode;

	ros::Subscriber SubLane = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid",1,LaneCallback);
	ros::Subscriber SubRoad = LRSubNode.subscribe<nav_msgs::OccupancyGrid>("/Road_Occupancy_Grid",1,RoadCallback);

	ros::Subscriber SubLidar = LidarSubNode.subscribe<sensor_msgs::LaserScan>("scan",1,LidarCallback);

	pub_local_map = MapPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/local_map",1);
	pub_cost_map  = MapPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/cost_map",1);

	pub_blown_local_map = BlownPubNode.advertise<nav_msgs::OccupancyGrid>("/scan/blown_local_map",1);

	/*namedWindow("final_local_map",CV_WINDOW_NORMAL);   
	namedWindow("final_local_map_cost",CV_WINDOW_NORMAL); 
	namedWindow("local_map",CV_WINDOW_NORMAL);
	namedWindow("blown_local_map",CV_WINDOW_NORMAL);
	namedWindow("mapImg obstacle",CV_WINDOW_NORMAL);
	namedWindow("mapImg lane",CV_WINDOW_NORMAL);*/

	/*createTrackbar("lane_c", "final_local_map_cost", &w2, 100);
	createTrackbar("obst_o", "final_local_map_cost", &w1, 100);*/

	ros::Rate RosLoopRate(15.0);
	while(ros::ok())
	{
		ros::spinOnce();//check for incoming messages
		RosLoopRate.sleep(); 
	}
	return 0; 
}


