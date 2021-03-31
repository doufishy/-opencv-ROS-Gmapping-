#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <kdtree/kdtree.h>
#include <tf2/utils.h>

class AEPlanner{
private:
	ros::NodeHandle nh_;

	// RRTNode* best_node_;
	// RRTNode* best_branch_root_;
	// std::shared_ptr<nav_msgs::OccupancyGrid> map_;
	std::shared_ptr<std::vector<int> > mapVector_;
	int mapHeight_ = 0;
	int mapWidth_ = 0;
	double mapResolution_ = 0;
	double originX_ = 0;
	double originY_ = 0;


	Eigen::Vector3d current_state;
	bool state_initialized;
	bool map_initialized;

	kdtree* kd_tree_;

	ros::Subscriber map_sub_;
	ros::Subscriber pose_sub_;

public:
	AEPlanner(const ros::NodeHandle& nh):
	nh_(nh),
	map_sub_(nh_.subscribe("map",1,&AEPlanner::mapCallback,this)),
	pose_sub_(nh_.subscribe("tf_pose",1,&AEPlanner::poseCallback,this)),
	state_initialized(false),
	map_initialized(false)

	{
		kd_tree_ = kd_create(2);
	}

	void show_pic(){
		cv::Mat mapImage = cv::Mat(mapHeight_, mapWidth_, CV_8UC1, cv::Scalar(125));
 
		for(unsigned int y = 0; y < mapHeight_; y++) {
			for(unsigned int x = 0; x < mapWidth_; x++) {
				unsigned int index = x + (y) * mapWidth_;
				if(mapVector_->at(index) <0 ) 
					continue;

				int red = 255.0 - 255.0* float(mapVector_->at(index))/100.0 ;
				mapImage.at<uchar>(y,x) = red;
			}
		}
		ROS_INFO("current_position::%lf,%lf",current_state[0],current_state[1]);
		// mapImage.at<uchar>(current_state[1],current_state[0]) = 0;
		int indexX = (current_state[0]- originX_) / mapResolution_;
		int indexY = (current_state[1]- originY_) / mapResolution_;
		std::cout << indexX <<";;"<< indexY <<std::endl;
		cv::circle(mapImage, cv::Point(indexX,indexY), 3, cv::Scalar(0), 3);

		cv::imshow("src",mapImage);
		cv::waitKey(1);
	}
	void mapCallback(const nav_msgs::OccupancyGridConstPtr& mapMsg){
		ROS_INFO("heard the map");
		// map_ = std::make_shared<nav_msgs::OccupancyGrid>(mapMsg);
		if(mapResolution_ != mapMsg->info.resolution){
			mapResolution_ = mapMsg->info.resolution;
			ROS_INFO("reset the Resolution::%f",mapResolution_);
		}
		if(mapHeight_ != mapMsg->info.height or mapWidth_ != mapMsg->info.width){
			mapHeight_ = mapMsg->info.height;
			mapWidth_  = mapMsg->info.width;
			ROS_INFO("resize the map::(%d,%d)",mapHeight_,mapWidth_);
			mapVector_ = std::make_shared<std::vector<int> >(std::vector<int>(mapHeight_*mapWidth_,-1));
		}
		if(originX_ != mapMsg->info.origin.position.x or originY_!=mapMsg->info.origin.position.y){
			originX_ = mapMsg->info.origin.position.x;
			originY_ = mapMsg->info.origin.position.y;
			ROS_INFO("reset the Origin coordinates::(%lf,%lf)",originX_,originY_);
		}


		for(unsigned int i = 0;i<mapHeight_*mapWidth_;i++){
			mapVector_->at(i) = mapMsg->data[i];
		}	
		show_pic();
	}
	void poseCallback(const geometry_msgs::PoseStamped& poseMsg){
		ROS_INFO("heard the pose");
		current_state[0] = poseMsg.pose.position.x;
		current_state[1] = poseMsg.pose.position.y;
		current_state[2] = tf2::getYaw(poseMsg.pose.orientation);
		state_initialized = true;
	}
	void execute(){

	}
};

int main(int argc, char * argv[]){
	ros::init(argc, argv, "aeplanner_node");
	ros::NodeHandle n;
	AEPlanner aep(n);
	ros::spin();
	return 1;
}