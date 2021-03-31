#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char *argv[]){
	ros::init(argc,argv,"pubpose_node");
	ros::NodeHandle nh;

	tf::TransformListener listener;
	ros::Publisher posePub = nh.advertise<geometry_msgs::PoseStamped>("tf_pose",1000);
	geometry_msgs::PoseStamped poseMsg;
	ros::Rate rate(10);
	while(nh.ok()){
		tf::StampedTransform transform;
		try{

            listener.lookupTransform("map", "base_link", ros::Time(0), transform);
		}
		catch(tf::TransformException &ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		poseMsg.header.frame_id = "map";
		poseMsg.header.stamp = transform.stamp_;
		poseMsg.pose.position.x = transform.getOrigin().x();
		poseMsg.pose.position.y = transform.getOrigin().y();
		poseMsg.pose.position.z = 0;
		poseMsg.pose.orientation.x  = 0;
		poseMsg.pose.orientation.y  = 0;
		poseMsg.pose.orientation.z  = transform.getRotation().getZ();
		poseMsg.pose.orientation.w  = transform.getRotation().getW();

		ROS_INFO("pub the tf_pose:(%lf,%lf),(0,0,%lf,%lf)",poseMsg.pose.position.x,poseMsg.pose.position.y,poseMsg.pose.orientation.z,poseMsg.pose.orientation.w);
		posePub.publish(poseMsg);
		
		

		rate.sleep();


	}

}