#include <ros/ros.h>
#include "dwa/New.hpp"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"
#include <math.h>

using namespace std;

state current;

void odomCallBack(const nav_msgs::Odometry::ConstPtr& odom_msg) {
	current.x_pos = odom_msg->pose.pose.position.x;
	current.y_pos = odom_msg->pose.pose.position.y;

	tf::Quaternion q(odom_msg->pose.pose.orientation.x, odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current.theta = yaw;

    double vx = odom_msg->twist.twist.linear.x;
    double vy = odom_msg->twist.twist.linear.y;
    current.vel = sqrt((vx*vx) + (vy*vy));

    current.omega = odom_msg->twist.twist.angular.z;
}

int main(int argc,char **argv) {
	ros::init(argc,argv,"nav_node");
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("odometry/filtered",1000,odomCallBack);
	ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate loop_rate(2);

    vector<vector<double > > distance = getObsDist();

    while(ros::ok()) {
    	update(current,vel_pub,distance);
    	ros::spinOnce();
    	loop_rate.sleep();
    }


	return 0;
}
