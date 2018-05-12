#ifndef NEW_H
#define NEW_H

#include<bits/stdc++.h> 
#include<dwa/obstacle_distance.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;



struct state
{
    float x_pos, y_pos, theta, vel, omega;
};

float dis(float x1, float y1, float x2, float y2);
state motion(state temp, float velocity, float omega);
float obstacle_cost(vector<state> trajectory, vector< vector<double> > &distance);
float heading_cost(state current);
float distance_cost(state current, state previous);
float velocity_cost(float vel);
vector< state > calc_trajectory (state init, float velocity, float omega);
vector<float> createDynamicWindow(state currentState);
vector<float> path_find(state current, vector< vector<double> > &distance);
vector<vector<double> > getObsDist();
void update(state current,ros::Publisher vel_pub,vector<vector<double> > distance);

#endif