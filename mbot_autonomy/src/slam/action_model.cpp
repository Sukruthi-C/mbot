#include <slam/action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(void)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    
    initialized_ = true;
}


void ActionModel::resetPrevious(const mbot_lcm_msgs::pose2D_t& odometry)
{
    if (!initialized_){
        return;
    }
    previousPose_ = odometry;
}


bool ActionModel::updateAction(const mbot_lcm_msgs::pose2D_t& odometry)
{
    if (!initialized_){
        return false;
    }
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    uint64_t t = odometry.utime;
    double dx = odometry.x-previousPose_.x;
    double dy = odometry.y-previousPose_.y;
    

    double alpha = angle_diff(atan2(dy_,dx_), previousPose_.theta);
    double beta = wrap_to_pi(odometry.theta -(alpha + previousPose_.theta));

    double ddist = sqrt(pow(dx,2) + pow(dy,2));

    std::normal_distribution<double> e1(0.0, a[0]*abs(alpha)+a[1]*abs(ddist));
    double alpha_hat = alpha - e1(numberGenerator_);

    std::normal_distribution<double> e2(0.0, a[2]*abs(ddist)+a[3]*abs(beta));
    double ddist_hat = alpha - e2(numberGenerator_);

    std::normal_distribution<double> e3(0.0, a[0]*abs(beta)+a[1]*abs(ddist));
    double beta_hat = beta - e3(numberGenerator_);

    dtheta_ = alpha_hat + beta_hat;
    dx_ = ddist_hat *cos(previousPose_.theta+alpha_hat);
    dy_ = ddist_hat *sin(previousPose_.theta+alpha_hat);

    return true;    // Placeholder
}

mbot_lcm_msgs::particle_t ActionModel::applyAction(const mbot_lcm_msgs::particle_t& sample)
{
      if (!initialized_){
        return sample;
    }
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.
    mbot_lcm_msgs::particle_t newSample;

    newSample.parent_pose = sample.pose;
    mbot_lcm_msgs::pose2D_t* pose = &newSample.pose;
    pose->x = sample.pose.x + dx_;
    pose->y = sample.pose.y + dy_;
    pose->theta = sample.pose.x + dtheta_;
    pose->utime = utime_;
    

    

    return newSample;
}
