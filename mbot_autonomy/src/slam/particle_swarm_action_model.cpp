#include <slam/particle_swarm_action_model.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/geometric/angle_functions.hpp>
#include <cassert>
#include <cmath>
#include <iostream>
#include <algorithm>


ActionModel::ActionModel(int num_particles, const mbot_lcm_msgs::pose2D_t& initial_pose)
: k1_(0.005f)
, k2_(0.025f)
, min_dist_(0.0025)
, min_theta_(0.02)
, initialized_(false)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    double weight = 1/num_particles;
    
    mbot_lcm_msgs::particle_t p;
    previousPose_ = initial_pose;
    p.parent_pose = initial_pose;
    p.weight = weight;
    for (int i = 0; i<num_particles; i++){
        swarm_.push_back(p);
    }
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
    
    action_time_ = odometry.utime;
    double dx = odometry.x-previousPose_.x;
    double dy = odometry.y-previousPose_.y;
    

    double alpha_ = angle_diff(atan2(dy,dx), previousPose_.theta);
    double beta_ = wrap_to_pi(odometry.theta -(alpha_ + previousPose_.theta));

    double ddist_ = sqrt(pow(dx,2) + pow(dy,2));

    if (ddist_<min_dist_ && alpha_<min_theta_&& beta_<min_theta_){
        return false;
    }

    return true;
}

bool ActionModel::applyAction()
{
      if (!initialized_){
        return false;
    }

    for (auto &particle : swarm_){
        std::normal_distribution<double> e1(0.0, a[0]*abs(alpha_)+a[1]*abs(ddist_));
        double alpha_hat = alpha_ - e1(numberGenerator_);

        std::normal_distribution<double> e2(0.0, a[2]*abs(ddist_)+a[3]*abs(beta_));
        double ddist_hat = alpha_ - e2(numberGenerator_);

        std::normal_distribution<double> e3(0.0, a[0]*abs(beta_)+a[1]*abs(ddist_));
        double beta_hat = beta_ - e3(numberGenerator_);


        double dx = ddist_hat*cos(previousPose_.theta+alpha_hat);
        double dy = ddist_hat*sin(previousPose_.theta+alpha_hat);
        double dtheta = alpha_hat+beta_hat;

        mbot_lcm_msgs::pose2D_t newPose;
        newPose.x = particle.pose.x + dx;
        newPose.y = particle.pose.y + dy;
        newPose.theta = particle.pose.theta + dtheta;
        newPose.utime = action_time_;


        particle.parent_pose = particle.pose;
        particle.pose = newPose;

    }

    double dx = ddist_*cos(previousPose_.theta+alpha_);
    double dy = ddist_*sin(previousPose_.theta+alpha_);
    double dtheta = alpha_+beta_;

    mbot_lcm_msgs::pose2D_t p = previousPose_;
    p.x += dx;
    p.y += dy;
    p.theta += dtheta;
    resetPrevious(p);

    return true;
}
