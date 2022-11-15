//#include <ros_planner_dwa/dwa_planner1.h>
#include <dwa_local_planner/dwa_planner_ros.h>

#include "dwa_planner1.h"
#include <base_local_planner/odometry_helper_ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
//#include <tf2/utils.h>
//#include <nav_core/parameter_magic.h>

#include <pluginlib/class_list_macros.h>



//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(my_local_planner::ROSDWAplanner, nav_core::BaseLocalPlanner)

namespace my_local_planner {
    ROSDWAplanner::ROSDWAplanner() : initialized_(false),odom_helper_("odom"){

    }

    void ROSDWAplanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
        // update generic local planner params

        max_vel_trans = 0.26;
        min_vel_trans = 0.13;
        max_vel_x = 0.26;
        min_vel_x = -0.26;
        max_vel_y = 0;
        min_vel_y = 0;
        max_vel_theta = 0.26;
        min_vel_theta = -0.26;
        acc_lim_x = 2.5;
        acc_lim_y = 0.0;
        acc_lim_theta = 3.2;
        acc_lim_trans;
        prune_plan = true ;
        xy_goal_tolerance = 0.1;
        yaw_goal_tolerance = 0.17;
        trans_stopped_vel;
        theta_stopped_vel;

        if (!isInitialized()) {

            ros::NodeHandle private_nh("~/" + name);
            g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
            l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
            tf_ = tf;
            costmap_ros_ = costmap_ros;
            costmap_ros_->getRobotPose(current_pose_);
            global_frame_ = costmap_ros_->getGlobalFrameID();
            // make sure to update the costmap we'll use for this cycle
            costmap = costmap_ros_->getCostmap();
            //costmapSizeX=costmap->getSizeInMetersX();
            //costmapSizeY=costmap->getSizeInMetersY();

            //create the actual planner that we'll use.. it'll configure itself from the parameter server

            if (private_nh.getParam("odom_topic", odom_topic_)) {
                odom_helper_.setOdomTopic(odom_topic_);
            }

            initialized_ = true;
        }
    }
    //Got cantransform error here
    bool ROSDWAplanner::isGoalReached(){
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }

        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }

        if(GoalReached(odom_helper_, current_pose_)) {
            ROS_INFO("Goal reached");
            return true;
        } else {
            return false;
        };
    }

    bool ROSDWAplanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        if (! isInitialized()) {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        ROS_INFO("Got new plan");
//        for(auto it : plan){
//            ROS_INFO("%f,%f",it.pose.position.x,it.pose.position.y);
//        }
        global_plan_.clear();
        for(auto it : plan){
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = it.header.stamp;
            pose.header.frame_id = it.header.frame_id;
            pose.pose.position.x = it.pose.position.x;
            pose.pose.position.y = it.pose.position.y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            global_plan_.push_back(pose);
        }
        //global_plan_ = plan;
        return true;
    }

    bool ROSDWAplanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        // dispatches to either dwa sampling control or stop and rotate control, depending on whether we have been close enough to goal
        if ( ! costmap_ros_->getRobotPose(current_pose_)) {
            ROS_ERROR("Could not get robot pose");
            return false;
        }
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if ( ! getLocalPlan(current_pose_, transformed_plan)) {
            ROS_ERROR("Could not get local plan");
            return false;
        }

        //if the global plan passed in is empty... we won't do anything
        if(transformed_plan.empty()) {
            ROS_WARN_NAMED("dwa_local_planner", "Received an empty transformed plan.");
            return false;
        }
        ROS_DEBUG_NAMED("dwa_local_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

        geometry_msgs::PoseStamped gp;

        if (!getGoalPose(*tf_,
                         *transformed_plan_,
                         global_frame_,
                         gp)) {
            return false;
        }
        //ROS_INFO("goalpose %f,%f",gp.pose.position.x,gp.pose.position.y);
        //Got current_pose_ transformed_plan_(pruned plan less than 1.6*1.6)
        if (!rotate(current_pose_,gp,cmd_vel)) {
            bool isOk = dwaComputeVelocityCommands(transformed_plan, *costmap, current_pose_, cmd_vel);
            if (isOk) {
                publishLocalPlan(transformed_plan);
            } else {
                ROS_WARN_NAMED("dwa_local_planner", "DWA planner failed to produce path.");
                std::vector <geometry_msgs::PoseStamped> empty_plan;
                publishGlobalPlan(empty_plan);
            }
            return isOk;
        }
        else return true;
    }
}



/*  max_vel_x: 0.26
min_vel_x: -0.26

max_vel_y: 0.0
min_vel_y: 0.0

# The velocity when robot is moving in a straight line
max_vel_trans:  0.26
min_vel_trans:  0.13

max_vel_theta: 1.82
min_vel_theta: 0.9

acc_lim_x: 2.5
acc_lim_y: 0.0
acc_lim_theta: 3.2

# Goal Tolerance Parametes
xy_goal_tolerance: 0.05
yaw_goal_tolerance: 0.17
latch_xy_goal_tolerance: false

# Forward Simulation Parameters
sim_time: 2.0
vx_samples: 20
vy_samples: 0
vth_samples: 40
controller_frequency: 10.0

# Trajectory Scoring Parameters
path_distance_bias: 32.0
goal_distance_bias: 20.0
occdist_scale: 0.02
forward_point_distance: 0.325
stop_time_buffer: 0.2
scaling_speed: 0.25
max_scaling_factor: 0.2

# Oscillation Prevention Parameters
oscillation_reset_dist: 0.05

# Debugging
publish_traj_pc : true
publish_cost_grid_pc: true
      base_local_planner::LocalPlannerLimits limits;
      max_vel_trans = max_vel_trans;
      min_vel_trans = min_vel_trans;
      max_vel_x = max_vel_x;
      min_vel_x = min_vel_x;
      max_vel_y = max_vel_y;
      min_vel_y = min_vel_y;
      max_vel_theta = max_vel_theta;
      min_vel_theta = min_vel_theta;
      acc_lim_x = acc_lim_x;
      acc_lim_y = acc_lim_y;
      acc_lim_theta = acc_lim_theta;
      acc_lim_trans = acc_lim_trans;
      xy_goal_tolerance = xy_goal_tolerance;
      yaw_goal_tolerance = yaw_goal_tolerance;
      prune_plan = prune_plan;
      trans_stopped_vel = trans_stopped_vel;
      theta_stopped_vel = theta_stopped_vel;
*/