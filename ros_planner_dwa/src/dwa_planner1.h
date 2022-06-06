#ifndef MY_LOCAL_PLANNER_ROS_PLANNER1_H_
#define MY_LOCAL_PLANNER_ROS_PLANNER1_H_
#include <move_base/move_base.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
//#include <base_local_planner/trajectory.h>
#include <tf2_ros/buffer.h>
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <nav_core/parameter_magic.h>
#include <angles/angles.h>

#include <nav_msgs/Odometry.h>
#include <cmath>
/*
#include <dwa_local_planner/dwa_planner.h>
#include <cmath>

//for computing path distance
#include <queue>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>*/

namespace my_local_planner{
    class ROSDWAplanner : public nav_core::BaseLocalPlanner{
    private:
        double max_vel_trans;
        double min_vel_trans;
        double max_vel_x;
        double min_vel_x;
        double max_vel_y;
        double min_vel_y;
        double max_vel_theta;
        double min_vel_theta;
        double acc_lim_x;
        double acc_lim_y;
        double acc_lim_theta;
        double acc_lim_trans;
        bool prune_plan;
        double xy_goal_tolerance;
        double yaw_goal_tolerance;
        double trans_stopped_vel;
        double theta_stopped_vel;
        bool restore_defaults;
        tf2_ros::Buffer* tf_; ///< @brief Used for transforming point clouds

        bool initialized_;

        costmap_2d::Costmap2DROS* costmap_ros_;

        costmap_2d::Costmap2D* costmap;

        geometry_msgs::PoseStamped current_pose_;

        std::vector<geometry_msgs::PoseStamped> global_plan_;

        struct Return {
            double rx=0;
            double ry=0;
            double cx=0;
            double cy=0;
            double d(){
                return hypot(rx-cx/2,ry-cy/2);
            }
            bool exist;
        };

        struct Cost{
            double vx;
            double vy;
            double w;
            double normv;
            double normw;
            double cost;
        };
    public:
        ROSDWAplanner();
        ROSDWAplanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

        bool isGoalReached();

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        //bool GoalReached(OdometryHelperRos& odom_helper,
                           //const geometry_msgs::PoseStamped& global_pose)

        // for visualisation, publishers of global and local plan
        ros::Publisher g_plan_pub_, l_plan_pub_;

        //boost::shared_ptr<DWAPlanner> dp_; ///< @brief The trajectory controller


        base_local_planner::OdometryHelperRos odom_helper_;

        std::string odom_topic_;
        std::string global_frame_;

        bool isInitialized() {
            return initialized_;
        }

        bool getGoalPose(const tf2_ros::Buffer& tf,
                         const std::vector<geometry_msgs::PoseStamped>& global_plan,
                         const std::string& global_frame, geometry_msgs::PoseStamped &goal_pose) {
            //we assume the global goal is the last point in the global plan
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                return false;
            }

            const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.back();
            try{
                geometry_msgs::TransformStamped transform = tf.lookupTransform(global_frame, ros::Time(),
                                                                               plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                                                                               plan_goal_pose.header.frame_id, ros::Duration(0.5));
                //point in source frame do what transformation into point in target frame
                tf2::doTransform(plan_goal_pose, goal_pose, transform);
                //goal_pose with frame_id of global_frame
            }
            catch(tf2::LookupException& ex) {
                ROS_ERROR("No Transform available Error: %s\n", ex.what());
                return false;
            }
            catch(tf2::ConnectivityException& ex) {
                ROS_ERROR("Connectivity Error: %s\n", ex.what());
                return false;
            }
            catch(tf2::ExtrapolationException& ex) {
                ROS_ERROR("Extrapolation Error: %s\n", ex.what());
                if (global_plan.size() > 0)
                    ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

                return false;
            }
            return true;
        }

        bool getGoalPoseLocal(const tf2_ros::Buffer& tf,
                         const std::vector<geometry_msgs::PoseStamped>& global_plan,
                              costmap_2d::Costmap2DROS* costmap_ros, geometry_msgs::PoseStamped &goal_pose) {
            //we assume the global goal is the last point in the global plan
            if (global_plan.empty())
            {
                ROS_ERROR("Received plan with zero length");
                return false;
            }
            std::string base_frame=costmap_ros->getBaseFrameID();
            const geometry_msgs::PoseStamped& plan_goal_pose = global_plan.back();
            try{
                geometry_msgs::TransformStamped transform = tf.lookupTransform(base_frame, ros::Time(),
                                                                               plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                                                                               plan_goal_pose.header.frame_id, ros::Duration(0.5));
                //point in source frame do what transformation into point in target frame
                tf2::doTransform(plan_goal_pose, goal_pose, transform);
                //goal_pose with frame_id of global_frame
            }
            catch(tf2::LookupException& ex) {
                ROS_ERROR("No Transform available Error: %s\n", ex.what());
                return false;
            }
            catch(tf2::ConnectivityException& ex) {
                ROS_ERROR("Connectivity Error: %s\n", ex.what());
                return false;
            }
            catch(tf2::ExtrapolationException& ex) {
                ROS_ERROR("Extrapolation Error: %s\n", ex.what());
                if (global_plan.size() > 0)
                    ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", "base_link", (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

                return false;
            }
            return true;
        }

        bool GoalReached(base_local_planner::OdometryHelperRos& odom_helper,
                                         const geometry_msgs::PoseStamped& global_pose) {

            //copy over the odometry information
            nav_msgs::Odometry base_odom;
            odom_helper.getOdom(base_odom);

            //we assume the global goal is the last point in the global plan
            geometry_msgs::PoseStamped goal_pose;
            if (!getGoalPose(*tf_,
                             global_plan_,
                             global_frame_,
                             goal_pose)) {
                return false;
            }

            double goal_x = goal_pose.pose.position.x;
            double goal_y = goal_pose.pose.position.y;
            //ROS_INFO("Check Goal Distance %f", getGoalPositionDistance(global_pose, goal_x, goal_y));
            //base_local_planner::LocalPlannerLimits limits = planner_util->getCurrentLimits();

            //check to see if we've reached the goal position
            if (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance) {
                double goal_th = tf2::getYaw(goal_pose.pose.orientation);
                //double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
                //check to see if the goal orientation has been reached
                //if (fabs(angle) <= yaw_goal_tolerance) {
                    //make sure that we're actually stopped before returning success
                    //if (stopped(base_odom, theta_stopped_vel, trans_stopped_vel)) {
                        return true;
                    //}
                //}
            }
            return false;
        }

        void prunePlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& plan, std::vector<geometry_msgs::PoseStamped>& global_plan){
            ROS_ASSERT(global_plan.size() >= plan.size())
            std::vector<geometry_msgs::PoseStamped>::iterator it = plan.end()-1;
            std::vector<geometry_msgs::PoseStamped>::iterator global_it = global_plan.end()-1;
            while(it != plan.begin()){
                const geometry_msgs::PoseStamped& w = *it;
                // Fixed error bound of 2 meters for now. Can reduce to a portion of the map size or based on the resolution
                double x_diff = global_pose.pose.position.x - w.pose.position.x;
                double y_diff = global_pose.pose.position.y - w.pose.position.y;
                double distance_sq = x_diff * x_diff + y_diff * y_diff;
                if(distance_sq < 0.8){
                    ROS_DEBUG("Nearest waypoint to <%f, %f> is <%f, %f>\n", global_pose.pose.position.x, global_pose.pose.position.y, w.pose.position.x, w.pose.position.y);
                    break;
                }
                it = plan.erase(it);
                global_it = global_plan.erase(global_it);
                --it;
                //beginning element?
            }
        }

        double getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y) {
            return hypot(goal_x - global_pose.pose.position.x, goal_y - global_pose.pose.position.y);
        }

        double getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double goal_th) {
            double yaw = tf2::getYaw(global_pose.pose.orientation);
            return angles::shortest_angular_distance(yaw, goal_th);
        }

        bool stopped(const nav_msgs::Odometry& base_odom,
                     const double& rot_stopped_velocity, const double& trans_stopped_velocity){
            return fabs(base_odom.twist.twist.angular.z) <= rot_stopped_velocity
                   && fabs(base_odom.twist.twist.linear.x) <= trans_stopped_velocity
                   && fabs(base_odom.twist.twist.linear.y) <= trans_stopped_velocity;
        }

        bool getLocalPlan(const geometry_msgs::PoseStamped& global_pose, std::vector<geometry_msgs::PoseStamped>& transformed_plan) {
            //get the global plan in our frame
            if(!transformGlobalPlan(
                    *tf_,
                    global_plan_,
                    global_pose,
                    *costmap,
                    global_frame_,
                    transformed_plan)) {
                ROS_WARN("Could not transform the global plan to the frame of the controller");
                return false;
            }

            //now we'll prune the plan based on the position of the robot
            if(prune_plan) {
                prunePlan(global_pose, transformed_plan, global_plan_);
                publishLocalPlan(transformed_plan);
                //ROS_INFO("Pruned");
            }
            return true;
        }

        bool transformGlobalPlan(
                const tf2_ros::Buffer& tf,
                const std::vector<geometry_msgs::PoseStamped>& global_plan,
                const geometry_msgs::PoseStamped& global_pose,
                const costmap_2d::Costmap2D& costmap,
                const std::string& global_frame,
                std::vector<geometry_msgs::PoseStamped>& transformed_plan){
            transformed_plan.clear();

            if (global_plan.empty()) {
                ROS_ERROR("Received plan with zero length");
                return false;
            }

            const geometry_msgs::PoseStamped& plan_pose = global_plan[0];
            try {
                // get plan_to_global_transform from plan frame to global_frame
                geometry_msgs::TransformStamped plan_to_global_transform = tf.lookupTransform(global_frame, ros::Time(),
                                                                                              plan_pose.header.frame_id, plan_pose.header.stamp, plan_pose.header.frame_id, ros::Duration(0.5));

                //let's get the pose of the robot in the frame of the plan
                geometry_msgs::PoseStamped robot_pose;
                tf.transform(global_pose, robot_pose, plan_pose.header.frame_id);
                //why not costmap_->current_pose_?
                //we'll discard points on the plan that are outside the local costmap
                double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                                 costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);

                unsigned int i = 0;
                double sq_dist_threshold = dist_threshold * dist_threshold;
                double sq_dist = 0;

                //we need to loop to a point on the plan that is within a certain distance of the robot
                while(i < (unsigned int)global_plan.size()) {
                    double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                    double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                    sq_dist = x_diff * x_diff + y_diff * y_diff;
                    if (sq_dist <= sq_dist_threshold) {
                        break;
                    }
                    ++i;
                }

                geometry_msgs::PoseStamped newer_pose;

                //now we'll transform until points are outside of our distance threshold
                while(i < (unsigned int)global_plan.size() && sq_dist <= sq_dist_threshold) {
                    const geometry_msgs::PoseStamped& pose = global_plan[i];
                    tf2::doTransform(pose, newer_pose, plan_to_global_transform);

                    transformed_plan.push_back(newer_pose);

                    double x_diff = robot_pose.pose.position.x - global_plan[i].pose.position.x;
                    double y_diff = robot_pose.pose.position.y - global_plan[i].pose.position.y;
                    sq_dist = x_diff * x_diff + y_diff * y_diff;

                    ++i;
                }
            }
            catch(tf2::LookupException& ex) {
                ROS_ERROR("No Transform available Error: %s\n", ex.what());
                return false;
            }
            catch(tf2::ConnectivityException& ex) {
                ROS_ERROR("Connectivity Error: %s\n", ex.what());
                return false;
            }
            catch(tf2::ExtrapolationException& ex) {
                ROS_ERROR("Extrapolation Error: %s\n", ex.what());
                if (!global_plan.empty())
                    ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

                return false;
            }
            ROS_INFO("transforming GlobalPlan");
            return true;
        }
        bool dwaComputeVelocityCommands(std::vector<geometry_msgs::PoseStamped>& transformed_plan, const costmap_2d::Costmap2D& costmap, geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist& cmd_vel) {
            // dynamic window sampling approach to get useful velocity commands
            if (!isInitialized()) {
                ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
                return false;
            }

            geometry_msgs::PoseStamped robot_vel;
            odom_helper_.getRobotVel(robot_vel);
            //compute what trajectory to drive along
            geometry_msgs::PoseStamped drive_cmds;
            drive_cmds.header.frame_id = costmap_ros_->getBaseFrameID();

            //Got current_pose_ transformed_plan_(pruned plan less than 2*2)
            //Next cumpute drive_cmds via robot_vel(now), global_pose
            double v_interval;
            double w_interval;
            std::vector <geometry_msgs::PoseStamped> window(11);
            double xvel= robot_vel.pose.position.x;//sqrt(robot_vel.pose.position.x*robot_vel.pose.position.x+robot_vel.pose.position.y*robot_vel.pose.position.y);
            if (fabs(xvel)<0.13){
                v_interval =
                        (fabs(max_vel_x - xvel) < fabs(min_vel_x - xvel) ? fabs(
                                max_vel_x - xvel) : fabs(min_vel_x - xvel)) / 5;
                //geometry_msgs::TransformStamped transform;
                //initialize velocity and angularity
                for (int i = 0; i < window.size(); i++) {
                    window[i].pose.position.x =robot_vel.pose.position.x + i * v_interval - 5 * v_interval;
                }
            }else if(xvel>0.13){
                v_interval = 0.026;
                //geometry_msgs::TransformStamped transform;
                //initialize velocity and angularity
                for (int i = 0; i < window.size(); i++) {
                    window[i].pose.position.x =0.13 + i * v_interval - 5 * v_interval;
                }
            }else{
                v_interval = 0.026;
                //geometry_msgs::TransformStamped transform;
                //initialize velocity and angularity
                for (int i = 0; i < window.size(); i++) {
                    window[i].pose.position.x =0.13 + i * v_interval - 5 * v_interval;
                }
            }

            if (fabs(tf2::getYaw(robot_vel.pose.orientation))<0.13){
                w_interval =
                        (fabs(max_vel_theta - tf2::getYaw(robot_vel.pose.orientation)) < fabs(min_vel_theta - tf2::getYaw(robot_vel.pose.orientation))
                         ? fabs(max_vel_theta - tf2::getYaw(robot_vel.pose.orientation)) : fabs(
                                        min_vel_theta - tf2::getYaw(robot_vel.pose.orientation))) / 5;
                //geometry_msgs::TransformStamped transform;
                //initialize velocity and angularity
                for (int i = 0; i < window.size(); i++) {
                    tf2::Quaternion q;
                    q.setRPY(0, 0, tf2::getYaw(robot_vel.pose.orientation) + i * w_interval - 5 * w_interval);
                    tf2::convert(q, window[i].pose.orientation);
                }
            }else if(tf2::getYaw(robot_vel.pose.orientation)>0.13){
                w_interval = 0.026;
                //geometry_msgs::TransformStamped transform;
                //initialize velocity and angularity
                for (int i = 0; i < window.size(); i++) {
                    tf2::Quaternion q;
                    q.setRPY(0, 0, 0.13+ i * w_interval - 5 * w_interval);
                    tf2::convert(q, window[i].pose.orientation);
                }
            }else{
                w_interval = 0.026;
                //geometry_msgs::TransformStamped transform;
                //initialize velocity and angularity
                for (int i = 0; i < window.size(); i++) {
                    tf2::Quaternion q;
                    q.setRPY(0, 0, -0.13+ i * w_interval - 5 * w_interval);
                    tf2::convert(q, window[i].pose.orientation);
                }
            }

            for (int i = 0; i < 11; ++i) {
                ROS_INFO("%f,%f",window[i].pose.position.x,tf2::getYaw(window[i].pose.orientation));
            }
            //ROS_INFO("%f", );
            double dt = 0.01;
            std::vector<double>heading(121);
            std::vector<double>breakdistance(121);
            std::vector<double>angle(121);
            std::vector <std::vector<geometry_msgs::PoseStamped>> trajectory(121,
                                                                             std::vector<geometry_msgs::PoseStamped>(
                                                                                     11));
            std::vector<double>dmin(121,10);
            //generate trajectory and trajectory point
            //velocity iteration 11 ROW
            for (int i = 0; i < window.size(); i++) {
                for (int o = 0; o < window.size(); o++) {
                    breakdistance[i*11+o] = window[i].pose.position.x * window[i].pose.position.x/(2*acc_lim_x);
                    //Initialize homogenous matrix T, every T right multiplication stands for a dt
                    Eigen::AngleAxisd rotation_vector(tf2::getYaw(window[o].pose.orientation) * dt, Eigen::Vector3d(0, 0, 1));
                    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
                    T.rotate(rotation_vector);
                    T.pretranslate(Eigen::Vector3d(window[i].pose.position.x * dt, 0, 0));
                    Eigen::Isometry3d tmp = Eigen::Isometry3d::Identity();

                    //j is trajectory point. iteration are 11 times
                    for (int j = 0; j < trajectory[0].size(); j++) {
                        trajectory[i * 11 + o][j].pose.position.x = tmp.translation()[0];
                        trajectory[i * 11 + o][j].pose.position.y = tmp.translation()[1];

                        Eigen::Matrix3d matrix = tmp.rotation();
                        Eigen::Vector3d vector = matrix.eulerAngles(2, 1, 0);
                        tf2::Quaternion q;
                        //??
                        q.setRPY(0, 0, vector[0]);
                        tf2::convert(q, trajectory[i * 11 + o][j].pose.orientation);
                        //When acquiring the last trajectory point, get the heading angle
                        if(j==10){
                            //heading[i * 11 + o]=getGoalOrientationAngleDifference(trajectory[i * 11 + o][j], tf2::getYaw(transformed_plan.back().pose.orientation));
                            geometry_msgs::PoseStamped goal_pose;
                            if (!getGoalPose(*tf_,
                                             global_plan_,
                                             global_frame_,
                                             goal_pose)) {
                                return false;
                            }

                            double goal_x = goal_pose.pose.position.x;
                            double goal_y = goal_pose.pose.position.y;
                            Eigen::Quaterniond q(current_pose_.pose.orientation.w,current_pose_.pose.orientation.x,current_pose_.pose.orientation.y,current_pose_.pose.orientation.z);
                            Eigen::Matrix3d mtr = q.toRotationMatrix();
                            Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
                            T1.rotate(mtr);
                            T1.pretranslate(Eigen::Vector3d(current_pose_.pose.position.x,current_pose_.pose.position.y , current_pose_.pose.position.z));
                            T1 = T1 * tmp;
                            Eigen::Vector3d vec = T1.rotation().eulerAngles(2,1,0);
                            tf2::Quaternion qq;
                            qq.setRPY(0,0,vec[0]);
                            geometry_msgs::PoseStamped endpoint;
                            tf2::convert(qq,endpoint.pose.orientation);
                            double yaw = tf2::getYaw(global_pose.pose.orientation);
                            double bian = hypot(goal_pose.pose.position.x-global_pose.pose.position.x,goal_pose.pose.position.y-global_pose.pose.position.y);
                            double goal_th;
                            if(goal_pose.pose.position.y-global_pose.pose.position.y<0){
                                goal_th = -acos((goal_pose.pose.position.x-global_pose.pose.position.x)/bian);
                            }else
                                goal_th = acos((goal_pose.pose.position.x-global_pose.pose.position.x)/bian);
                            angle[i*11+o]= fabs(angles::shortest_angular_distance(yaw, goal_th));
                            heading[i * 11 + o] = hypot(T1.translation()[0]-goal_x,T1.translation()[1]-goal_y);
                            ROS_INFO("%d,%f",i * 11 + o,heading[i * 11 + o]);
                            //ROS_INFO("%f", getGoalPositionDistance(current_pose_,goal_x,goal_y));
                            //ROS_INFO("%f",fabs(getGoalPositionDistance(current_pose_,goal_x,goal_y)-heading[i * 11 + o]));
                        }
                        //every time frame move along the local frame, so right multiply
                        tmp = tmp * T;


                        //Till now, we have got a point on a full trajectory. Next compute the distance between point and nearest obstacle.
                        //Here I choose to evaluate 8 points around.
                        unsigned int Xcord = static_cast<int>((tmp.translation()[0] + costmap.getSizeInMetersX() / 2) /
                                                              costmap.getResolution());
                        unsigned int Ycord = static_cast<int>((tmp.translation()[1] + costmap.getSizeInMetersY() / 2) /
                                                              costmap.getResolution());
                        for (int u = 0; u + Xcord < costmap.getSizeInCellsX() && Xcord - u > 0 &&
                                    u + Ycord < costmap.getSizeInCellsY() && Ycord - u > 0; u++) {
                            unsigned int Xcordf = Xcord + u;
                            unsigned int Xcordb = Xcord - u;
                            unsigned int Ycordf = Ycord + u;
                            unsigned int Ycordb = Ycord - u;
                            Return returnd[8];
                            returnd[0] = getObstacleDistance(Xcord, Ycordf, costmap);
                            returnd[1] = getObstacleDistance(Xcord, Ycordb, costmap);
                            returnd[2] = getObstacleDistance(Xcordf, Ycordf, costmap);
                            returnd[3] = getObstacleDistance(Xcordf, Ycord, costmap);
                            returnd[4] = getObstacleDistance(Xcordf, Ycordb, costmap);
                            returnd[5] = getObstacleDistance(Xcordb, Ycordf, costmap);
                            returnd[6] = getObstacleDistance(Xcordb, Ycord, costmap);
                            returnd[7] = getObstacleDistance(Xcordb, Ycordb, costmap);
                            for(int p=0; p<7; p++){
                                if(returnd[p].exist){
                                    dmin[i * 11 + o] = dmin[i * 11 + o] < returnd[p].d() ? dmin[i * 11 + o] : returnd[p].d();
                                    if(dmin[i * 11 + o]<breakdistance[i*11+o]){
                                        dmin[i * 11 + o]=-100;
                                    }
                                    break;
                                }
                                else if(p==6)
                                    dmin[i * 11 + o] = costmap.getSizeInMetersX();
                                else
                                    continue;
                            }
                            //Now we get minimum obstacle distance dmin, next we compute heading

                        }
                    }
                    /*ROS_INFO("%d,The least distance is %f",i * 11 + o,dmin[i * 11 + o]);*/}
            }
            std::vector<Cost> cost = costFunction(heading, dmin, window, angle, global_pose);
            Cost mincost = getMinCost(cost);
            ROS_INFO("vx = %f; vy = %f; w = %f", mincost.vx, mincost.vy, mincost.w);
            odom_helper_.getRobotVel(robot_vel);
            ROS_INFO("robot_vel is %f %f %f", robot_vel.pose.position.x, robot_vel.pose.position.y, tf2::getYaw(robot_vel.pose.orientation));
            cmd_vel.linear.x = mincost.vx;
            cmd_vel.linear.y = mincost.vy;
            cmd_vel.angular.z = mincost.w;
            /*cmd_vel.linear.x = 1;
            cmd_vel.linear.y = 1;
            cmd_vel.angular.z = 1;*/
            return true;
        }

        Cost getMinCost(std::vector<Cost>& cost){
            double mincost=10000;
            double &ref= mincost;
            int num;
            for(int i=0; i<121; i++){
                if(cost[i].cost < ref){
                    ref= cost[i].cost;
                    num = i;
                }
            }
            //ROS_INFO("Least Cost is %f", cost[num].cost);
            return cost[num];
        }

        Return getObstacleDistance(unsigned int x, unsigned int y, const costmap_2d::Costmap2D& costmap){
            Return ret;
            if(costmap.getCost(x, y)>130){
                ret.rx=x*costmap.getResolution();
                ret.ry=y*costmap.getResolution();
                ret.cx=costmap.getSizeInMetersX();
                ret.cy=costmap.getSizeInMetersY();
                ret.exist=true;
                return ret;
            }
            else{
                ret.rx=x;
                ret.ry=y;
                ret.exist=false;
                return ret;
            }
        }

        std::vector<Cost> costFunction(std::vector<double>& heading, std::vector<double>& dmin, std::vector<geometry_msgs::PoseStamped>& window, std::vector<double>& angle, geometry_msgs::PoseStamped &global_pose){
            std::vector<Cost>cost;
            cost.resize(121);
            //std::vector<geometry_msgs::PoseStamped>backup = window;
            //accumulate
            double alpha = 50;
            double beta = 0.2;
            double gamma1 = 0.2;
            double theta = 1;
            double angleS = 0;
            double headingS=0;
            double dminS=0;
            double velocityS=0;
            for (int i=0; i<11; i++){
                velocityS = velocityS + 11 * fabs(window[i].pose.position.x-0.22);
                //wS= wS+11 * fabs(tf2::getYaw(window[i].pose.orientation)-0.16);
            }

            for (int i=0; i<121; i++){
                headingS = headingS + heading[i];
                dminS = dminS + dmin[i];
                angleS = angleS + angle[i];
            }
            //normalize
            for (int i=0; i<121; i++){
                heading[i] = heading[i]/headingS;
                dmin[i] = dmin[i]/dminS;
                angle[i] = angle[i]/angleS;
                cost[i].cost=alpha*heading[i]-beta*dmin[i]+theta*angle[i];
            }

            for(int i=0; i<11; i++){
                for(int j=0; j<11; j++){
                    cost[i*11+j].vx= window[i].pose.position.x;
                    cost[i*11+j].vy= 0;
                    cost[i*11+j].w = tf2::getYaw(window[j].pose.orientation);
                    cost[i*11+j].normv = (cost[i*11+j].vx-0.22)/velocityS;
                    //cost[i*11+j].normw = (cost[i*11+j].w-0.16)/wS;
                    cost[i*11+j].cost=cost[i*11+j].cost+gamma1*cost[i*11+j].normv;//+gamma2*cost[i*11+j].normw;
                    //ROS_INFO("%f",cost[i*11+j].vx);
                }
            }

            return cost;
        }

        void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
            publishPlan(path, l_plan_pub_);
        }

        void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path) {
            publishPlan(path, g_plan_pub_);
        }

        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub) {
            //given an empty path we won't do anything
            if(path.empty())
                return;

            //create a path message
            nav_msgs::Path gui_path;
            gui_path.poses.resize(path.size());
            gui_path.header.frame_id = path[0].header.frame_id;
            gui_path.header.stamp = path[0].header.stamp;

            // Extract the plan in world co-ordinates, we assume the path is all in the same frame
            for(unsigned int i=0; i < path.size(); i++){
                gui_path.poses[i] = path[i];
            }

            pub.publish(gui_path);
        }

        bool rotate(geometry_msgs::PoseStamped& global_pose, geometry_msgs::PoseStamped& goal_pose, geometry_msgs::Twist& cmd_vel){
            double yaw = tf2::getYaw(global_pose.pose.orientation);
            double bian = hypot(goal_pose.pose.position.x-global_pose.pose.position.x,goal_pose.pose.position.y-global_pose.pose.position.y);
            double goal_th;
            if(goal_pose.pose.position.y-global_pose.pose.position.y<0){
                goal_th = -acos((goal_pose.pose.position.x-global_pose.pose.position.x)/bian);
            }else
                goal_th = acos((goal_pose.pose.position.x-global_pose.pose.position.x)/bian);
            double angle = angles::shortest_angular_distance(yaw, goal_th);
            //ROS_INFO("yaw=%f,goal_th=%f,angledif=%f",yaw, goal_th, angle );
            if(angle > (3.14/8)) {
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = 0.26;
                ROS_INFO("rotating conterclockwise");
                return true;
            }else if(angle < (-3.14/8)){
                cmd_vel.linear.x = 0.05;
                cmd_vel.angular.z = -0.26;
                ROS_INFO("rotating clockwise");
                return true;
            }else{
                return false;
            }
        }
    };
}

#endif