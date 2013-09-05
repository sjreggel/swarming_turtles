/*
 * Copyright (c) 2012, Daniel Claes, Maastricht University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Maastricht University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <angles/angles.h>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>


#include "swarming_turtles_navigation/ROSAgent.h"

#include "swarming_turtles_navigation/orca.h"
#include "swarming_turtles_navigation/collvoid_publishers.h"


template <typename T>
void getParam (const ros::NodeHandle nh, const std::string& name, T* place)
{
  bool found = nh.getParam(name, *place);
  ROS_ASSERT_MSG (found, "Could not find parameter %s", name.c_str());
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << *place);
}


template <class T>
T getParamDef (const ros::NodeHandle nh, const std::string& name, const T& default_val)
{
  T val;
  nh.param(name, val, default_val);
  ROS_DEBUG_STREAM_NAMED ("init", "Initialized " << name << " to " << val <<
                          "(default was " << default_val << ")");
  return val;
}

using namespace collvoid;

namespace collvoid{

  ROSAgent::ROSAgent()  {
    initialized_ = false;
    cur_allowed_error_ = 0;
    cur_loc_unc_radius_ = 0;
    min_dist_obst_ = DBL_MAX;
    agents_.positions.clear();
  }
  
  void ROSAgent::init(ros::NodeHandle private_nh, tf::TransformListener* tf){
    tf_ = tf;
    private_nh.param<std::string>("base_frame", base_frame_, "/base_link");
    private_nh.param<std::string>("global_frame", global_frame_, "/odom");

    standalone_ = getParamDef(private_nh, "standalone", true);

    as_ = new actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>(private_nh, "swarm_nav_goal", boost::bind(&ROSAgent::executeCB, this, _1), false);
    
    ros::NodeHandle nh;
    id_ = nh.getNamespace();
    if (strcmp(id_.c_str(), "/") == 0) {
      char hostname[1024];
      hostname[1023] = '\0';
      gethostname(hostname,1023); 
      id_ = std::string(hostname);
    }
    ROS_INFO("Standalone My name is: %s",id_.c_str());
    ros::Duration(1.0).sleep();
    initCommon(nh);

    getParam(private_nh,"footprint_radius",&footprint_radius_);

    radius_ = footprint_radius_;
    //only coverting round footprints for now
    geometry_msgs::PolygonStamped footprint;
    geometry_msgs::Point32 p;
    double angle = 0;
    double step = 2 * M_PI / 72;
    while(angle < 2 * M_PI){
      geometry_msgs::Point32 pt;
      pt.x = radius_ * cos(angle);
      pt.y = radius_ * sin(angle);
      pt.z = 0.0;
      footprint.polygon.points.push_back(pt);
      angle += step;
    }
    setFootprint(footprint);
    
    eps_= getParamDef(private_nh, "eps", 0.1);
    convex_ = getParamDef(private_nh, "convex", false);
    getParam(private_nh, "holo_robot",&holo_robot_);

    if (standalone_) 
      initParams(private_nh);

    as_->start();
    initialized_ = true;

  }

  void ROSAgent::executeCB(const move_base_msgs::MoveBaseGoalConstPtr& msg){
    bool done = false;

    tf::Stamped<tf::Pose> goal_pose, global_pose;
    tf::poseStampedMsgToTF(msg->target_pose, goal_pose);

    //just get the latest available transform... for accuracy they should send
    //goals in the frame of the planner
    goal_pose.stamp_ = ros::Time();

    try{
      tf_->transformPose(global_frame_, goal_pose, global_pose);
    }
    catch(tf::TransformException& ex){
      ROS_WARN("Failed to transform the goal pose from %s into the %s frame: %s",
	       goal_pose.frame_id_.c_str(), global_frame_.c_str(), ex.what());
      as_->setPreempted();
      return;
    }

    geometry_msgs::PoseStamped global_pose_msg;
    tf::poseStampedTFToMsg(global_pose, global_pose_msg);

    Vector2 goal = Vector2(global_pose_msg.pose.position.x, global_pose_msg.pose.position.y);

   
    int counter = 0;
    double ang = tf::getYaw(global_pose_msg.pose.orientation);
    //ROS_ERROR("Vector goal %f, %f, ang %f", goal.x(), goal.y(), ang);
    
    ros::Rate rate = ros::Rate(10);
    bool failed = false;
    while(!done && !failed){
      if (as_->isPreemptRequested() || ! ros::ok()) {
	as_->setPreempted();
	geometry_msgs::Twist msg;
	twist_pub_.publish(msg);
	return;
      }
      geometry_msgs::Twist msg;
      
      msg = computeVelocityCommand(goal, ang);

      twist_pub_.publish(msg);
      if (msg.linear.x == 0 && msg.linear.y == 0 && msg.angular.z == 0 ) {
	if (collvoid::abs(goal-position_) < xy_goal_tolerance_) { 
	  done = true;
	}
	else {
	  if (counter++ > 5) {
	    failed = true;
	  }
	}
      }
      else{
	counter = 0;
      }
      rate.sleep();
	
    }
    ROS_INFO("done");
    if (done)
      as_->setSucceeded();
    if (failed)
      as_->setPreempted();
      
  }

  

  void ROSAgent::initParams(ros::NodeHandle private_nh) {
    getParam(private_nh,"max_vel_with_obstacles", &max_vel_with_obstacles_);
    getParam(private_nh,"max_vel_x", &max_vel_x_);
    getParam(private_nh,"min_vel_x", &min_vel_x_);
    getParam(private_nh,"max_vel_y", &max_vel_y_);
    getParam(private_nh,"min_vel_y", &min_vel_y_);
    getParam(private_nh,"max_vel_th", &max_vel_th_);
    getParam(private_nh,"min_vel_th", &min_vel_th_);
    getParam(private_nh,"min_vel_th_inplace", &min_vel_th_inplace_);
    getParam(private_nh,"use_obstacles", &use_obstacles_);


    time_horizon_obst_ = getParamDef(private_nh,"time_horizon_obst",10.0);
    time_to_holo_ = getParamDef(private_nh,"time_to_holo", 0.4);
    min_error_holo_ = getParamDef(private_nh,"min_error_holo", 0.01);
    max_error_holo_ = getParamDef(private_nh, "max_error_holo", 0.15);
    delete_observations_ = getParamDef(private_nh, "delete_observations", true);
    threshold_last_seen_ = getParamDef(private_nh,"threshold_last_seen",1.0);
    max_neighbors_ = getParamDef(private_nh, "max_neighbors", 10);
    sim_period_ = getParamDef(private_nh, "sim_period", 0.2);
    
    yaw_goal_tolerance_ = getParamDef(private_nh, "yaw_goal_tolerance", 0.1);
    xy_goal_tolerance_ = getParamDef(private_nh, "xy_goal_tolerance", 0.1);

    
    getParam(private_nh, "orca", &orca_);
    getParam(private_nh, "clearpath", &clearpath_);
    getParam(private_nh, "use_truncation", &use_truncation_);
    
    num_samples_ = getParamDef(private_nh, "num_samples", 400);
    type_vo_ = getParamDef(private_nh, "type_vo", 0); //HRVO

    trunc_time_ = getParamDef(private_nh,"trunc_time",10.0);
    left_pref_ = getParamDef(private_nh,"left_pref",0.1);
    twist_pub_ = private_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  }


  void ROSAgent::initAsMe(tf::TransformListener* tf){
    //Params (get from server or set via local_planner)
    tf_ = tf;
    controlled_ = true;
    ros::NodeHandle nh;
    getParam(nh, "move_base/use_obstacles", &use_obstacles_);
    controlled_ = getParamDef(nh,"move_base/controlled", true);
    initCommon(nh);
    initialized_ = true;
  }
 
  void ROSAgent::initCommon(ros::NodeHandle nh){
    //Publishers
    vo_pub_ = nh.advertise<visualization_msgs::Marker>("vo", 1);
    lines_pub_ = nh.advertise<visualization_msgs::Marker>("orca_lines", 1);
    samples_pub_ = nh.advertise<visualization_msgs::MarkerArray>("samples", 1);
    speed_pub_ = nh.advertise<visualization_msgs::Marker>("speed",1);
    obstacles_pub_ = nh.advertise<visualization_msgs::Marker>("obstacles", 1);

    //Subscribers
    odom_sub_ = nh.subscribe("odom",1, &ROSAgent::odomCallback, this);
    position_share_sub_ = nh.subscribe("position_share", 1, &ROSAgent::positionShareCallback, this);

    laser_scan_sub_.subscribe (nh, "scan_obst", 1);
    laser_notifier.reset(new tf::MessageFilter<sensor_msgs::LaserScan>(laser_scan_sub_, *tf_, global_frame_, 10));

    laser_notifier->registerCallback(boost::bind(&ROSAgent::baseScanCallback, this, _1));
    laser_notifier->setTolerance(ros::Duration(0.1));
    //laser_scan_sub_ = nh.subscribe("base_scan", 1, &ROSAgent::baseScanCallback, this);
    

    ROS_INFO("New Agent as me initialized");
  }


  geometry_msgs::Twist ROSAgent::computeVelocityCommand(Vector2 waypoint, double goal_ang) {
    if(!initialized_){
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return geometry_msgs::Twist();
    }
    tf::Stamped<tf::Pose> global_pose;
    //let's get the pose of the robot in the frame of the plan
    global_pose.setIdentity();
    global_pose.frame_id_ = base_frame_;
    global_pose.stamp_ = ros::Time();
    tf_->transformPose(global_frame_, global_pose, global_pose);
    // Set current velocities from odometry
    geometry_msgs::Twist global_vel;
    me_lock_.lock();
    global_vel.linear.x = base_odom_.twist.twist.linear.x;
    global_vel.linear.y = base_odom_.twist.twist.linear.y;
    global_vel.angular.z = base_odom_.twist.twist.angular.z;
    setAgentParams(this);
    me_lock_.unlock();

    Vector2 pos = Vector2(global_pose.getOrigin().x(), global_pose.getOrigin().y());
    Vector2 goal_dir = waypoint - pos;

    
    geometry_msgs::Twist cmd_vel;

    //rotate to goal:
    if (collvoid::abs(goal_dir) < xy_goal_tolerance_) {
      double robot_yaw = tf::getYaw(global_pose.getRotation());
      double ang_diff = angles::shortest_angular_distance(robot_yaw, goal_ang);
      if (fabs(ang_diff) < yaw_goal_tolerance_) {
	return cmd_vel;
      }
      double v_th_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
						   std::max(min_vel_th_inplace_, ang_diff)) : std::max(-1.0 * max_vel_th_,
													   std::min(-1.0 * min_vel_th_inplace_, ang_diff));
      v_th_samp = sign(v_th_samp) * std::max(min_vel_th_inplace_,fabs(v_th_samp));
      cmd_vel.angular.z = v_th_samp;
      return cmd_vel;
    }
    
    else {
      if (collvoid::abs(goal_dir) > max_vel_x_) {
	goal_dir = max_vel_x_ * collvoid::normalize(goal_dir);
      }
      else if (collvoid::abs(goal_dir) < min_vel_x_) {
	goal_dir = min_vel_x_ * 1.2* collvoid::normalize(goal_dir);
      }

      double goal_dir_ang = atan2(goal_dir.y(), goal_dir.x());
      //ROS_INFO("Pose (%f, %f), goal (%f, %f), dir (%f, %f), ang %f", pos.x(), pos.y(), waypoint.x(), waypoint.y(), goal_dir.x(), goal_dir.y(), goal_dir_ang);

      
      computeNewVelocity(goal_dir, cmd_vel);

      if(std::abs(cmd_vel.angular.z)<min_vel_th_)
	cmd_vel.angular.z = 0.0;
      if(std::abs(cmd_vel.linear.x)<min_vel_x_)
	cmd_vel.linear.x = 0.0;
      if(std::abs(cmd_vel.linear.y)<min_vel_y_)
	cmd_vel.linear.y = 0.0;

      return cmd_vel;
    }
    
    
  }
  

  void ROSAgent::computeNewVelocity(Vector2 pref_velocity, geometry_msgs::Twist& cmd_vel){
    boost::mutex::scoped_lock lock(me_lock_);
    //Forward project me
    setAgentParams(this);

    //get all neighbors
    updateAllNeighbors();

    new_velocity_ = Vector2(0.0,0.0);

    additional_orca_lines_.clear();
    vo_agents_.clear();

    //get closest agent/obstacle
    double min_dist_neigh = DBL_MAX;
    if (agent_neighbors_.size() > 0)
      min_dist_neigh =  collvoid::abs(agent_neighbors_[0]->position_ - position_);

    double min_dist = std::min(min_dist_neigh, min_dist_obst_);
    
    //incorporate NH constraints
    max_speed_x_ = max_vel_x_;

    if (!holo_robot_) {
      addNHConstraints(min_dist, pref_velocity);
    }
    //add acceleration constraints
    //addAccelerationConstraintsXY(max_vel_x_,acc_lim_x_, max_vel_y_, acc_lim_y_, velocity_, heading_, sim_period_, holo_robot_, additional_orca_lines_);

    computeObstacles();
    
    if (orca_){
      computeOrcaVelocity(pref_velocity);
    }
    else {
      samples_.clear();
      if(clearpath_) {
	computeClearpathVelocity(pref_velocity);
      }
      else {
	computeSampledVelocity(pref_velocity);
      }
    }


    double speed_ang = atan2(new_velocity_.y(), new_velocity_.x());
    double dif_ang = angles::shortest_angular_distance(heading_, speed_ang);

    //ROS_INFO("speed_ang %f, dif_ang %f, heading %f", speed_ang, dif_ang, heading_);

    if (!holo_robot_){
      double vel = collvoid::abs(new_velocity_);
      double vstar;
 
      if (std::abs(dif_ang) > EPSILON)
	vstar = calcVstar(vel,dif_ang);
      else
	vstar = max_vel_x_;

      cmd_vel.linear.x = std::min(vstar,vMaxAng());
      cmd_vel.linear.y = 0.0;

      //ROS_ERROR("dif_ang %f", dif_ang);
      if (std::abs(dif_ang) > 3.0*M_PI / 4.0) {
	cmd_vel.angular.z = sign(base_odom_.twist.twist.angular.z) * std::min(std::abs(dif_ang/time_to_holo_),max_vel_th_);
	
      }
      else {
	cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang/time_to_holo_),max_vel_th_);
      }
      //ROS_ERROR("vstar = %.3f", vstar);
      // if (std::abs(cmd_vel.angular.z) == max_vel_th_)
      // 	cmd_vel.linear.x = 0;
      
  
    }
    else {
      collvoid::Vector2 rotated_vel = rotateVectorByAngle(new_velocity_.x(), new_velocity_.y(), -heading_);

      cmd_vel.linear.x = rotated_vel.x();
      cmd_vel.linear.y = rotated_vel.y();
      // if (min_dist < 1.0/2.0 * radius_) {
      // 	if (min_dist == min_dist_neigh) {
      // 	  dif_ang = dif_ang;
      // 	}
      // 	else {
      // 	  double ang_obst = atan2(min_obst_vec.y(), min_obst_vec.x());
      // 	  double diff = angles::shortest_angular_distance(heading_, ang_obst);

      // 	  dif_ang = angles::shortest_angular_distance(0.0, diff - sign(diff) * M_PI / 2.0);
      // 	}
      // }
      //      if (std::abs(dif_ang) < M_PI/2.0) 
      if (min_dist > 2*footprint_radius_)
	cmd_vel.angular.z = sign(dif_ang) * std::min(std::abs(dif_ang),max_vel_th_);
    }

  }

  
  void ROSAgent::computeClearpathVelocity(Vector2 pref_velocity) {
    //account for nh error
    boost::mutex::scoped_lock lock(neighbors_lock_);

    radius_ += cur_allowed_error_;
    ((Agent*) this) -> computeClearpathVelocity(pref_velocity);
    radius_ -= cur_allowed_error_;
    
    publishHoloSpeed(position_, new_velocity_, global_frame_, base_frame_, speed_pub_);
    publishVOs(position_, vo_agents_, use_truncation_, global_frame_, base_frame_, vo_pub_);
    publishPoints(position_, samples_, global_frame_, base_frame_, samples_pub_);
    publishOrcaLines(additional_orca_lines_, position_, global_frame_, base_frame_, lines_pub_);

  }
  

  void ROSAgent::computeSampledVelocity(Vector2 pref_velocity) {

    createSamplesWithinMovementConstraints(samples_, base_odom_.twist.twist.linear.x, base_odom_.twist.twist.linear.y, base_odom_.twist.twist.angular.z, acc_lim_x_, acc_lim_y_, acc_lim_th_, min_vel_x_, max_vel_x_, min_vel_y_, max_vel_y_, min_vel_th_, max_vel_th_ , heading_, pref_velocity, sim_period_, num_samples_, holo_robot_);

    //account for nh error
    boost::mutex::scoped_lock lock(neighbors_lock_);

    radius_ += cur_allowed_error_;
    ((Agent*) this) -> computeSampledVelocity(pref_velocity);
    radius_ -= cur_allowed_error_;
   
    //    Vector2 null_vec = Vector2(0,0);
    publishHoloSpeed(position_, new_velocity_, global_frame_, base_frame_, speed_pub_);
    publishVOs(position_, vo_agents_, use_truncation_, global_frame_, base_frame_, vo_pub_);
    publishPoints(position_, samples_, global_frame_, base_frame_, samples_pub_);
    publishOrcaLines(additional_orca_lines_, position_, global_frame_, base_frame_, lines_pub_);
    
  }
  
  
  void ROSAgent::computeOrcaVelocity(Vector2 pref_velocity){
    ((Agent*) this) -> computeOrcaVelocity(pref_velocity, convex_);

    // publish calcuated speed and orca_lines
    publishHoloSpeed(position_, new_velocity_, global_frame_, base_frame_, speed_pub_);
    publishOrcaLines(orca_lines_, position_, global_frame_, base_frame_, lines_pub_);

  }

  void ROSAgent::addNHConstraints(double min_dist, Vector2 pref_velocity){
    double min_error = min_error_holo_;
    double max_error = max_error_holo_;
    double error = max_error;
    double v_max_ang = vMaxAng();

    //ROS_ERROR("v_max_ang %.2f", v_max_ang);

    if (min_dist < 2.0*footprint_radius_ + cur_loc_unc_radius_){
      error = (max_error-min_error) / (collvoid::sqr(2*(footprint_radius_ + cur_loc_unc_radius_))) * collvoid::sqr(min_dist) + min_error; // how much error do i allow?
      //ROS_DEBUG("Error = %f", error);
      if (min_dist < 0) {
	error = min_error;
	// ROS_DEBUG("%s I think I am in collision", me_->getId().c_str());
      }
    }
    cur_allowed_error_ = 1.0/3.0 * cur_allowed_error_ + 2.0/3.0 * error;
    //ROS_ERROR("error = %f", cur_allowed_error_);
    double speed_ang = atan2(pref_velocity.y(), pref_velocity.x());
    double dif_ang = angles::shortest_angular_distance(heading_, speed_ang);
    if (std::abs(dif_ang) > M_PI/2.0) { // || cur_allowed_error_ < 2.0 * min_error) {
      double max_track_speed  = calculateMaxTrackSpeedAngle(time_to_holo_, M_PI / 2.0, cur_allowed_error_, max_vel_x_, max_vel_th_, v_max_ang);
      if (max_track_speed <= 2*min_error) {
	max_track_speed = 2* min_error;
      }
      addMovementConstraintsDiffSimple(max_track_speed, heading_, additional_orca_lines_);
    }
    else {
      addMovementConstraintsDiff(cur_allowed_error_, time_to_holo_, max_vel_x_,max_vel_th_, heading_, v_max_ang, additional_orca_lines_);
    }
    max_speed_x_ = vMaxAng();
 
  }

  void ROSAgent::computeObstacles(){
    boost::mutex::scoped_lock lock(obstacle_lock_);
    
    std::vector<Vector2> own_footprint;
    BOOST_FOREACH(geometry_msgs::Point32 p, footprint_msg_.polygon.points) {
      own_footprint.push_back(Vector2(p.x, p.y));
      //      ROS_WARN("footprint point p = (%f, %f) ", footprint_[i].x, footprint_[i].y);
    }
    min_dist_obst_ = DBL_MAX;
    ros::Time cur_time = ros::Time::now();
    int i = 0;
    std::vector<int> delete_list;
    BOOST_FOREACH(Obstacle obst, obstacles_from_laser_) {
      if (obst.point1 != obst.point2) {// && (cur_time - obst.last_seen).toSec() < 0.2) {
	double dist = distSqPointLineSegment(obst.point1, obst.point2, position_);
	if (dist < sqr((abs(velocity_) + 4.0 * footprint_radius_))) {
	  if (use_obstacles_) {
	    if (orca_) {
	      createObstacleLine(own_footprint, obst.point1, obst.point2);
	    }
	    else {
	      VO obstacle_vo = createObstacleVO(position_, footprint_radius_, own_footprint, obst.point1, obst.point2);
	      vo_agents_.push_back(obstacle_vo);
	    }
	  }
	  if (dist < min_dist_obst_) {
	    min_dist_obst_ = dist;
	  }
	}
      }
      else {
	delete_list.push_back(i);
      }
      i++;
    }
    for (int i = (int)delete_list.size() -1; i >= 0; i--) {
     obstacles_from_laser_.erase(obstacles_from_laser_.begin() + delete_list[i]);
    }

  }
  

  bool ROSAgent::compareNeighborsPositions(const AgentPtr& agent1, const AgentPtr& agent2) {
    return compareVectorPosition(agent1->position_, agent2->position_);
  }


  bool ROSAgent::compareConvexHullPointsPosition(const ConvexHullPoint& p1, const ConvexHullPoint& p2) {
    return collvoid::absSqr(p1.point) <= collvoid::absSqr(p2.point);
  }

  
  bool ROSAgent::compareVectorPosition(const collvoid::Vector2& v1, const collvoid::Vector2& v2){
    return collvoid::absSqr(position_ - v1) <= collvoid::absSqr(position_ - v2);
  }

  void ROSAgent::sortObstacleLines(){
    boost::mutex::scoped_lock lock(obstacle_lock_);
    std::sort(obstacle_points_.begin(),obstacle_points_.end(), boost::bind(&ROSAgent::compareVectorPosition,this,_1,_2));
  }

  collvoid::Vector2 ROSAgent::LineSegmentToLineSegmentIntersection(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4){
    double r, s, d;
    collvoid::Vector2 res;
    //Make sure the lines aren't parallel
    if ((y2 - y1) / (x2 - x1) != (y4 - y3) / (x4 - x3)){
      d = (((x2 - x1) * (y4 - y3)) - (y2 - y1) * (x4 - x3));
      if (d != 0){
	r = (((y1 - y3) * (x4 - x3)) - (x1 - x3) * (y4 - y3)) / d;
	s = (((y1 - y3) * (x2 - x1)) - (x1 - x3) * (y2 - y1)) / d;
	if (r >= 0 && r <= 1){
	  if (s >= 0 && s <= 1){
	    return collvoid::Vector2(x1 + r * (x2 - x1), y1 + r * (y2 - y1));
	  }
	}
      }
    } 
    return res;
  }

  bool ROSAgent::pointInNeighbor(collvoid::Vector2& point) {
    for (size_t i = 0; i<agent_neighbors_.size();i++){
      if (collvoid::abs(point - agent_neighbors_[i]->position_) <= agent_neighbors_[i]->radius_)
	return true;
    }
    return false;
  }

  double ROSAgent::getDistToFootprint(collvoid::Vector2& point){
    collvoid::Vector2 result, null;
    for (size_t i = 0; i < footprint_lines_.size(); i++){
      collvoid::Vector2 first = footprint_lines_[i].first;
      collvoid::Vector2 second = footprint_lines_[i].second;
          
      result = LineSegmentToLineSegmentIntersection(first.x(),first.y(),second.x(),second.y(), 0.0, 0.0, point.x(),point.y());
      if (result != null) {
	//ROS_DEBUG("Result = %f, %f, dist %f", result.x(), result.y(), collvoid::abs(result));
	return collvoid::abs(result);
      }
    }
    ROS_DEBUG("Obstacle Point within Footprint. I am close to/in collision");
    return -1;
  }

  void ROSAgent::computeObstacleLine(Vector2& obst){
    Line line;
    Vector2 relative_position = obst - position_;
    double dist_to_footprint;
    double dist = collvoid::abs(position_ - obst);
    if (!has_polygon_footprint_)
      dist_to_footprint = footprint_radius_;
    else {
      dist_to_footprint = getDistToFootprint(relative_position);
      if (dist_to_footprint == -1){
	dist_to_footprint = footprint_radius_;
      }
    }
    dist  = dist - dist_to_footprint - 0.03;
    //if (dist < (double)max_vel_with_obstacles_){
    //  dist *= dist;
    //} 
    //    line.point = normalize(relative_position) * (dist - dist_to_footprint - 0.03);
    line.point = normalize(relative_position) * (dist); 
    line.dir = Vector2 (-normalize(relative_position).y(),normalize(relative_position).x()) ; 
    additional_orca_lines_.push_back(line);
  }
  
  void ROSAgent::createObstacleLine(std::vector<Vector2>& own_footprint, Vector2& obst1, Vector2& obst2) {
      
    double dist = distSqPointLineSegment(obst1, obst2, position_);

    if (dist == absSqr(position_ - obst1)) {
      computeObstacleLine(obst1);
    }
    else if (dist == absSqr(position_ - obst2)) {
      computeObstacleLine(obst2);
    }
    // if (false) {
    // }
    else {
      Vector2 position_obst = projectPointOnLine(obst1, obst2-obst1, position_);
      Vector2 rel_position = position_obst - position_;
      dist = std::sqrt(dist);
      double dist_to_footprint = getDistToFootprint(rel_position);
      if (dist_to_footprint == -1){
	dist_to_footprint = footprint_radius_;
      }
      dist = dist - dist_to_footprint - 0.03;

      if (dist < 0.0) {
	Line line;
	line.point = (dist - 0.02) * normalize (rel_position);
	line.dir = normalize(obst1 - obst2);
	additional_orca_lines_.push_back(line);
	return;
      }

      if (abs(position_ - obst1) > 2 * footprint_radius_ && abs(position_ - obst2) > 2 * footprint_radius_) {
	Line line;
	line.point = dist * normalize (rel_position);
	line.dir = -normalize(obst1 - obst2);
	additional_orca_lines_.push_back(line);
	return;
	
      }
      // return;
      rel_position = (abs(rel_position) - dist / 2.0) * normalize(rel_position);
      
      std::vector<Vector2> obst;
      obst.push_back(obst1 - position_obst);
      obst.push_back(obst2 - position_obst);
      std::vector<Vector2> mink_sum = minkowskiSum(own_footprint, obst);

      Vector2 min, max;
      double min_ang = 0.0;
      double max_ang = 0.0; 
      
      for (int i = 0; i< (int) mink_sum.size(); i++){
	double angle = angleBetween(rel_position, rel_position + mink_sum[i]);
	if (leftOf(Vector2(0.0,0.0), rel_position, rel_position + mink_sum[i])) {
	  if (-angle < min_ang) {
	    min = rel_position + mink_sum[i];
	    min_ang = -angle;
	  }
      }
	else {
	  if (angle > max_ang) {
	    max = rel_position + mink_sum[i];
    	  max_ang = angle;
	  }
	}
      }
      
      Line line;
      line.point = (dist / 2.0) * normalize(rel_position);
      if (absSqr(position_obst - obst1) > absSqr(position_obst - obst2)) {
	// ROS_ERROR("max_ang = %.2f", max_ang);
	line.dir = rotateVectorByAngle(normalize(max), 0.1);
      }
      else {
	// ROS_ERROR("min_ang = %.2f", min_ang);
	line.dir = rotateVectorByAngle(normalize(min), 0.1);;
      }
      additional_orca_lines_.push_back(line);

    }
  }

  
  // void ROSAgent::computeObstacleLines(){
  //   std::vector<int> delete_points;
  //   for(size_t i = 0; i< obstacle_points_.size(); i++){
  //     //ROS_DEBUG("obstacle at %f %f dist %f",obstacle_points_[i].x(),obstacle_points_[i].y(),collvoid::abs(position_-obstacle_points_[i]));
  //     if (pointInNeighbor(obstacle_points_[i])){
  // 	delete_points.push_back((int)i);
  // 	continue;
  //     }
  //     double dist = collvoid::abs(position_ - obstacle_points_[i]);
  //     Line line;
  //     Vector2 relative_position = obstacle_points_[i] - position_;
  //     double dist_to_footprint;
  //     if (!has_polygon_footprint_)
  // 	dist_to_footprint = footprint_radius_;
  //     else {
  // 	dist_to_footprint = getDistToFootprint(relative_position);
  // 	if (dist_to_footprint == -1){
  // 	  dist_to_footprint = footprint_radius_;
  // 	}
  //     }
  //     dist = std::min(dist - dist_to_footprint - 0.01, (double)max_vel_with_obstacles_);
  //     //if (dist < (double)max_vel_with_obstacles_){
  //     //  dist *= dist;
  //     //} 
  //     //    line.point = normalize(relative_position) * (dist - dist_to_footprint - 0.03);
  //     line.point = normalize(relative_position) * (dist); 
  //     line.dir = Vector2 (-normalize(relative_position).y(),normalize(relative_position).x()) ; 

  //     //TODO relatvie velocity instead of velocity!!
  //     if (dist > time_horizon_obst_ * collvoid::abs(velocity_))
  // 	return;

      
  //     additional_orca_lines_.push_back(line);
    
  //   }
  //   if (delete_observations_) 
  //     return;
  //   else {
  //     while(!delete_points.empty()){
  // 	int del = delete_points.back();
  // 	obstacle_points_.erase(obstacle_points_.begin()+del);
  // 	delete_points.pop_back();
  //     }
  //   }
  // }
 
  void ROSAgent::setFootprint(geometry_msgs::PolygonStamped footprint ){
    if (footprint.polygon.points.size() < 2) {
      ROS_ERROR("The footprint specified has less than two nodes");
      return;
    }
    footprint_msg_ = footprint;
    setMinkowskiFootprintVector2(footprint_msg_);

    footprint_lines_.clear();
    geometry_msgs::Point32 p = footprint_msg_.polygon.points[0];
    collvoid::Vector2 first = collvoid::Vector2(p.x, p.y);
    collvoid::Vector2 old = collvoid::Vector2(p.x, p.y);
    //add linesegments for footprint
    for (size_t i = 0; i<footprint_msg_.polygon.points.size(); i++) {
      p = footprint_msg_.polygon.points[i];
      collvoid::Vector2 point = collvoid::Vector2(p.x, p.y);
      footprint_lines_.push_back(std::make_pair(old, point));
      old = point;
    }
    //add last segment
    footprint_lines_.push_back(std::make_pair(old, first));
    has_polygon_footprint_ = true;
  }

  void ROSAgent::setFootprintRadius(double radius){
    footprint_radius_ = radius;
    radius_ = radius + cur_loc_unc_radius_;
  }

  void ROSAgent::setMinkowskiFootprintVector2(geometry_msgs::PolygonStamped minkowski_footprint) {
    minkowski_footprint_.clear();
    BOOST_FOREACH(geometry_msgs::Point32 p, minkowski_footprint.polygon.points) {
      minkowski_footprint_.push_back(Vector2(p.x, p.y));
    }
  }

  void ROSAgent::setIsHoloRobot(bool holo_robot) {
    holo_robot_ = holo_robot;
  }
  
  void ROSAgent::setRobotBaseFrame(std::string base_frame){
    base_frame_ = base_frame;
  }

  void ROSAgent::setGlobalFrame(std::string global_frame){
    global_frame_ = global_frame;
  }

  void ROSAgent::setId(std::string id) {
    this->id_ = id;
  }

  void ROSAgent::setMaxVelWithObstacles(double max_vel_with_obstacles){
    max_vel_with_obstacles_ = max_vel_with_obstacles;
  }

  void ROSAgent::setWheelBase(double wheel_base){
    wheel_base_ = wheel_base;
  }

  void ROSAgent::setAccelerationConstraints(double acc_lim_x, double acc_lim_y, double acc_lim_th){
    acc_lim_x_ = acc_lim_x;
    acc_lim_y_ = acc_lim_y;
    acc_lim_th_ = acc_lim_th;
  }
  
  void ROSAgent::setMinMaxSpeeds(double min_vel_x, double max_vel_x, double min_vel_y, double max_vel_y, double min_vel_th, double max_vel_th, double min_vel_th_inplace){
    min_vel_x_ = min_vel_x;
    max_vel_x_ = max_vel_x;
    min_vel_y_ = min_vel_y;
    max_vel_y_ = max_vel_y;
    min_vel_th_ = min_vel_th;
    max_vel_th_ = max_vel_th;
    min_vel_th_inplace_ = min_vel_th_inplace;
  }

  void ROSAgent::setPublishPositionsPeriod(double publish_positions_period){
    publish_positions_period_ = publish_positions_period;
  }
  
  void ROSAgent::setPublishMePeriod(double publish_me_period){
    publish_me_period_ = publish_me_period;
  }
  
  void ROSAgent::setTimeToHolo(double time_to_holo){
    time_to_holo_ = time_to_holo;
  }
  
  void ROSAgent::setTimeHorizonObst(double time_horizon_obst){
    time_horizon_obst_ = time_horizon_obst;
  }
  
  void ROSAgent::setMinMaxErrorHolo(double min_error_holo, double max_error_holo){
    min_error_holo_ = min_error_holo;
    max_error_holo_ = max_error_holo;
  }
  
  void ROSAgent::setDeleteObservations(bool delete_observations){
    delete_observations_ = delete_observations;
  }
  
  void ROSAgent::setThresholdLastSeen(double threshold_last_seen){ //implement!!
    threshold_last_seen_ = threshold_last_seen;
  }
  
  void ROSAgent::setTypeVO(int type_vo) {
    type_vo_ = type_vo;
  }

  void ROSAgent::setOrca(bool orca) {
    orca_ = orca;
  }

  void ROSAgent::setConvex(bool convex) {
    convex_ = convex;
  }

  void ROSAgent::setClearpath(bool clearpath) {
    clearpath_ = clearpath;
  }

  void ROSAgent::setUseTruncation(bool use_truncation){
    use_truncation_ = use_truncation;
  }

  void ROSAgent::setNumSamples(int num_samples){
    num_samples_ = num_samples;
  }

  bool ROSAgent::isHoloRobot() {
    return holo_robot_;
  }
  
  ros::Time ROSAgent::lastSeen(){
    return last_seen_;
  }

  void ROSAgent::positionShareCallback(const swarming_turtles_msgs::PositionShares::ConstPtr& msg) {
    boost::mutex::scoped_lock(neighbors_lock_);
    agents_.positions.clear();
    for (int i=0; i<(int)msg->positions.size(); i++) {  
      agents_.positions.push_back(msg->positions[i]);
    }
  }
  
  void ROSAgent::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    //we assume that the odometry is published in the frame of the base
    boost::mutex::scoped_lock(me_lock_);
    base_odom_.twist.twist.linear.x = msg->twist.twist.linear.x;
    base_odom_.twist.twist.linear.y = msg->twist.twist.linear.y;
    base_odom_.twist.twist.angular.z = msg->twist.twist.angular.z;
    
    last_seen_ = msg->header.stamp;
    tf::Stamped<tf::Pose> global_pose;
    //let's get the pose of the robot in the frame of the plan
    global_pose.setIdentity();
    global_pose.frame_id_ = base_frame_;
    global_pose.stamp_ = msg->header.stamp;

    try {
      tf_->waitForTransform(global_frame_, base_frame_, global_pose.stamp_, ros::Duration(0.2));
      tf_->transformPose(global_frame_, global_pose, global_pose);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ROS_ERROR("point odom transform failed");
      return;
    };

    geometry_msgs::PoseStamped pose_msg;
    tf::poseStampedTFToMsg(global_pose, pose_msg);

    base_odom_.pose.pose = pose_msg.pose;
  }


  void ROSAgent::updateAllNeighbors() {
    boost::mutex::scoped_lock(neighbors_lock_);
    agent_neighbors_.clear();
    for (int i=0; i<(int)agents_.positions.size(); i++) {
      swarming_turtles_msgs::PositionShare msg = agents_.positions[i];
      
      ROSAgentPtr agent(new ROSAgent);
      agent->id_ = msg.name;
      agent->holo_robot_ = msg.holo_robot;
      agent->radius_ = msg.radius;
      agent->controlled_ = msg.controlled;
      agent->velocity_ = Vector2(msg.velocity.x, msg.velocity.y);
      agent->position_ = Vector2(msg.position.x, msg.position.y);
      
      agent->footprint_msg_ = msg.footprint;
      //calculate mink footprint?
      
      //TODO?
      agent->timestep_ = sim_period_;
      agent_neighbors_.push_back(agent);
      ROS_INFO("neighbor %s, radius = %f,   pos(%.2f, %.2f), vel (%.2f, %.2f)", msg.name.c_str(), agent->radius_, agent->position_.x() - position_.x(), agent->position_.y() - position_.y(), agent->velocity_.x(), agent->velocity_.y());
    }
  }
  
  void ROSAgent::setAgentParams(ROSAgent* agent) {
    double time_dif = (ros::Time::now() - agent->last_seen_).toSec();
    double yaw, x_dif, y_dif, th_dif;
    //time_dif = 0.0;

    yaw = tf::getYaw(agent->base_odom_.pose.pose.orientation);
    th_dif =  time_dif * agent->base_odom_.twist.twist.angular.z;
    if (agent->isHoloRobot()) {
      x_dif = time_dif * agent->base_odom_.twist.twist.linear.x;
      y_dif = time_dif * agent->base_odom_.twist.twist.linear.y;
    }
    else {
      x_dif = time_dif * agent->base_odom_.twist.twist.linear.x * cos(yaw + th_dif/2.0);
      y_dif = time_dif * agent->base_odom_.twist.twist.linear.x * sin(yaw + th_dif/2.0);
      Vector2 vel = rotateVectorByAngle(x_dif, y_dif, yaw);
      x_dif = vel.x();
      y_dif = vel.y();

    }
    agent->heading_ = yaw + th_dif;
    agent->position_ = Vector2(agent->base_odom_.pose.pose.position.x + x_dif, agent->base_odom_.pose.pose.position.y + y_dif);
    //agent->last_seen_ = ros::Time::now();
    agent->timestep_ = time_dif;

    //agent->footprint_ = agent->minkowski_footprint_;
    agent->footprint_ = rotateFootprint(agent->minkowski_footprint_, agent->heading_);
      
    if (agent->holo_robot_) {
      agent->velocity_ = rotateVectorByAngle(agent->base_odom_.twist.twist.linear.x, agent->base_odom_.twist.twist.linear.y, (yaw+th_dif));
    }
    else {
      double dif_x, dif_y, dif_ang;
      dif_ang = sim_period_ * agent->base_odom_.twist.twist.angular.z;
      dif_x = agent->base_odom_.twist.twist.linear.x * cos(dif_ang / 2.0);
      dif_y = agent->base_odom_.twist.twist.linear.x * sin(dif_ang / 2.0);
      agent->velocity_ = rotateVectorByAngle(dif_x, dif_y, (yaw+th_dif));
    }
  }
  
  double ROSAgent::vMaxAng(){
    //double theoretical_max_v = max_vel_th_ * wheel_base_ / 2.0; 
    //return theoretical_max_v - std::abs(base_odom_.twist.twist.angular.z) * wheel_base_/2.0;
    return max_vel_x_; //TODO: fixme!!
  }

  
  geometry_msgs::PolygonStamped ROSAgent::createFootprintMsgFromVector2(const std::vector<Vector2>& footprint) {
    geometry_msgs::PolygonStamped result;
    result.header.frame_id = base_frame_;
    result.header.stamp = ros::Time::now();
    
    BOOST_FOREACH(Vector2 point, footprint) {
      geometry_msgs::Point32 p;
      p.x = point.x();
      p.y = point.y();
      result.polygon.points.push_back(p);
    }
    
    return result;
  }

  std::vector<Vector2> ROSAgent::rotateFootprint(const std::vector<Vector2>& footprint, double angle) {
    std::vector<Vector2> result;
    BOOST_FOREACH(Vector2 point, footprint) {
      Vector2 rotated = rotateVectorByAngle(point, angle);
      result.push_back(rotated);
    }
    return result;
  }
  
  geometry_msgs::PoseStamped ROSAgent::transformMapPoseToBaseLink(geometry_msgs::PoseStamped in) {
    geometry_msgs::PoseStamped result;
    try {
      tf_->waitForTransform(global_frame_, base_frame_, in.header.stamp, ros::Duration(0.3));
      tf_->transformPose(base_frame_, in, result);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ROS_ERROR("point transform failed");
    };
    return result;
  }


  void ROSAgent::baseScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    sensor_msgs::PointCloud cloud;
    //  ROS_ERROR("got cloud");

    try {
      tf_->waitForTransform(msg->header.frame_id, global_frame_, msg->header.stamp, ros::Duration(0.3));
      projector_.transformLaserScanToPointCloud(global_frame_, *msg, cloud, *tf_);
    }
    catch (tf::TransformException& e) {
      ROS_ERROR("%s", e.what());
      return;
    }

    boost::mutex::scoped_lock lock(obstacle_lock_);

    obstacles_from_laser_.clear();

    double threshold_convex = 0.03;
    double threshold_concave = -0.03;
    //    ROS_ERROR("%d", (int)cloud.points.size());
    
    for (int i = 0; i< (int)cloud.points.size(); i++) {
      Vector2 start = Vector2(cloud.points[i].x, cloud.points[i].y);
      while (pointInNeighbor(start) && i < (int) cloud.points.size() ) {
	i++;
       	start = Vector2(cloud.points[i].x, cloud.points[i].y);
      }
      if (i == (int)cloud.points.size()) {
	if (!pointInNeighbor(start)) {
	  //obstacles_from_laser_.push_back(std::make_pair(start,start));
	}
	return;
      }
      
      bool found = false;
      Vector2 prev = Vector2(start.x(), start.y());
      double first_ang = 0;
      double prev_ang = 0;
      Vector2 next;
      while (!found) {
	i++;
	if (i == (int)cloud.points.size()) {
	  break;
	}
	next = Vector2(cloud.points[i].x, cloud.points[i].y);
	while (pointInNeighbor(next) && i < (int) cloud.points.size() ) {
	  i++;
	  next = Vector2(cloud.points[i].x, cloud.points[i].y);
	}
     
	if (abs(next-prev) > 2*footprint_radius_) {
	  found = true;
	  break;
	}
	Vector2 dif = next - start;
	double ang = atan2(dif.y(), dif.x());
	if (prev != start) {
	  if (ang - first_ang < threshold_concave) {
	    found = true;
	    i -=2;
	    break;
	  }
	  if (ang - prev_ang < threshold_concave) {
	    found = true;
	    i-=2;
	    break;
	  }
	  if (ang - prev_ang > threshold_convex) { //going towards me
	    found = true;
	    i -=2;
	    break;
	  }
	  if (ang - first_ang > threshold_convex) { //going towards me
	    found = true;
	    i -=2;
	    break;
	  }

	  
	}
	else {
	  first_ang = ang;
	}
	prev = next;
	prev_ang = ang;
      }
      Obstacle obst;
      obst.point1 = start;
      obst.point2 = prev;
      obst.last_seen = msg->header.stamp;
    
      obstacles_from_laser_.push_back(obst);
    }
    publishObstacleLines(obstacles_from_laser_, global_frame_, base_frame_, obstacles_pub_);
    
  }

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ROSAgent");
  //ros::NodeHandle nh;
  ros::NodeHandle nh("~");
   
  ROSAgentPtr me(new ROSAgent);
  tf::TransformListener tf;
  me->init(nh,&tf);
  ROS_INFO("ROSAgent initialized");
  ros::spin();
  
}


