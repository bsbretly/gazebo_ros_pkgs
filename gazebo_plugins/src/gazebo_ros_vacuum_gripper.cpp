/*
 * Copyright 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
   Desc: GazeboVacquumGripper plugin for manipulating objects in Gazebo
   Author: Kentaro Wada
   Date: 7 Dec 2015
 */

#include <algorithm>
#include <assert.h>
// #include <math/gzmath.hh>
#include <std_msgs/Bool.h>
#include <gazebo_plugins/gazebo_ros_vacuum_gripper.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosVacuumGripper);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosVacuumGripper::GazeboRosVacuumGripper()
{
  connect_count_ = 0;
  status_ = true;
  contact_duration = 10; //in sec
  contact_done = false;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVacuumGripper::~GazeboRosVacuumGripper()
{
  update_connection_.reset();

  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_.join();

  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVacuumGripper::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ROS_INFO_NAMED("vacuum_gripper", "Loading gazebo_ros_vacuum_gripper");

  // Set attached model;
  parent_ = _model;

  // Get the world name.
  world_ = _model->GetWorld();

  // load parameters
  robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("vacuum_gripper", "vacuum_gripper plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  link_ = _model->GetLink(link_name_);
  if (!link_)
  {
    std::string found;
    physics::Link_V links = _model->GetLinks();
    for (size_t i = 0; i < links.size(); i++) {
      found += std::string(" ") + links[i]->GetName();
    }
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: link named: %s does not exist", link_name_.c_str());
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: You should check it exists and is not connected with fixed joint");
    ROS_FATAL_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper plugin error: Found links are: %s", found.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("vacuum_gripper", "vacuum_gripper plugin missing <serviceName>, cannot proceed");
    return;
  }
  else
    topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("vacuum_gripper", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(robot_namespace_);

  // Custom Callback Queue
  ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<std_msgs::Bool>(
    topic_name_, 1,
    boost::bind(&GazeboRosVacuumGripper::Connect, this),
    boost::bind(&GazeboRosVacuumGripper::Disconnect, this),
    ros::VoidPtr(), &queue_);
  pub_ = rosnode_->advertise(ao);

  // Custom Callback Queue
  ros::AdvertiseServiceOptions aso1 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "on", boost::bind(&GazeboRosVacuumGripper::OnServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv1_ = rosnode_->advertiseService(aso1);
  ros::AdvertiseServiceOptions aso2 =
    ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
    "off", boost::bind(&GazeboRosVacuumGripper::OffServiceCallback,
    this, _1, _2), ros::VoidPtr(), &queue_);
  srv2_ = rosnode_->advertiseService(aso2);

  // Custom Callback Queue
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosVacuumGripper::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosVacuumGripper::UpdateChild, this));

  ROS_INFO_NAMED("vacuum_gripper", "Loaded gazebo_ros_vacuum_gripper");
}

bool GazeboRosVacuumGripper::OnServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'on'");
  } else {
    status_ = true;
    ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: off -> on");
  }
  return true;
}
bool GazeboRosVacuumGripper::OffServiceCallback(std_srvs::Empty::Request &req,
                                     std_srvs::Empty::Response &res)
{
  if (status_) {
    status_ = false;
    ROS_INFO_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: status: on -> off");
  } else {
    ROS_WARN_NAMED("vacuum_gripper", "gazebo_ros_vacuum_gripper: already status is 'off'");
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVacuumGripper::UpdateChild()
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosVacuumGripper::UpdateChild");
#endif
  std_msgs::Bool grasping_msg;
  grasping_msg.data = false;
  if (!status_) {
#ifdef ENABLE_PROFILER
    IGN_PROFILE_BEGIN("publish status");
#endif
    pub_.publish(grasping_msg);
#ifdef ENABLE_PROFILER
    IGN_PROFILE_END();
#endif
    return;
  }
  // apply force
  lock_.lock();
#ifdef ENABLE_PROFILER
  IGN_PROFILE_BEGIN("apply force");
#endif
#if GAZEBO_MAJOR_VERSION >= 8
  ignition::math::Pose3d parent_pose = link_->WorldPose();
  physics::Model_V models = world_->Models();
#else
  ignition::math::Pose3d parent_pose = link_->GetWorldPose().Ign();
  physics::Model_V models = world_->GetModels();
#endif
  for (size_t i = 0; i < models.size(); i++) {
    if (models[i]->GetName() == link_->GetName() ||
        models[i]->GetName() == parent_->GetName())
    {
      continue;
    }
    physics::Link_V links = models[i]->GetLinks();
    for (size_t j = 0; j < links.size(); j++) {
      if(links[j]->GetName() != "wall")
        continue;
#if GAZEBO_MAJOR_VERSION >= 8
      ignition::math::Pose3d link_pose = links[j]->WorldPose();
#else
      ignition::math::Pose3d link_pose = links[j]->GetWorldPose().Ign();
#endif
      auto wall_x =  link_pose.X();
      auto quad_tip_x = parent_pose.X();
      double norm = std::abs(wall_x - quad_tip_x);
      // std::cout << links[j]->GetName() <<  norm << std::endl;
      // std::cout << link_->WorldPose() <<  std::endl << link_ ->RelativePose() << std::endl;
      if (norm < 0.4) {
        
#if GAZEBO_MAJOR_VERSION >= 8
        link_->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
        link_->SetAngularVel(ignition::math::Vector3d(0, 0, 0));
#else
        links[j]->SetLinearVel(link_->GetWorldLinearVel());
        links[j]->SetAngularVel(link_->GetWorldAngularVel());
#endif
   
        grasping_msg.data = true;
      }
    }
  }
  if (grasping_msg.data && !contact_done){
    contact_done = true;
    first_contact = ros::Time::now();
  } 
  if (contact_done){
    auto current_contact_duration = (ros::Time::now() - first_contact).toSec();
    if (current_contact_duration >= contact_duration){
      status_ = false;
    }
  }
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
  IGN_PROFILE_BEGIN("publish grasping_msg");
#endif
  pub_.publish(grasping_msg);
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
  lock_.unlock();
}

// Custom Callback Queue
////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboRosVacuumGripper::QueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosVacuumGripper::Connect()
{
  this->connect_count_++;
}

////////////////////////////////////////////////////////////////////////////////
// Someone subscribes to me
void GazeboRosVacuumGripper::Disconnect()
{
  this->connect_count_--;
}

}  // namespace gazebo
