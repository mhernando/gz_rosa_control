// Copyright (c) 2023, Miguel Hernando
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the company nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/// A omnidirectional (four mechanum wheels ) drive plugin for gazebo. Based on the diffdrive plugin
/*
 * Developed for ROSA social robot
 * \author  Miguel Hernando (miguel.hernando@upm.es)
 *
 */
/**
  Example Usage:
  
  
*/



#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "gazebo_ros_omni_drive.hpp"
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sdf/sdf.hh>

#ifdef NO_ERROR
// NO_ERROR is a macro defined in Windows that's used as an enum in tf2
#undef NO_ERROR
#endif

#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>
#include <vector>
#include <sstream>

namespace gazebo_plugins
{
class GazeboRosOmniDrivePrivate
{
public:

  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update odometry.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometry(const gazebo::common::Time & _current_time);

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

 /// Publish wheels transforms
  void PublishWheelsTf(const gazebo::common::Time & _current_time);

  /// Computes the wheels speeds and as consecuence the real twist
  /// updates the wheel joint speeds in gazebo
  /// \param[in] _dt step time in seconds
  /// \param[out] twist coherent with the computed wheel speeds
  geometry_msgs::msg::Twist ComputeRealTwist(double _dt);

  /// Computes Inverse Kinematics vx, vy, wz->v_wheels
  void IK(const double vx, const double vy, const double wz, double vm[]);

  /// Computes the Forward Kinematics v_wheels->vx, vy, wz
  void FK(const double vm[], double &vx, double &vy, double &wz);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// To broadcast TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Velocity received on command.
  geometry_msgs::msg::Twist target_cmd_vel_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Publish period in seconds.
  double publish_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Last publish time.
  gazebo::common::Time last_publish_time_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Robot base frame ID
  std::string robot_base_frame_;
  /// True to publish wheel tf messages.
  bool publish_wheel_tf_;
  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// Last Model Pose
  ignition::math::Pose3d last_pose_;

  //Omniwheel specific parameters : TBT
  double wheel_radius_; 
  double base_length_; 
  double base_width_;
  double wheel_max_speed_; //rad/s
  double wheel_acceleration_; //rad/s² 
  bool wheel_joints_update; //true if the wheel joints are bound correctly 
  double max_torque_;
  double joint_config_[4]={1,1,1,1};
  /// Pointers to wheel joints.
  gazebo::physics::JointPtr wheel_joints_[4]={};
  double wheel_target_velocities[4]={};
};

GazeboRosOmniDrive::GazeboRosOmniDrive()
: impl_(std::make_unique<GazeboRosOmniDrivePrivate>())
{
}

GazeboRosOmniDrive::~GazeboRosOmniDrive()
{
}

void GazeboRosOmniDrive::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  impl_->world_ = _model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();

  // Odometry
  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;

  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 20.0).first;
  impl_->update_period_ = update_rate > 0.0 ? 1.0 / update_rate : 0.0;

  impl_->last_update_time_ = impl_->world_->SimTime();

  // Update rate
  auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first;
  impl_->publish_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;

  impl_->last_publish_time_ = impl_->world_->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosOmniDrivePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", impl_->cmd_vel_sub_->get_topic_name());

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(impl_->ros_node_->get_logger(), "Advertise odometry on [%s]", impl_->odometry_pub_->get_topic_name());
  }

  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  
  // Broadcast TF
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
  
  if (impl_->publish_odom_tf_ ) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    RCLCPP_INFO(impl_->ros_node_->get_logger(),"Publishing odom transforms between [%s] and [%s]", 
      impl_->odometry_frame_.c_str(), impl_->robot_base_frame_.c_str());
  }

  auto covariance_x = _sdf->Get<double>("covariance_x", 0.00001).first;
  auto covariance_y = _sdf->Get<double>("covariance_y", 0.00001).first;
  auto covariance_yaw = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Set covariance
  impl_->odom_.pose.covariance[0] = covariance_x;
  impl_->odom_.pose.covariance[7] = covariance_y;
  impl_->odom_.pose.covariance[14] = 1000000000000.0;
  impl_->odom_.pose.covariance[21] = 1000000000000.0;
  impl_->odom_.pose.covariance[28] = 1000000000000.0;
  impl_->odom_.pose.covariance[35] = covariance_yaw;

  impl_->odom_.twist.covariance[0] = covariance_x;
  impl_->odom_.twist.covariance[7] = covariance_y;
  impl_->odom_.twist.covariance[14] = 1000000000000.0;
  impl_->odom_.twist.covariance[21] = 1000000000000.0;
  impl_->odom_.twist.covariance[28] = 1000000000000.0;
  impl_->odom_.twist.covariance[35] = covariance_yaw;

  // Set header
  impl_->odom_.header.frame_id = impl_->odometry_frame_;
  impl_->odom_.child_frame_id = impl_->robot_base_frame_;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosOmniDrivePrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  impl_->last_pose_ = impl_->model_->WorldPose();
  

  impl_->wheel_radius_ = _sdf->Get<double>("wheel_radius", 0.15).first;
  impl_->base_length_ = _sdf->Get<double>("base_length", 0.5).first;
  impl_->base_width_ = _sdf->Get<double>("base_width", 0.5).first;
  impl_->wheel_max_speed_ = _sdf->Get<double>("wheel_max_speed", 10.0).first;//rad/s
  impl_->wheel_acceleration_ = _sdf->Get<double>("wheel_acceleration", 5.0).first; //rad/s²
  impl_->max_torque_= _sdf->Get<double>("max_torque", 100.0).first; //N/m
  const char * wjoints[]={"front_left_joint","front_right_joint","rear_left_joint","rear_right_joint"};
  //the four wheels should be specified atherwise the wheel speed simulation is disabled
  impl_->wheel_joints_update=true;

  auto s_joint_conf = _sdf->Get<std::string>("joint_config", "1 1 1 1").first;
  std::istringstream ss(s_joint_conf);
  int sj[4];  ss>>sj[0]>>sj[1]>>sj[2]>>sj[3];
  for(int i=0;i<4;i++)impl_->joint_config_[i]=((sj[i]==-1)?-1.0:1.0);
  for(int i=0;i<4;i++){
    auto jelem = _sdf->GetElement(wjoints[i]); 
    if(jelem != nullptr){
      auto jname = jelem->Get<std::string>();
      auto joint = _model->GetJoint(jname);
      if (!joint) {
        RCLCPP_WARN(impl_->ros_node_->get_logger(),
        "[%s] not found", wjoints[i]);
        impl_->wheel_joints_update=false;
      }else {
        impl_->wheel_joints_[i]= joint;
              RCLCPP_INFO(impl_->ros_node_->get_logger(),
              "[%s] successfully linked to [%s]", wjoints[i],jname.c_str());
        joint->SetParam("fmax", 0, impl_->max_torque_);
      }
    }
  }
  
  impl_->publish_wheel_tf_ = false;
  if(impl_->wheel_joints_update)
    impl_->publish_wheel_tf_ = _sdf->Get<bool>("publish_wheel_tf", true).first;
  
  // Create the TF broadcaster if needed
  if (impl_->publish_wheel_tf_ && impl_->transform_broadcaster_ == nullptr) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
      RCLCPP_INFO(impl_->ros_node_->get_logger(), "Creating tf broadcaster for wheel tfs");
  }


}

void GazeboRosOmniDrive::Reset()
{
  impl_->last_update_time_ = impl_->world_->SimTime();
  impl_->target_cmd_vel_.linear.x = 0;
  impl_->target_cmd_vel_.linear.y = 0;
  impl_->target_cmd_vel_.angular.z = 0;
}
//Gazebo Ros Omnidirectional Private implementation 
//no locked because its called from a locked function 
geometry_msgs::msg::Twist  GazeboRosOmniDrivePrivate::ComputeRealTwist(double _dt)
{
  auto ret=target_cmd_vel_;
  if(!wheel_joints_update) return ret;
  double vm[4], max_dv=_dt*wheel_acceleration_,vx,vy,wz;
 
  for(int i =0;i<4;i++){
    vm[i]=joint_config_[i]*(wheel_joints_[i]->GetVelocity(0));
    auto dv=wheel_target_velocities[i]-vm[i];
    dv=fabs(dv)<max_dv?dv:(dv>0?max_dv:-max_dv);
    vm[i]+=dv;
    wheel_joints_[i]->SetParam("fmax", 0, max_torque_);
    wheel_joints_[i]->SetParam("vel", 0, joint_config_[i]*vm[i]);
  }
  FK(vm,vx,vy,wz);
  ret.linear.x=vx;
  ret.linear.y=vy;
  ret.angular.z=wz;
  return ret;

}
void GazeboRosOmniDrivePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();

  std::lock_guard<std::mutex> scoped_lock(lock_);

  if (seconds_since_last_update >= update_period_) {
    ignition::math::Pose3d pose = model_->WorldPose();

    // Compute Velocities even when the robot is not in a plane
    ignition::math::Quaternion<double> q;
    q.Euler(pose.Rot().Roll(), pose.Rot().Pitch(), pose.Rot().Yaw());

    
    auto real_twist = ComputeRealTwist(seconds_since_last_update);
    //real_twist = target_cmd_vel_; //debugging
    ignition::math::Vector3d lin_vel(real_twist.linear.x, real_twist.linear.y, 0.0);
    ignition::math::Vector3d rot_vel(0.0, 0.0, real_twist.angular.z);

    // Apply Gravity force to the component Z
    auto delta_pose = pose - last_pose_;
    double g_vel;
    ignition::math::Vector3d g = model_->GetWorld()->Gravity();

    if(abs(g.Z()) > 0.0) {
      g_vel = delta_pose.Z() / seconds_since_last_update + g.Z() * seconds_since_last_update;
      lin_vel.Z() = lin_vel.Z() + g_vel;
    }

    auto lin_vel_rot = q.RotateVector(lin_vel);
    auto rot_vel_rot = q.RotateVector(rot_vel);

    last_pose_ = pose;

    model_->SetLinearVel(lin_vel_rot);
    model_->SetAngularVel(rot_vel_rot);

    last_update_time_ = _info.simTime;
  }

  if (publish_odom_ || publish_odom_tf_) {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();

    if (seconds_since_last_publish < publish_period_) {
      return;
    }

    UpdateOdometry(_info.simTime);


    if (publish_wheel_tf_)PublishWheelsTf(_info.simTime);
    if (publish_odom_)odometry_pub_->publish(odom_);
    if (publish_odom_tf_) PublishOdometryTf(_info.simTime);

    last_publish_time_ = _info.simTime;
  }
}
void GazeboRosOmniDrivePrivate::IK(const double vx, const double vy, const double wz, double vm[])
{
const double L=(base_length_+base_width_)/2.0; 
const double inv_R=1.0/wheel_radius_;
vm[0]=inv_R*(vx - vy - L*wz);
vm[1]=inv_R*(vx + vy + L*wz);
vm[2]=inv_R*(vx + vy - L*wz);
vm[3]=inv_R*(vx - vy + L*wz);
}
void GazeboRosOmniDrivePrivate::FK(const double vm[], double &vx, double &vy, double &wz)
{
const double L=(base_length_+base_width_)/2.0; 
const double R_4=wheel_radius_/4.0;
vx=(vm[0]+vm[1]+vm[2]+vm[3])*R_4;
vy=(-vm[0]+vm[1]+vm[2]-vm[3])*R_4;
wz=(-vm[0]+vm[1]-vm[2]+vm[3])*R_4/L;
}
void GazeboRosOmniDrivePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  
  target_cmd_vel_ = *_msg;
  //compute the wheel target speeds:
  auto &vx = target_cmd_vel_.linear.x, &vy = target_cmd_vel_.linear.y, &wz= target_cmd_vel_.angular.z;
  double vm[4] , max=0;
  IK(vx,vy,wz,vm);
  for(auto v:vm)max=fabs(v)>max?fabs(v):max;
  double factor= max > wheel_max_speed_? max/wheel_max_speed_:1.0F;

  std::lock_guard<std::mutex> scoped_lock(lock_);
  for(int i=0;i<4;i++)wheel_target_velocities[i]=vm[i]*factor;
  
}

void GazeboRosOmniDrivePrivate::UpdateOdometry(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);

  // Get velocity in odom frame
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  auto linear = model_->WorldLinearVel();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  // Set timestamp
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
}


void GazeboRosOmniDrivePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = odometry_frame_;
  msg.child_frame_id = robot_base_frame_;
  msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);

  transform_broadcaster_->sendTransform(msg);
}

void GazeboRosOmniDrivePrivate::PublishWheelsTf(const gazebo::common::Time & _current_time)
{
  //RCLCPP_INFO(ros_node_->get_logger(), "Wheels TF published");
  for (auto j: wheel_joints_) {
    auto pose_wheel = j->GetChild()->RelativePose();

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
    msg.header.frame_id = j->GetParent()->GetName();
    msg.child_frame_id = j->GetChild()->GetName();
    msg.transform.translation = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(pose_wheel.Pos());
    msg.transform.rotation = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pose_wheel.Rot());

    transform_broadcaster_->sendTransform(msg);
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosOmniDrive)
}  // namespace gazebo_plugins
