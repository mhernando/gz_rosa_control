/*
 * Copyright 2013 Open Source Robotics Foundation
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
 * Original Author: Piyush Khandelwal
 * Date: 29 July 2013
 */

/*
 * Modified by: Dharmin B.
 * Date: 09 April 2020
 */

#include <boost/bind.hpp>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace gazebo
{

    class RopodControlPlugin : public ModelPlugin
    {

        public:
            RopodControlPlugin();
            ~RopodControlPlugin();
            void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

        protected:
            virtual void UpdateChild();
            virtual void FiniChild();

        private:
            void publishOdometry(double step_time);

            physics::ModelPtr parent_;
            event::ConnectionPtr update_connection_;

            boost::shared_ptr<ros::NodeHandle> nh_;
            ros::Publisher odometry_pub_;
            ros::Subscriber vel_sub_;
            boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;
            nav_msgs::Odometry odom_;
            std::string tf_prefix_;

            boost::mutex lock;

            std::string robot_namespace_;
            std::string command_topic_;
            std::string odometry_topic_;
            std::string odometry_frame_;
            std::string robot_base_frame_;
            double odometry_rate_;
            double cmd_vel_timeout_;

            // command velocity callback
            void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

            double x_;
            double y_;
            double rot_;
            common::Time last_odom_publish_time_;
            common::Time last_cmd_msg_time_;
            ignition::math::Pose3d last_odom_pose_;

    };

}

namespace gazebo
{

    RopodControlPlugin::RopodControlPlugin() {}

    RopodControlPlugin::~RopodControlPlugin()
    {
    }

    // Load the controller
    void RopodControlPlugin::Load(physics::ModelPtr parent,
            sdf::ElementPtr sdf)
    {

        parent_ = parent;

        /* Parse parameters */

        robot_namespace_ = "";
        if (!sdf->HasElement("robotNamespace"))
        {
            ROS_INFO_NAMED("ropod_control_plugin", "RopodControlPlugin missing <robotNamespace>, "
                    "defaults to \"%s\"", robot_namespace_.c_str());
        }
        else
        {
            robot_namespace_ =
                sdf->GetElement("robotNamespace")->Get<std::string>();
        }

        command_topic_ = "cmd_vel";
        if (!sdf->HasElement("commandTopic"))
        {
            ROS_WARN_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = %s) missing <commandTopic>, "
                    "defaults to \"%s\"",
                    robot_namespace_.c_str(), command_topic_.c_str());
        }
        else
        {
            command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
        }

        odometry_topic_ = "odom";
        if (!sdf->HasElement("odometryTopic"))
        {
            ROS_WARN_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = %s) missing <odometryTopic>, "
                    "defaults to \"%s\"",
                    robot_namespace_.c_str(), odometry_topic_.c_str());
        }
        else
        {
            odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
        }

        odometry_frame_ = "odom";
        if (!sdf->HasElement("odometryFrame"))
        {
            ROS_WARN_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = %s) missing <odometryFrame>, "
                    "defaults to \"%s\"",
                    robot_namespace_.c_str(), odometry_frame_.c_str());
        }
        else
        {
            odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
        }

        robot_base_frame_ = "base_link";
        if (!sdf->HasElement("robotBaseFrame"))
        {
            ROS_WARN_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = %s) missing <robotBaseFrame>, "
                    "defaults to \"%s\"",
                    robot_namespace_.c_str(), robot_base_frame_.c_str());
        }
        else
        {
            robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
        }

        odometry_rate_ = 20.0;
        if (!sdf->HasElement("odometryRate"))
        {
            ROS_WARN_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = %s) missing <odometryRate>, "
                    "defaults to %f",
                    robot_namespace_.c_str(), odometry_rate_);
        }
        else
        {
            odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
        }

        cmd_vel_timeout_ = 0.2;
        if (!sdf->HasElement("cmdVelTimeout"))
        {
            ROS_WARN_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = %s) missing <cmdVelTimeout>, "
                    "defaults to %f",
                    robot_namespace_.c_str(), cmd_vel_timeout_);
        }
        else
        {
            cmd_vel_timeout_ = sdf->GetElement("cmdVelTimeout")->Get<double>();
        }

#if GAZEBO_MAJOR_VERSION >= 8
        last_odom_publish_time_ = parent_->GetWorld()->SimTime();
        last_cmd_msg_time_ = parent_->GetWorld()->SimTime();
        last_odom_pose_ = parent_->WorldPose();
#else
        last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
        last_cmd_msg_time_ = parent_->GetWorld()->GetSimTime();
        last_odom_pose_ = parent_->GetWorldPose().Ign();
#endif
        x_ = 0;
        y_ = 0;
        rot_ = 0;

        // Ensure that ROS has been initialized and subscribe to cmd_vel
        if (!ros::isInitialized())
        {
            ROS_FATAL_STREAM_NAMED("ropod_control_plugin", "RopodControlPlugin (ns = " << robot_namespace_
                    << "). A ROS node for Gazebo has not been initialized, "
                    << "unable to load plugin. Load the Gazebo system plugin "
                    << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
            return;
        }
        nh_.reset(new ros::NodeHandle(robot_namespace_));

        tf_prefix_ = tf::getPrefixParam(*nh_);
        transform_broadcaster_.reset(new tf::TransformBroadcaster());

        vel_sub_ = nh_->subscribe<geometry_msgs::Twist>(command_topic_, 1,
                &RopodControlPlugin::cmdVelCallback, this);
        odometry_pub_ = nh_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

        // listen to the update event (broadcast every simulation iteration)
        update_connection_ =
            event::Events::ConnectWorldUpdateBegin(
                    boost::bind(&RopodControlPlugin::UpdateChild, this));

        ROS_INFO("Ropod plugin loaded");
    }

    // Update the controller
    void RopodControlPlugin::UpdateChild()
    {
        boost::mutex::scoped_lock scoped_lock(lock);
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = parent_->WorldPose();
        common::Time current_time = parent_->GetWorld()->SimTime();
        double linear_vel_z = parent_->WorldLinearVel().Z();
        double angular_vel_x = parent_->WorldAngularVel().X();
        double angular_vel_y = parent_->WorldAngularVel().Y();
#else
        ignition::math::Pose3d pose = parent_->GetWorldPose().Ign();
        common::Time current_time = parent_->GetWorld()->GetSimTime();
        double linear_vel_z = parent_->GetWorldLinearVel().z;
        double angular_vel_x = parent_->GetWorldAngularVel().x;
        double angular_vel_y = parent_->GetWorldAngularVel().y;
#endif
        float yaw = pose.Rot().Yaw();
        double seconds_since_last_cmd_msg =
            (current_time - last_cmd_msg_time_).Double();
        if (seconds_since_last_cmd_msg > cmd_vel_timeout_)
        {
            x_ = 0.0;
            y_ = 0.0;
            rot_ = 0.0;
        }
        parent_->SetLinearVel(ignition::math::Vector3d(
                    x_ * cosf(yaw) - y_ * sinf(yaw),
                    y_ * cosf(yaw) + x_ * sinf(yaw),
                    linear_vel_z));
        parent_->SetAngularVel(ignition::math::Vector3d(angular_vel_x, angular_vel_y, rot_));
        if (odometry_rate_ > 0.0) {
            double seconds_since_last_update =
                (current_time - last_odom_publish_time_).Double();
            if (seconds_since_last_update > (1.0 / odometry_rate_)) {
                publishOdometry(seconds_since_last_update);
                last_odom_publish_time_ = current_time;
            }
        }
    }

    // Finalize the controller
    void RopodControlPlugin::FiniChild()
    {
        nh_->shutdown();
    }

    void RopodControlPlugin::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
    {
        boost::mutex::scoped_lock scoped_lock(lock);
        rot_ = cmd_msg->angular.z;
        x_ = cmd_msg->linear.x;
        y_ = cmd_msg->linear.y;
#if GAZEBO_MAJOR_VERSION >= 8
        last_cmd_msg_time_ = parent_->GetWorld()->SimTime();
#else
        last_cmd_msg_time_ = parent_->GetWorld()->GetSimTime();
#endif
    }

    void RopodControlPlugin::publishOdometry(double step_time)
    {

        ros::Time current_time = ros::Time::now();
        std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
        std::string base_link_frame =
            tf::resolve(tf_prefix_, robot_base_frame_);

        // getting data for base_link to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Pose3d pose = this->parent_->WorldPose();
#else
        ignition::math::Pose3d pose = this->parent_->GetWorldPose().Ign();
#endif

        tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        tf::Vector3    vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

        tf::Transform base_link_to_odom(qt, vt);
        transform_broadcaster_->sendTransform(
                tf::StampedTransform(base_link_to_odom, current_time,
                                     odom_frame, base_link_frame));

        // publish odom topic
        odom_.pose.pose.position.x = pose.Pos().X();
        odom_.pose.pose.position.y = pose.Pos().Y();

        odom_.pose.pose.orientation.x = pose.Rot().X();
        odom_.pose.pose.orientation.y = pose.Rot().Y();
        odom_.pose.pose.orientation.z = pose.Rot().Z();
        odom_.pose.pose.orientation.w = pose.Rot().W();

        // get velocity in /odom frame
        ignition::math::Vector3d linear;
        linear.X() = (pose.Pos().X() - last_odom_pose_.Pos().X()) / step_time;
        linear.Y() = (pose.Pos().Y() - last_odom_pose_.Pos().Y()) / step_time;
        if (rot_ > M_PI / step_time)
        {
            // we cannot calculate the angular velocity correctly
            odom_.twist.twist.angular.z = rot_;
        }
        else
        {
            float last_yaw = last_odom_pose_.Rot().Yaw();
            float current_yaw = pose.Rot().Yaw();
            while (current_yaw < last_yaw - M_PI) current_yaw += 2 * M_PI;
            while (current_yaw > last_yaw + M_PI) current_yaw -= 2 * M_PI;
            float angular_diff = current_yaw - last_yaw;
            odom_.twist.twist.angular.z = angular_diff / step_time;
        }
        last_odom_pose_ = pose;

        // convert velocity to child_frame_id (aka base_link)
        float yaw = pose.Rot().Yaw();
        odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
        odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

        odom_.header.stamp = current_time;
        odom_.header.frame_id = odom_frame;
        odom_.child_frame_id = base_link_frame;

        odometry_pub_.publish(odom_);
    }

    GZ_REGISTER_MODEL_PLUGIN(RopodControlPlugin)
}
