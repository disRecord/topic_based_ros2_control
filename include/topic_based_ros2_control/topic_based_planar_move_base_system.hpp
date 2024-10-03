// Copyright 2022 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Jafar Abdi
   Desc: ros2_control system interface for topic based sim
*/

#pragma once

// C++
#include <memory>
#include <string>

// ROS
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace topic_based_ros2_control
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using geometry_msgs::msg::Pose2D;
using geometry_msgs::msg::Twist;
using geometry_msgs::msg::TransformStamped;
using tf2_msgs::msg::TFMessage;

class TopicBasedPlanarMoveBaseSystem : public hardware_interface::SystemInterface
{
	private:
		// node interafce
		rclcpp::Node::SharedPtr node_;
		rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
		rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

		// paramters holder
		bool fake_system_;
		bool publish_in_base_frame_;
		bool command_in_base_frame_;
		std::string base_frame_;
		std::string odom_frame_;
		
		// data buffers
		std::string planar_joint_name_;
		Pose2D state_velocity_;
		Pose2D state_position_;
		Pose2D command_velocity_;
		TFMessage tf_msg_;

	public:

		CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

		std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

		std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

		hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

		hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

	private:
		void odom_callback(const nav_msgs::msg::Odometry& odom);

};

}  // namespace topic_based_ros2_control
