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

/* Author: Oleg Goncharo */
#include <topic_based_ros2_control/topic_based_planar_move_base_system.hpp>

#include <cmath>
#include <exception>
#include <vector>
#include <sstream>

#include <angles/angles.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include <rclcpp/executors.hpp>
#include <tf2_ros/qos.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/quaternion.hpp>

namespace topic_based_ros2_control
{

template <typename T>
T parse_parameter_value(const std::string& name, const std::string& svalue, const std::string& msg, const std::function<bool(const T&)>& check_function) 
{
	T value;
	// parse parameter (suport bool, string and numeric types)
	std::istringstream is(svalue);
	is >> std::boolalpha >> value;
	// check if read succeed and all bytes was read
	if (is.fail() || !(is.eof() || is.tellg() == int(svalue.size()))) goto error;
	// check value
	if (!check_function(value)) goto error;
	// all ok, return reusl
	return value;
error:
	std::stringstream ss;
	ss << " invalid parameter " << name << "='" << svalue <<"': " << msg;
	throw std::runtime_error(std::move(ss).str());
}

template <>
bool parse_parameter_value<bool>(const std::string& name, const std::string& svalue, const std::string& msg, const std::function<bool(const bool&)>& check_function) 
{
	if ( (svalue == "true" || svalue == "True" || svalue == "1") && check_function(true) ) {
		return true;
	}
	else if ( (svalue == "false" || svalue == "False" || svalue == "0") && check_function(false) ) {
		return false;
	}
	else {
		std::stringstream ss;
		ss << " invalid boolean value " << name << "='" << svalue <<"': " << msg;
		throw std::runtime_error(std::move(ss).str());
	}	
}

template <typename T>
T get_parameter_required(const std::unordered_map<std::string, std::string> parameters, const std::string& name, const std::string& msg = "parse error.", std::function<bool(const T&)> check_function = [](const T&) { return true; }) 
{
	auto it = parameters.find(name);
	if (it == parameters.end()) {
		// not found
		throw std::runtime_error("parameter '" + name + "' is required.");
	}
	// parse it
	return parse_parameter_value<T>(name, it->second, msg, check_function);
}

template <typename T>
std::optional<T> get_parameter_optional(const std::unordered_map<std::string, std::string> parameters, const std::string& name, const std::string& msg = "parse error.", std::function<bool(const T&)> check_function = [](const T&) { return true; }) 
{
	auto it = parameters.find(name);
	if (it == parameters.end()) {
		// not found
		return std::nullopt;	
	}
	// parse it
	return parse_parameter_value<T>(name, it->second, msg, check_function);
}

template <typename T>
T get_parameter_default(const std::unordered_map<std::string, std::string> parameters, const std::string& name, const T& default_value, const std::string& msg = "parse error.", std::function<bool(const T&)> check_function = [](const T&) { return true; }) 
{
	auto it = parameters.find(name);
	if (it == parameters.end()) {
		// not found
		return default_value;
	}
	// parse it
	return parse_parameter_value<T>(name, it->second, msg, check_function);
}

static void assign_pose2d_field(const std::string& field, double value, Pose2D& pose) 
{
	if (field == "x") pose.x = value;
	else if (field == "y") pose.y = value;
	else if (field == "theta") pose.theta = value;
}

CallbackReturn TopicBasedPlanarMoveBaseSystem::on_init(const hardware_interface::HardwareInfo& info)
{
	if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
	{
		return CallbackReturn::ERROR;
	}

	try {
		//
		// Node initilization
		//

		// node-related parameters
		// node namespace
		std::string node_ns;
		if (auto it = info_.hardware_parameters.find("node_ns"); it != info_.hardware_parameters.end()) {
			node_ns = it->second; 
		}
		// node options
		std::vector<std::string> node_arguments{"--ros-args"};
		if (auto it = info_.hardware_parameters.find("node_arguments"); it != info_.hardware_parameters.end()) {
			std::istringstream argument_stream(it->second);
			std::string argument;
			while (std::getline(argument_stream, argument, ' ')) {
				node_arguments.push_back(argument);
			}
		}

		// initilize node
		rclcpp::NodeOptions options;
		options.arguments(node_arguments);
		node_ = rclcpp::Node::make_shared(info.name, node_ns, options);

		//
		// General parameters
		//

		// fake system
		fake_system_ = get_parameter_default<bool>(info.hardware_parameters, "fake_system", false);
		// frames
		base_frame_ = get_parameter_required<std::string>(info.hardware_parameters, "base_frame", "non-empty string is expected.", [](const std::string& s) { return !s.empty(); });
		odom_frame_ = get_parameter_required<std::string>(info.hardware_parameters, "odom_frame", "non-empty string is expected.", [](const std::string& s) { return !s.empty(); });
		// publish parameters
		publish_in_base_frame_ = get_parameter_default<bool>(info.hardware_parameters, "publish_in_base_frame", true, "false or true is expected.");
		command_in_base_frame_ = get_parameter_default<bool>(info.hardware_parameters, "command_in_base_frame", true, "false or true is expected.");
		bool publish_tf = get_parameter_default<bool>(info.hardware_parameters, "publish_tf", true, "false or true is expected.");

		//
		// Node interface
		//

		// publishers
		cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(get_parameter_default<std::string>(info.hardware_parameters, "cmd_vel_topic", "/cmd_vel"), rclcpp::QoS(1));
		// tf braodcaster
		if (publish_tf) {
			// publiser with tf QoS (copied from TransformBoadcaster)
			rclcpp::PublisherOptionsWithAllocator< std::allocator<void> > pub_options;
      		pub_options.qos_overriding_options = rclcpp::QosOverridingOptions{rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability, rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
			tf_pub_ = rclcpp::create_publisher<tf2_msgs::msg::TFMessage>(*node_, "/tf", tf2_ros::DynamicBroadcasterQoS(), pub_options);
			// message buffer
			tf_msg_.transforms.resize(1);
			TransformStamped& tf = tf_msg_.transforms.front();
			tf.header.frame_id = odom_frame_;
			tf.child_frame_id = base_frame_;
		}
		// subscriptions
		odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(get_parameter_default<std::string>(info.hardware_parameters, "odom_topic", "/odom"), rclcpp::SensorDataQoS(rclcpp::KeepLast(1)), std::bind(&TopicBasedPlanarMoveBaseSystem::odom_callback, this, std::placeholders::_1));

		//
		// Check joints
		//
		if (info.joints.size() != 3 || info.sensors.size() != 0 || info.transmissions.size() != 0 || info.gpios.size() != 0) {
			throw std::runtime_error("hardware configuration must contain only 3 joints and no sensors, gpios or transmissions.");
		}
		// get planar joint name
		auto pos = info.joints[0].name.find('/');
		if (pos == std::string::npos) {
			throw std::runtime_error("invalid joint name '" + info.joints[0].name + "'. Joint names must have format '<planar_joint_name>/<suffix>' where <suffix> is x, y or theta.");
		}
		planar_joint_name_ = info.joints[0].name.substr(0, pos);
		// check joints format
		std::set<std::string> allowed_suffixes{"x", "y", "theta"};
		for (const auto& joint : info.joints) {
			// name format check
			if (joint.name.size() <= planar_joint_name_.size() || joint.name.substr(0, pos) != planar_joint_name_ || joint.name[pos] != '/' || 
					allowed_suffixes.find(joint.name.substr(pos+1)) == allowed_suffixes.end()) {
				throw std::runtime_error("invalid joint name '" + joint.name + "'. Joint names must have format '<planar_joint_name>/<suffix>' where <suffix> is x, y or theta.");
			}
			allowed_suffixes.erase(joint.name.substr(pos+1));
			// states and command sizes
			if (joint.type != "joint" || joint.state_interfaces.size() != 2 || joint.command_interfaces.size() != 1) {
				throw std::runtime_error("invalid joint '" + joint.name + "': type must be 'joint', state and command interfaces sizes must be 2 and 1 respectively.");
			}
			// process states
			for (const auto& interface : joint.state_interfaces) {
				// check name
				Pose2D * state_buffer; 
				if (interface.name == "velocity") {
					state_buffer = &state_position_;
				}
				else if (interface.name == "position") {
					state_buffer = &state_velocity_;
				}
				else {
					std::runtime_error("joint '" + joint.name + "': state interface '" + interface.name + "' is not supported.");
				}
				// initial value
				if (!interface.initial_value.empty()) {
					assign_pose2d_field(joint.name, std::stoi(interface.initial_value), *state_buffer);
				}
			}
			// process commands
			// check name
			if (joint.command_interfaces[0].name != "velocity") {
				throw std::runtime_error("joint '" + joint.name + "': command interface '" + joint.command_interfaces[0].name + "' is not supported.");
			}
			// TODO: command limits
			// TODO: is_mimic, params
		}
		// initializaton is finished
		return CallbackReturn::SUCCESS;
	}
	catch (std::runtime_error& e) {
		// TODO: type of errors
		if (node_) {
			RCLCPP_ERROR(node_->get_logger(), e.what());
		}
		return CallbackReturn::ERROR;
	}
}

std::vector<hardware_interface::StateInterface> TopicBasedPlanarMoveBaseSystem::export_state_interfaces()
{
	std::vector<hardware_interface::StateInterface> state_interfaces;

	state_interfaces.emplace_back(planar_joint_name_ + "/x", "position", &state_position_.x);
	state_interfaces.emplace_back(planar_joint_name_ + "/y", "position", &state_position_.y);
	state_interfaces.emplace_back(planar_joint_name_ + "/theta", "position", &state_position_.theta);
	state_interfaces.emplace_back(planar_joint_name_ + "/x", "velocity", &state_velocity_.x);
	state_interfaces.emplace_back(planar_joint_name_ + "/y", "velocity", &state_velocity_.y);
	state_interfaces.emplace_back(planar_joint_name_ + "/theta", "velocity", &state_velocity_.theta);

	return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TopicBasedPlanarMoveBaseSystem::export_command_interfaces()
{
	std::vector<hardware_interface::CommandInterface> command_interfaces;

	command_interfaces.emplace_back(planar_joint_name_ + "/x", "velocity", &command_velocity_.x);
	command_interfaces.emplace_back(planar_joint_name_ + "/y", "velocity", &command_velocity_.y);
	command_interfaces.emplace_back(planar_joint_name_ + "/theta", "velocity", &command_velocity_.theta);

	return command_interfaces;
}

void TopicBasedPlanarMoveBaseSystem::odom_callback(const nav_msgs::msg::Odometry& odom) 
{
	// velocity
	state_velocity_.x = odom.twist.twist.linear.x;
	state_velocity_.y = odom.twist.twist.linear.y;
	state_velocity_.theta = odom.twist.twist.angular.z;
	// postion
	state_position_.x = odom.pose.pose.position.x;
	state_position_.y = odom.pose.pose.position.y;
	// orientation: extract roataion around z
	tf2::Quaternion quat;
	fromMsg(odom.pose.pose.orientation, quat);
	quat.setX(0.0); quat.setY(0.0);
	quat.normalize();
	tf2::Matrix3x3 R(quat);
	double r, p, y;
	R.getRPY(r, p, y);
	state_position_.theta = y;
	// publish tf
	if (tf_pub_ != nullptr) {
		// copy odometry data to tf message
		TransformStamped& tf = tf_msg_.transforms.front();
		tf.header.stamp = odom.header.stamp;
		tf.transform.translation.x = odom.pose.pose.position.x;
		tf.transform.translation.y = odom.pose.pose.position.y;
		tf.transform.translation.z = odom.pose.pose.position.z;
		tf.transform.rotation = odom.pose.pose.orientation;
		// publish it
		tf_pub_->publish(tf_msg_);
	}
}

hardware_interface::return_type TopicBasedPlanarMoveBaseSystem::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
	// process node events
	// get data from buffer
	if (rclcpp::ok())
	{
	  rclcpp::spin_some(node_);
	}
    // all is OK
	// TODO: check that messageas are actually being received
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type TopicBasedPlanarMoveBaseSystem::write(const rclcpp::Time& time, const rclcpp::Duration& period)
{ 
	if (!fake_system_) {
		// prepare twist msg
		Twist cmd_vel;
		if (command_in_base_frame_ == publish_in_base_frame_) {
			// convertion is not required
			cmd_vel.linear.x = command_velocity_.x;
			cmd_vel.linear.y = command_velocity_.y;
			cmd_vel.angular.z = command_velocity_.theta;
		}
		else {
			if (command_in_base_frame_) {
				// convet base_frame -> odom_frame
				double theta = state_position_.theta;
				cmd_vel.linear.x = std::cos(theta) * command_velocity_.x - std::sin(theta) * command_velocity_.y;
				cmd_vel.linear.y = std::sin(theta) * command_velocity_.x + std::cos(theta) * command_velocity_.y;
				cmd_vel.angular.z = command_velocity_.theta;
			}
			else {
				// convet odom_frame -> base_frame
				double theta = state_position_.theta;
				cmd_vel.linear.x = std::cos(theta) * command_velocity_.x + std::sin(theta) * command_velocity_.y;
				cmd_vel.linear.y = - std::sin(theta) * command_velocity_.x + std::cos(theta) * command_velocity_.y;
				cmd_vel.angular.z = command_velocity_.theta;
			}
		}
		// publish it
		if (rclcpp::ok())
		{
			cmd_vel_pub_->publish(cmd_vel);
		}
	}
	else {
		// update velocity
		if (!command_in_base_frame_) {
			// convertion is not necessary
			state_velocity_ = command_velocity_;
		}
		else {
			// convet base_frame -> odom_frame
			double theta = state_position_.theta;
			state_velocity_.x = std::cos(theta) * command_velocity_.x - std::sin(theta) * command_velocity_.y;
			state_velocity_.y = std::sin(theta) * command_velocity_.x + std::cos(theta) * command_velocity_.y;
			state_velocity_.theta = command_velocity_.theta;
		}
		// update position
		double dt = period.seconds();
		state_position_.x += state_velocity_.x * dt;
		state_position_.y += state_velocity_.y * dt;
		state_position_.theta = angles::normalize_angle(state_position_.theta + state_velocity_.theta * dt);
		// publish tf
		if (tf_pub_ != nullptr) {
			// copy position to tf message
			TransformStamped& tf = tf_msg_.transforms.front();
			tf.header.stamp = time;
			tf.transform.translation.x = state_position_.x;
			tf.transform.translation.y = state_position_.y;
			tf.transform.translation.z = 0.0;
			tf2::Quaternion quat;
			quat.setRPY(0.0, 0.0, state_position_.theta);
			tf.transform.rotation = tf2::toMsg(quat);
			// publish it
			tf_pub_->publish(tf_msg_);
		}
	}
	return hardware_interface::return_type::OK;
}
}  // end namespace topic_based_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(topic_based_ros2_control::TopicBasedPlanarMoveBaseSystem, hardware_interface::SystemInterface)
