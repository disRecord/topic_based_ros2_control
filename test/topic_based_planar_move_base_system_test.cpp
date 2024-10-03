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

#include <cmath>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_lifecycle/state.hpp>

TEST(TestTopicBasedPlanarMoveBaseSystem, load_system)
{
  const std::string urdf =
      R"(
<robot name="move_base_test">
  <link name="odom" />
  <link name="position_link1" />
  <joint name="position.x" type="prismatic">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0" />
    <parent link="odom"/>
    <child link="position_link1"/>
    <limit effort="1000" velocity="1000" lower="-1e6" upper="1e6" />
  </joint>
  <link name="position_link2" />
  <joint name="position.y" type="prismatic">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0" />
    <parent link="position_link1"/>
    <child link="position_link2"/>
    <limit effort="1000" velocity="1000" lower="-1e6" upper="1e6" />
  </joint>
  <link name="base_link" />
  <joint name="position.theta" type="continuous">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1" />
    <parent link="position_link2"/>
    <child link="base_link"/>
  </joint>

  <ros2_control name="TopicBasedDiffDriveSystem" type="system">
    <hardware>
      <plugin>topic_based_ros2_control/TopicBasedPlanarMoveBaseSystem</plugin>
      <param name="cmd_vel_topic">/cmd_vel</param>
      <param name="odom_topic">/odom</param>
      <param name="base_frame">base_link</param>
      <param name="odom_frame">odom</param>
    </hardware>
    <joint name="position/x">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="position/y">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="position/theta">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>
)";
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf, true, false));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  return RUN_ALL_TESTS();
}
