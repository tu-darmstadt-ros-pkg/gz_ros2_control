// Copyright 2026 Aljoscha Schmidt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gz_ros2_control/gz_system.hpp"

namespace components = gz::sim::components;
using hardware_interface::return_type;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr unsigned int kUpdateRate = 100;
constexpr double kGain = 0.1;
const rclcpp::Time kTime;
const rclcpp::Duration kPeriod(0, 0);

class GazeboSimSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("gz_system_test");
    node_->declare_parameter("hold_joints", true);
    node_->declare_parameter("position_proportional_gain", kGain);

    hardware_interface::HardwareInfo info;
    info.name = "gz_system";
    info.type = "system";
    info.joints = {joint("position_joint", "position"), joint("velocity_joint", "velocity"),
      joint("effort_joint", "effort"), joint("mimic_joint", "")};
    info.mimic_joints.push_back({3, 0, 2.0, 0.0});

    std::map<std::string, gz::sim::Entity> enabled_joints;
    for (const auto & component : info.joints) {
      const auto entity = ecm_.CreateEntity();
      ecm_.CreateComponent(entity, components::Joint());
      ecm_.CreateComponent(entity, components::Name(component.name));
      ecm_.CreateComponent(entity, components::JointType(sdf::JointType::REVOLUTE));
      ecm_.CreateComponent(entity, components::JointAxis(sdf::JointAxis()));
      ecm_.CreateComponent(entity, components::JointPosition({0.0}));
      ecm_.CreateComponent(entity, components::JointVelocity({0.0}));
      ecm_.CreateComponent(entity, components::JointTransmittedWrench());
      enabled_joints[component.name] = entity;
    }

    ASSERT_TRUE(system_.initSim(node_, enabled_joints, info, ecm_, kUpdateRate));
    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = info;
    ASSERT_EQ(system_.on_init(params), gz_ros2_control::CallbackReturn::SUCCESS);
    for (const auto & command : system_.on_export_command_interfaces()) {
      commands_[command->get_name()] = command;
    }
    for (const auto & [name, entity] : enabled_joints) {
      entities_[name] = entity;
    }
    ASSERT_EQ(
      system_.perform_command_mode_switch(
        {"position_joint/position", "velocity_joint/velocity", "effort_joint/effort"}, {}),
      return_type::OK);
  }

  static hardware_interface::ComponentInfo joint(
    const std::string & name, const std::string & command)
  {
    hardware_interface::ComponentInfo info;
    info.name = name;
    info.type = "joint";
    hardware_interface::InterfaceInfo interface;
    if (!command.empty()) {
      interface.name = command;
      info.command_interfaces.push_back(interface);
    }
    interface.name = "position";
    info.state_interfaces.push_back(interface);
    return info;
  }

  void command(const std::string & interface, double value)
  {
    ASSERT_TRUE(commands_.at(interface)->set_value(value, true));
  }

  void set_position(const std::string & joint, double position)
  {
    ecm_.SetComponentData<components::JointPosition>(entities_.at(joint), {position});
  }

  void step()
  {
    ASSERT_EQ(system_.read(kTime, kPeriod), return_type::OK);
    ASSERT_EQ(system_.write(kTime, kPeriod), return_type::OK);
    system_.applyCommands(ecm_);
  }

  double velocity_cmd(const std::string & joint)
  {
    const auto * cmd = ecm_.Component<components::JointVelocityCmd>(entities_.at(joint));
    return cmd ? cmd->Data().at(0) : kNaN;
  }

  double effort_cmd(const std::string & joint)
  {
    const auto * cmd = ecm_.Component<components::JointForceCmd>(entities_.at(joint));
    return cmd ? cmd->Data().at(0) : kNaN;
  }

  rclcpp::Node::SharedPtr node_;
  gz::sim::EntityComponentManager ecm_;
  gz_ros2_control::GazeboSimSystem system_;
  std::map<std::string, hardware_interface::CommandInterface::SharedPtr> commands_;
  std::map<std::string, gz::sim::Entity> entities_;
};

TEST_F(GazeboSimSystemTest, WriteLeavesTheEcmToApplyCommands)
{
  command("velocity_joint/velocity", 0.5);
  ASSERT_EQ(system_.write(kTime, kPeriod), return_type::OK);
  EXPECT_EQ(ecm_.Component<components::JointVelocityCmd>(entities_.at("velocity_joint")), nullptr);

  system_.applyCommands(ecm_);
  EXPECT_DOUBLE_EQ(velocity_cmd("velocity_joint"), 0.5);
}

TEST_F(GazeboSimSystemTest, ApplyCommandsRepeatsTheLastWrite)
{
  command("velocity_joint/velocity", 0.5);
  step();
  ecm_.SetComponentData<components::JointVelocityCmd>(entities_.at("velocity_joint"), {0.0});
  system_.applyCommands(ecm_);
  EXPECT_DOUBLE_EQ(velocity_cmd("velocity_joint"), 0.5);
}

TEST_F(GazeboSimSystemTest, NanVelocityStopsTheJoint)
{
  command("velocity_joint/velocity", 0.5);
  step();
  command("velocity_joint/velocity", kNaN);
  step();
  EXPECT_EQ(velocity_cmd("velocity_joint"), 0.0);
}

TEST_F(GazeboSimSystemTest, NanPositionHoldsThePositionLatchedAtTheFirstNan)
{
  set_position("position_joint", 1.0);
  command("position_joint/position", kNaN);
  step();
  EXPECT_EQ(velocity_cmd("position_joint"), 0.0);

  set_position("position_joint", 1.2);
  step();
  EXPECT_NEAR(velocity_cmd("position_joint"), -kGain * 0.2 * kUpdateRate, 1e-9);

  command("position_joint/position", 2.0);
  step();
  EXPECT_NEAR(velocity_cmd("position_joint"), kGain * 0.8 * kUpdateRate, 1e-9);

  command("position_joint/position", kNaN);
  step();
  EXPECT_EQ(velocity_cmd("position_joint"), 0.0);
}

TEST_F(GazeboSimSystemTest, NanEffortKeepsTheLastEffort)
{
  command("effort_joint/effort", kNaN);
  step();
  EXPECT_EQ(effort_cmd("effort_joint"), 0.0);

  command("effort_joint/effort", 3.0);
  step();
  command("effort_joint/effort", kNaN);
  step();
  EXPECT_EQ(effort_cmd("effort_joint"), 3.0);
}
