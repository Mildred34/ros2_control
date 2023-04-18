// Copyright 2020 PAL Robotics S.L.
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

#ifndef TRANSMISSION_INTERFACE__ABDUCTION_TRANSMISSION_HPP_
#define TRANSMISSION_INTERFACE__ABDUCTION_TRANSMISSION_HPP_

#include <algorithm>
#include <cassert>
#include <string>
#include <vector>

#include "transmission_interface/accessor.hpp"
#include "transmission_interface/exception.hpp"
#include "transmission_interface/transmission.hpp"

namespace transmission_interface
{
class AbductionTransmission : public Transmission
{
public:
  /**
   * \param actuator_reduction Reduction ratio of actuators.
   * \param joint_reduction    Reduction ratio of joints.
   * \param joint_offset       Joint position offset used in the position mappings.
   * \pre Nonzero actuator reduction values.
   */
  explicit AbductionTransmission(
    const std::vector<double> & actuator_reduction, const std::vector<double> & joint_reduction,
    const std::vector<double> & joint_offset = std::vector<double>(2, 0.0));

  /// Set up the data the transmission operates on.
  /**
   * \param[in] joint_handles Vector of interface handles of one joint
   * \param[in] actuator_handles Vector of interface handles of one actuator
   * \pre Vectors are nonzero.
   * \pre Joint handles share the same joint name and actuator handles share the same actuator name.
   */
  void configure(
    const std::vector<JointHandle> & joint_handles,
    const std::vector<ActuatorHandle> & actuator_handles) override;

  /// Transform variables from actuator to joint space.
  /**
   *  This method operates on the handles provided when configuring the transmission.
   *  To call this method it is not required that all supported interface types are provided, e.g. one can supply only velocity handles
   */
  void actuator_to_joint() override;

  /// Transform variables from joint to actuator space.
  /**
   *  This method operates on the handles provided when configuring the transmission.
   *  To call this method it is not required that all supported interface types are provided, e.g. one can supply only velocity handles
   */
  void joint_to_actuator() override;

  std::size_t num_actuators() const override { return 1; }
  std::size_t num_joints() const override { return 2; }

  const std::vector<double> & get_actuator_reduction() const { return actuator_reduction_; }
  const std::vector<double> & get_joint_reduction() const { return joint_reduction_; }
  const std::vector<double> & get_joint_offset() const { return joint_offset_; }

  /// Get human-friendly report of handles
  std::string get_handles_info() const;

protected:
  std::vector<double> actuator_reduction_;
  std::vector<double> joint_reduction_;
  std::vector<double> joint_offset_;

  std::vector<JointHandle> joint_position_;
  std::vector<JointHandle> joint_velocity_;
  std::vector<JointHandle> joint_effort_;

  std::vector<ActuatorHandle> actuator_position_;
  std::vector<ActuatorHandle> actuator_velocity_;
  std::vector<ActuatorHandle> actuator_effort_;
};

inline AbductionTransmission::AbductionTransmission(
  const std::vector<double> & actuator_reduction, const std::vector<double> & joint_reduction,
  const std::vector<double> & joint_offset)
: actuator_reduction_(actuator_reduction),
  joint_reduction_(joint_reduction),
  joint_offset_(joint_offset)
{
  if (
    num_actuators() != actuator_reduction_.size() || num_joints() != joint_reduction_.size() ||
    num_joints() != joint_offset_.size())
  {
    throw Exception("Reduction and offset vectors must have size 2.");
  }
  if (
    0.0 == actuator_reduction_[0] || 0.0 == joint_reduction_[0] ||
    0.0 == joint_reduction_[1])
  {
    throw Exception("Transmission reduction ratios cannot be zero.");
  }
}

inline void AbductionTransmission::configure(
  const std::vector<JointHandle> & joint_handles,
  const std::vector<ActuatorHandle> & actuator_handles)
{
  if (joint_handles.empty())
  {
    throw Exception("No joint handles were passed in");
  }

  if (actuator_handles.empty())
  {
    throw Exception("No actuator handles were passed in");
  }

  const auto joint_names = get_names(joint_handles);
  if (joint_names.size() != 2)
  {
    throw Exception(
      "There should be exactly two unique joint names but was given " + to_string(joint_names));
  }

  const auto actuator_names = get_names(actuator_handles);
  if (actuator_names.size() != 1)
  {
    throw Exception(
      "There should be exactly one unique actuator name but was given " +
      to_string(actuator_names));
  }

  joint_position_ =
    get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_POSITION);
  joint_velocity_ =
    get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_VELOCITY);
  joint_effort_ = get_ordered_handles(joint_handles, joint_names, hardware_interface::HW_IF_EFFORT);

  if (joint_position_.size() != 2 && joint_velocity_.size() != 2 && joint_effort_.size() != 2)
  {
    throw Exception("Not enough valid or required joint handles were presented.");
  }

  actuator_position_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_POSITION);
  actuator_velocity_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_VELOCITY);
  actuator_effort_ =
    get_ordered_handles(actuator_handles, actuator_names, hardware_interface::HW_IF_EFFORT);

  if (
    actuator_position_.size() != 1 && actuator_velocity_.size() != 1 &&
    actuator_effort_.size() != 1)
  {
    throw Exception(
      "Not enough valid or required actuator handles were presented. \n" + get_handles_info());
  }

}

inline void AbductionTransmission::actuator_to_joint()
{

  const auto & ar = actuator_reduction_;
  const auto & jr = joint_reduction_;

  // position
  auto & act_pos = actuator_position_;
  auto & joint_pos = joint_position_;

  if (act_pos.size() == num_actuators() && joint_pos.size() == num_joints())
  {
    assert(act_pos[0] && joint_pos[0] && joint_pos[1]);

    joint_pos[0].set_value(-act_pos[0].get_value());
    joint_pos[1].set_value(act_pos[0].get_value());

  }

  // velocity
  auto & act_vel = actuator_velocity_;
  auto & joint_vel = joint_velocity_;
  if (act_vel.size() == num_actuators() && joint_vel.size() == num_joints())
  {
    assert(act_vel[0] && joint_vel[0] && joint_vel[1]);

    joint_vel[0].set_value(-act_vel[0].get_value());
    joint_vel[1].set_value(act_vel[0].get_value());

  }

  // effort
  auto & act_eff = actuator_effort_;
  auto & joint_eff = joint_effort_;
  if (act_eff.size() == num_actuators() && joint_eff.size() == num_joints())
  {
    assert(act_eff[0] && joint_eff[0] && joint_eff[1]);

    joint_eff[0].set_value(act_eff[0].get_value() / (ar[0]*jr[0]));
    joint_eff[1].set_value(act_eff[1].get_value() / (ar[0]*jr[1]));
  }

}

inline void AbductionTransmission::joint_to_actuator()
{
  const auto & ar = actuator_reduction_;
  const auto & jr = joint_reduction_;

  // position
  auto & act_pos = actuator_position_;
  auto & joint_pos = joint_position_;

  if (act_pos.size() == num_actuators() && joint_pos.size() == num_joints())
  {
    assert(act_pos[0] && joint_pos[0] && joint_pos[1]);

    act_pos[0].set_value(joint_pos[0].get_value() - joint_offset_[0]);
  }

  // velocity
  auto & act_vel = actuator_velocity_;
  auto & joint_vel = joint_velocity_;
  if (act_vel.size() == num_actuators() && joint_vel.size() == num_joints())
  {
    assert(act_vel[0] && joint_vel[0] && joint_vel[1]);

    act_vel[0].set_value(joint_vel[0].get_value());

  }

  // effort
  auto & act_eff = actuator_effort_;
  auto & joint_eff = joint_effort_;
  if (act_eff.size() == num_actuators() && joint_eff.size() == num_joints())
  {
    assert(act_eff[0] && joint_eff[0] && joint_eff[1]);

    act_eff[0].set_value((joint_eff[0].get_value() - joint_eff[1].get_value()) * (ar[0] * jr[0]));
  }
}

std::string AbductionTransmission::get_handles_info() const
{
  return std::string("Got the following handles:\n") +
         "Joint position: " + to_string(get_names(joint_position_)) +
         ", Actuator position: " + to_string(get_names(actuator_position_)) + "\n" +
         "Joint velocity: " + to_string(get_names(joint_velocity_)) +
         ", Actuator velocity: " + to_string(get_names(actuator_velocity_)) + "\n" +
         "Joint effort: " + to_string(get_names(joint_effort_)) +
         ", Actuator effort: " + to_string(get_names(actuator_effort_));
}

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__ABDUCTION_TRANSMISSION_HPP_
