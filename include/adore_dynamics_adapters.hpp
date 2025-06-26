/********************************************************************************
 * Copyright (C) 2024-2025 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Marko Mizdrak
 ********************************************************************************/
#pragma once

#include "adore_dynamics_conversions.hpp"

#include <rclcpp/type_adapter.hpp>

namespace rclcpp
{

// Adapter for VehicleStateDynamic
template<>
struct TypeAdapter<adore::dynamics::VehicleStateDynamic, adore_ros2_msgs::msg::VehicleStateDynamic>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::dynamics::VehicleStateDynamic;
  using ros_message_type = adore_ros2_msgs::msg::VehicleStateDynamic;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::dynamics::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::dynamics::conversions::to_cpp_type( src );
  }
};

// Adapter for VehicleCommand
template<>
struct TypeAdapter<adore::dynamics::VehicleCommand, adore_ros2_msgs::msg::VehicleCommand>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::dynamics::VehicleCommand;
  using ros_message_type = adore_ros2_msgs::msg::VehicleCommand;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::dynamics::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::dynamics::conversions::to_cpp_type( src );
  }
};

// Adapter for VehicleInfo
template<>
struct TypeAdapter<adore::dynamics::VehicleInfo, adore_ros2_msgs::msg::VehicleInfo>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::dynamics::VehicleInfo;
  using ros_message_type = adore_ros2_msgs::msg::VehicleInfo;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::dynamics::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::dynamics::conversions::to_cpp_type( src );
  }
};

// Adapter for Trajectory
template<>
struct TypeAdapter<adore::dynamics::Trajectory, adore_ros2_msgs::msg::Trajectory>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::dynamics::Trajectory;
  using ros_message_type = adore_ros2_msgs::msg::Trajectory;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::dynamics::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::dynamics::conversions::to_cpp_type( src );
  }
};

// Adapter for TrafficParticipant
template<>
struct TypeAdapter<adore::dynamics::TrafficParticipant, adore_ros2_msgs::msg::TrafficParticipant>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::dynamics::TrafficParticipant;
  using ros_message_type = adore_ros2_msgs::msg::TrafficParticipant;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::dynamics::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::dynamics::conversions::to_cpp_type( src );
  }
};

// Adapter for TrafficParticipantSet
template<>
struct TypeAdapter<adore::dynamics::TrafficParticipantSet, adore_ros2_msgs::msg::TrafficParticipantSet>
{
  using is_specialized   = std::true_type;
  using custom_type      = adore::dynamics::TrafficParticipantSet;
  using ros_message_type = adore_ros2_msgs::msg::TrafficParticipantSet;

  static void
  convert_to_ros_message( const custom_type& src, ros_message_type& dst )
  {
    dst = adore::dynamics::conversions::to_ros_msg( src );
  }

  static void
  convert_to_custom( const ros_message_type& src, custom_type& dst )
  {
    dst = adore::dynamics::conversions::to_cpp_type( src );
  }
};

} // namespace rclcpp

namespace adore
{
using StateAdapter          = rclcpp::TypeAdapter<dynamics::VehicleStateDynamic, adore_ros2_msgs::msg::VehicleStateDynamic>;
using VehicleCommandAdapter = rclcpp::TypeAdapter<dynamics::VehicleCommand, adore_ros2_msgs::msg::VehicleCommand>;
using VehicleInfoAdapter    = rclcpp::TypeAdapter<dynamics::VehicleInfo, adore_ros2_msgs::msg::VehicleInfo>;
using TrajectoryAdapter     = rclcpp::TypeAdapter<dynamics::Trajectory, adore_ros2_msgs::msg::Trajectory>;
using ParticipantAdapter    = rclcpp::TypeAdapter<dynamics::TrafficParticipant, adore_ros2_msgs::msg::TrafficParticipant>;
using ParticipantSetAdapter = rclcpp::TypeAdapter<dynamics::TrafficParticipantSet, adore_ros2_msgs::msg::TrafficParticipantSet>;
} // namespace adore