#include <cmath>
#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "adore_dynamics_conversions.hpp"
#include "adore_math/angles.h"
#include <adore_ros2_msgs/msg/gear_state.hpp>
#include <adore_ros2_msgs/msg/physical_vehicle_parameters.hpp>
#include <adore_ros2_msgs/msg/route.hpp>
#include <adore_ros2_msgs/msg/traffic_participant.hpp>
#include <adore_ros2_msgs/msg/traffic_participant_set.hpp>
#include <adore_ros2_msgs/msg/trajectory.hpp>
#include <adore_ros2_msgs/msg/trajectory_transpose.hpp>
#include <adore_ros2_msgs/msg/vehicle_command.hpp>
#include <adore_ros2_msgs/msg/vehicle_info.hpp>
#include <adore_ros2_msgs/msg/vehicle_state_dynamic.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{

namespace dyn          = adore::dynamics;
namespace conv         = adore::dynamics::conversions;
constexpr double k_eps = 1e-9;

void
expect_vehicle_state_equal( const dyn::VehicleStateDynamic& a, const dyn::VehicleStateDynamic& b )
{
  EXPECT_NEAR( a.time, b.time, k_eps );
  EXPECT_NEAR( a.x, b.x, k_eps );
  EXPECT_NEAR( a.y, b.y, k_eps );
  EXPECT_NEAR( a.z, b.z, k_eps );
  EXPECT_NEAR( a.vx, b.vx, k_eps );
  EXPECT_NEAR( a.vy, b.vy, k_eps );
  EXPECT_NEAR( a.yaw_angle, b.yaw_angle, k_eps );
  EXPECT_NEAR( a.yaw_rate, b.yaw_rate, k_eps );
  EXPECT_NEAR( a.ax, b.ax, k_eps );
  EXPECT_NEAR( a.ay, b.ay, k_eps );
  EXPECT_NEAR( a.steering_angle, b.steering_angle, k_eps );
  EXPECT_NEAR( a.steering_rate, b.steering_rate, k_eps );
}

void
expect_physical_params_equal( const dyn::PhysicalVehicleParameters& a, const dyn::PhysicalVehicleParameters& b )
{
  EXPECT_NEAR( a.cog_to_front_axle, b.cog_to_front_axle, k_eps );
  EXPECT_NEAR( a.rear_axle_to_cog, b.rear_axle_to_cog, k_eps );
  EXPECT_NEAR( a.wheelbase, b.wheelbase, k_eps );
  EXPECT_NEAR( a.front_axle_to_front_border, b.front_axle_to_front_border, k_eps );
  EXPECT_NEAR( a.rear_border_to_rear_axle, b.rear_border_to_rear_axle, k_eps );
  EXPECT_NEAR( a.mass, b.mass, k_eps );
  EXPECT_NEAR( a.friction_coefficient, b.friction_coefficient, k_eps );
  EXPECT_NEAR( a.cog_height, b.cog_height, k_eps );
  EXPECT_NEAR( a.front_tire_stiffness, b.front_tire_stiffness, k_eps );
  EXPECT_NEAR( a.rear_tire_stiffness, b.rear_tire_stiffness, k_eps );
  EXPECT_NEAR( a.rotational_inertia_div_mass, b.rotational_inertia_div_mass, k_eps );
  EXPECT_NEAR( a.front_track_width, b.front_track_width, k_eps );
  EXPECT_NEAR( a.rear_track_width, b.rear_track_width, k_eps );
  EXPECT_NEAR( a.steering_ratio, b.steering_ratio, k_eps );
  EXPECT_NEAR( a.steering_angle_max, b.steering_angle_max, k_eps );
  EXPECT_NEAR( a.steering_angle_min, b.steering_angle_min, k_eps );
  EXPECT_NEAR( a.acceleration_max, b.acceleration_max, k_eps );
  EXPECT_NEAR( a.acceleration_min, b.acceleration_min, k_eps );
  EXPECT_NEAR( a.cornering_stiffness, b.cornering_stiffness, k_eps );
  EXPECT_NEAR( a.brake_balance_front, b.brake_balance_front, k_eps );
  EXPECT_NEAR( a.acceleration_balance_front, b.acceleration_balance_front, k_eps );
  EXPECT_NEAR( a.body_width, b.body_width, k_eps );
  EXPECT_NEAR( a.body_length, b.body_length, k_eps );
  EXPECT_NEAR( a.body_height, b.body_height, k_eps );
}

dyn::VehicleStateDynamic
make_sample_vehicle_state()
{
  dyn::VehicleStateDynamic state;
  state.time           = 123.456;
  state.x              = 1.0;
  state.y              = 2.0;
  state.z              = 0.5;
  state.vx             = 3.0;
  state.vy             = 4.0;
  state.yaw_angle      = 0.7;
  state.yaw_rate       = -0.2;
  state.ax             = 0.1;
  state.ay             = -0.3;
  state.steering_angle = 0.05;
  state.steering_rate  = -0.01;
  state.frame_id       = "base_link";
  return state;
}

dyn::PhysicalVehicleParameters
make_sample_physical_params()
{
  dyn::PhysicalVehicleParameters p;
  p.cog_to_front_axle           = 1.0;
  p.rear_axle_to_cog            = 2.0;
  p.wheelbase                   = 3.0;
  p.front_axle_to_front_border  = 4.0;
  p.rear_border_to_rear_axle    = 5.0;
  p.mass                        = 6.0;
  p.friction_coefficient        = 7.0;
  p.cog_height                  = 8.0;
  p.front_tire_stiffness        = 9.0;
  p.rear_tire_stiffness         = 10.0;
  p.rotational_inertia_div_mass = 11.0;
  p.front_track_width           = 12.0;
  p.rear_track_width            = 13.0;
  p.steering_ratio              = 14.0;
  p.steering_angle_max          = 15.0;
  p.steering_angle_min          = 16.0;
  p.acceleration_max            = 17.0;
  p.acceleration_min            = 18.0;
  p.cornering_stiffness         = 19.0;
  p.brake_balance_front         = 20.0;
  p.acceleration_balance_front  = 21.0;
  p.body_width                  = 22.0;
  p.body_length                 = 23.0;
  p.body_height                 = 24.0;
  return p;
}

adore_ros2_msgs::msg::Route
make_sample_route_msg_for_participant()
{
  adore_ros2_msgs::msg::Route route_msg;

  // Simple bounding box
  route_msg.map.x_min = -1.0;
  route_msg.map.x_max = 1.0;
  route_msg.map.y_min = -2.0;
  route_msg.map.y_max = 2.0;

  // One section
  adore_ros2_msgs::msg::RouteSection sec;
  sec.start_s = 0.0;
  sec.end_s   = 10.0;
  sec.route_s = 0.0;
  sec.lane_id = 1;
  route_msg.sections.push_back( sec );

  // One center point (ensures route is considered "present")
  adore_ros2_msgs::msg::MapPoint cp;
  cp.x         = 0.0;
  cp.y         = 0.0;
  cp.s         = 5.0;
  cp.parent_id = 1;
  cp.max_speed = 13.0;
  route_msg.center_points.push_back( cp );

  // Start / goal
  route_msg.start.x = -0.5;
  route_msg.start.y = 0.0;
  route_msg.goal.x  = 0.5;
  route_msg.goal.y  = 0.0;

  return route_msg;
}

} // namespace

// ---------------------- basic conversions ----------------------

TEST( AdoreDynamicsConversions, vehicle_command_round_trip )
{
  dyn::VehicleCommand cmd;
  cmd.steering_angle = 0.3;
  cmd.acceleration   = -1.5;

  auto msg      = conv::to_ros_msg( cmd );
  auto cmd_back = conv::to_cpp_type( msg );

  EXPECT_DOUBLE_EQ( cmd.steering_angle, msg.steering_angle );
  EXPECT_DOUBLE_EQ( cmd.acceleration, msg.acceleration );
  EXPECT_DOUBLE_EQ( cmd.steering_angle, cmd_back.steering_angle );
  EXPECT_DOUBLE_EQ( cmd.acceleration, cmd_back.acceleration );
}

TEST( AdoreDynamicsConversions, vehicle_state_round_trip )
{
  auto state = make_sample_vehicle_state();

  auto msg        = conv::to_ros_msg( state );
  auto state_back = conv::to_cpp_type( msg );

  expect_vehicle_state_equal( state, state_back );
}

TEST( AdoreDynamicsConversions, update_state_with_odometry )
{
  dyn::VehicleStateDynamic state;

  nav_msgs::msg::Odometry odom;
  odom.pose.pose.position.x = 10.0;
  odom.pose.pose.position.y = 20.0;
  odom.pose.pose.position.z = 1.5;

  double          yaw = 0.9;
  tf2::Quaternion q;
  q.setRPY( 0.0, 0.0, yaw );
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();

  odom.twist.twist.linear.x  = 4.0;
  odom.twist.twist.linear.y  = -2.0;
  odom.twist.twist.angular.z = 0.25;

  odom.header.stamp.sec     = 42;
  odom.header.stamp.nanosec = 100000000; // 0.1 s

  conv::update_state_with_odometry( state, odom );

  EXPECT_NEAR( state.x, odom.pose.pose.position.x, k_eps );
  EXPECT_NEAR( state.y, odom.pose.pose.position.y, k_eps );
  EXPECT_NEAR( state.z, odom.pose.pose.position.z, k_eps );
  EXPECT_NEAR( state.vx, odom.twist.twist.linear.x, k_eps );
  EXPECT_NEAR( state.vy, odom.twist.twist.linear.y, k_eps );
  EXPECT_NEAR( state.yaw_rate, odom.twist.twist.angular.z, k_eps );
  EXPECT_NEAR( state.yaw_angle, yaw, 1e-6 );
  EXPECT_NEAR( state.time, 42.1, 1e-9 );
}

TEST( AdoreDynamicsConversions, dynamic_state_to_odometry_msg )
{
  auto state = make_sample_vehicle_state();

  auto clock = std::make_shared<rclcpp::Clock>( RCL_SYSTEM_TIME );
  auto odom  = conv::dynamic_state_to_odometry_msg( state, clock );

  EXPECT_NEAR( odom.pose.pose.position.x, state.x, k_eps );
  EXPECT_NEAR( odom.pose.pose.position.y, state.y, k_eps );
  EXPECT_NEAR( odom.pose.pose.position.z, state.z, k_eps );

  EXPECT_NEAR( odom.twist.twist.linear.x, state.vx, k_eps );
  EXPECT_NEAR( odom.twist.twist.linear.y, state.vy, k_eps );
  EXPECT_NEAR( odom.twist.twist.angular.z, state.yaw_rate, k_eps );
  EXPECT_NEAR( odom.twist.twist.angular.x, 0.0, k_eps );
  EXPECT_NEAR( odom.twist.twist.angular.y, 0.0, k_eps );

  double yaw = adore::math::get_yaw( odom.pose.pose.orientation );
  EXPECT_NEAR( yaw, state.yaw_angle, 1e-6 );
}

TEST( AdoreDynamicsConversions, vehicle_state_to_transform )
{
  auto state = make_sample_vehicle_state();

  rclcpp::Time      timestamp( 555.0 );
  const std::string child_frame = "base_link";

  auto tf_msg = conv::vehicle_state_to_transform( state, timestamp, child_frame );

  EXPECT_EQ( tf_msg.header.frame_id, "world" );
  EXPECT_EQ( tf_msg.child_frame_id, child_frame );

  EXPECT_NEAR( tf_msg.transform.translation.x, state.x, k_eps );
  EXPECT_NEAR( tf_msg.transform.translation.y, state.y, k_eps );
  EXPECT_NEAR( tf_msg.transform.translation.z, state.z, k_eps );

  double yaw = adore::math::get_yaw( tf_msg.transform.rotation );
  EXPECT_NEAR( yaw, state.yaw_angle, 1e-6 );

  rclcpp::Time t_from_header( tf_msg.header.stamp );
  EXPECT_NEAR( t_from_header.seconds(), timestamp.seconds(), 1e-6 );
}

// ---------------------- gear + vehicle info + physical params ----------------------

TEST( AdoreDynamicsConversions, gear_state_round_trip )
{
  auto gear = static_cast<dyn::GearState>( 3 );

  auto msg       = conv::to_ros_msg( gear );
  auto gear_back = conv::to_cpp_type( msg );

  EXPECT_EQ( msg.gear_state, static_cast<uint8_t>( gear ) );
  EXPECT_EQ( static_cast<uint8_t>( gear_back ), static_cast<uint8_t>( gear ) );
}

TEST( AdoreDynamicsConversions, vehicle_info_round_trip )
{
  dyn::VehicleInfo info;
  info.gear_state                    = static_cast<dyn::GearState>( 2 );
  info.wheel_speed                   = 12.34;
  info.left_indicator_on             = true;
  info.right_indicator_on            = false;
  info.automatic_steering_on         = true;
  info.automatic_acceleration_on     = false;
  info.automatic_acceleration_active = true;
  info.clearance                     = 1.5;

  auto msg       = conv::to_ros_msg( info );
  auto info_back = conv::to_cpp_type( msg );

  EXPECT_EQ( static_cast<uint8_t>( info_back.gear_state ), static_cast<uint8_t>( info.gear_state ) );
  EXPECT_NEAR( info_back.wheel_speed, info.wheel_speed, k_eps );
  EXPECT_EQ( info_back.left_indicator_on, info.left_indicator_on );
  EXPECT_EQ( info_back.right_indicator_on, info.right_indicator_on );
  EXPECT_EQ( info_back.automatic_steering_on, info.automatic_steering_on );
  EXPECT_EQ( info_back.automatic_acceleration_on, info.automatic_acceleration_on );
  EXPECT_EQ( info_back.automatic_acceleration_active, info.automatic_acceleration_active );
  EXPECT_NEAR( info_back.clearance, info.clearance, k_eps );
}

TEST( AdoreDynamicsConversions, physical_vehicle_parameters_round_trip )
{
  auto params = make_sample_physical_params();

  auto msg         = conv::to_ros_msg( params );
  auto params_back = conv::to_cpp_type( msg );

  expect_physical_params_equal( params, params_back );
}

// ---------------------- trajectory + transpose ----------------------

TEST( AdoreDynamicsConversions, trajectory_round_trip )
{
  dyn::Trajectory traj;
  traj.label = "test_traj";

  auto s1      = make_sample_vehicle_state();
  auto s2      = make_sample_vehicle_state();
  s2.time      = s1.time + 1.0;
  s2.x         = s1.x + 1.0;
  s2.yaw_angle = s1.yaw_angle + 0.1;
  traj.states.push_back( s1 );
  traj.states.push_back( s2 );

  auto msg       = conv::to_ros_msg( traj );
  auto traj_back = conv::to_cpp_type( msg );

  EXPECT_EQ( traj.label, traj_back.label );
  ASSERT_EQ( traj.states.size(), traj_back.states.size() );
  for( std::size_t i = 0; i < traj.states.size(); ++i )
  {
    expect_vehicle_state_equal( traj.states[i], traj_back.states[i] );
  }

  EXPECT_EQ( msg.header.frame_id, "world" );
}

TEST( AdoreDynamicsConversions, trajectory_transpose_matches_states )
{
  dyn::Trajectory traj;
  traj.label = "transpose_test";

  dyn::VehicleStateDynamic s1;
  s1.time           = 1.0;
  s1.x              = 1.0;
  s1.y              = 2.0;
  s1.z              = 3.0;
  s1.vx             = 4.0;
  s1.vy             = 5.0;
  s1.ax             = 6.0;
  s1.ay             = 7.0;
  s1.yaw_angle      = 0.1;
  s1.yaw_rate       = 0.2;
  s1.steering_angle = 0.3;
  s1.steering_rate  = 0.4;

  dyn::VehicleStateDynamic s2 = s1;
  s2.time                     = 2.0;
  s2.x                        = 10.0;
  s2.y                        = 20.0;
  s2.yaw_angle                = 0.5;
  s2.yaw_rate                 = -0.2;
  s2.steering_angle           = -0.3;
  s2.steering_rate            = -0.4;

  traj.states.push_back( s1 );
  traj.states.push_back( s2 );

  auto traj_msg  = conv::to_ros_msg( traj );
  auto transpose = conv::transpose( traj_msg );

  ASSERT_EQ( transpose.x.size(), traj_msg.states.size() );
  ASSERT_EQ( transpose.y.size(), traj_msg.states.size() );
  ASSERT_EQ( transpose.time.size(), traj_msg.states.size() );

  for( std::size_t i = 0; i < traj_msg.states.size(); ++i )
  {
    const auto& st = traj_msg.states[i];

    EXPECT_NEAR( transpose.x[i], st.x, k_eps );
    EXPECT_NEAR( transpose.y[i], st.y, k_eps );
    EXPECT_NEAR( transpose.z[i], st.z, k_eps );
    EXPECT_NEAR( transpose.vx[i], st.vx, k_eps );
    EXPECT_NEAR( transpose.vy[i], st.vy, k_eps );
    EXPECT_NEAR( transpose.ax[i], st.ax, k_eps );
    EXPECT_NEAR( transpose.ay[i], st.ay, k_eps );
    EXPECT_NEAR( transpose.yaw_angle[i], st.yaw_angle, k_eps );
    EXPECT_NEAR( transpose.yaw_rate[i], st.yaw_rate, k_eps );
    EXPECT_NEAR( transpose.steering_angle[i], st.steering_angle, k_eps );
    EXPECT_NEAR( transpose.steering_rate[i], st.steering_rate, k_eps );
    EXPECT_NEAR( transpose.time[i], st.time, k_eps );
  }
}

// ---------------------- traffic participant + set ----------------------

TEST( AdoreDynamicsConversions, traffic_participant_ros_to_cpp_to_ros_full )
{
  adore_ros2_msgs::msg::TrafficParticipant msg;

  // motion state
  msg.motion_state.time            = 10.0;
  msg.motion_state.x               = 1.0;
  msg.motion_state.y               = 2.0;
  msg.motion_state.z               = 0.0;
  msg.motion_state.vx              = 3.0;
  msg.motion_state.vy              = 4.0;
  msg.motion_state.ax              = 0.1;
  msg.motion_state.ay              = 0.2;
  msg.motion_state.yaw_angle       = 0.3;
  msg.motion_state.yaw_rate        = -0.1;
  msg.motion_state.steering_angle  = 0.05;
  msg.motion_state.steering_rate   = -0.02;
  msg.motion_state.header.frame_id = "map";

  // physical parameters (some values)
  msg.physical_parameters.cog_to_front_axle = 1.1;
  msg.physical_parameters.rear_axle_to_cog  = 2.2;

  // classification
  msg.classification.type_id = 4u;

  // v2x id present
  msg.v2x_station_id = 999u;

  // tracking id
  msg.tracking_id = 42u;

  // goal point present (non-sentinel)
  msg.goal_point.x = 5.0;
  msg.goal_point.y = -3.0;

  // predicted trajectory: two states
  adore_ros2_msgs::msg::VehicleStateDynamic t_state1 = msg.motion_state;
  adore_ros2_msgs::msg::VehicleStateDynamic t_state2 = msg.motion_state;
  t_state2.x                                         = 10.0;
  t_state2.y                                         = 20.0;
  t_state2.time                                      = 11.0;

  msg.predicted_trajectory.states.push_back( t_state1 );
  msg.predicted_trajectory.states.push_back( t_state2 );
  msg.predicted_trajectory.label = "predicted";

  // route present (center_points non-empty)
  msg.route = make_sample_route_msg_for_participant();

  auto participant_cpp = conv::to_cpp_type( msg );
  auto msg_back        = conv::to_ros_msg( participant_cpp );

  // id and classification
  EXPECT_EQ( participant_cpp.id, msg.tracking_id );
  EXPECT_EQ( static_cast<uint8_t>( participant_cpp.classification ), msg.classification.type_id );
  EXPECT_EQ( msg_back.tracking_id, msg.tracking_id );
  EXPECT_EQ( msg_back.classification.type_id, msg.classification.type_id );

  // v2x optional
  ASSERT_TRUE( participant_cpp.v2x_id.has_value() );
  EXPECT_EQ( participant_cpp.v2x_id.value(), msg.v2x_station_id );
  EXPECT_EQ( msg_back.v2x_station_id, msg.v2x_station_id );

  // goal point optional
  ASSERT_TRUE( participant_cpp.goal_point.has_value() );
  EXPECT_NEAR( participant_cpp.goal_point->x, msg.goal_point.x, k_eps );
  EXPECT_NEAR( participant_cpp.goal_point->y, msg.goal_point.y, k_eps );
  EXPECT_NEAR( msg_back.goal_point.x, msg.goal_point.x, k_eps );
  EXPECT_NEAR( msg_back.goal_point.y, msg.goal_point.y, k_eps );

  // predicted trajectory optional
  ASSERT_TRUE( participant_cpp.trajectory.has_value() );
  EXPECT_EQ( participant_cpp.trajectory->states.size(), msg.predicted_trajectory.states.size() );
  EXPECT_EQ( msg_back.predicted_trajectory.states.size(), msg.predicted_trajectory.states.size() );

  // route optional
  ASSERT_TRUE( participant_cpp.route.has_value() );
  EXPECT_FALSE( msg_back.route.center_points.empty() );
}

TEST( AdoreDynamicsConversions, traffic_participant_sentinels_to_optionals )
{
  adore_ros2_msgs::msg::TrafficParticipant msg;

  // v2x_station_id stays default 0
  msg.v2x_station_id = 0u;

  // goal_point (0,0) sentinel
  msg.goal_point.x = 0.0;
  msg.goal_point.y = 0.0;

  // empty predicted_trajectory, empty route
  // (all defaults)

  auto participant_cpp = conv::to_cpp_type( msg );

  EXPECT_FALSE( participant_cpp.v2x_id.has_value() );
  EXPECT_FALSE( participant_cpp.goal_point.has_value() );
  EXPECT_FALSE( participant_cpp.trajectory.has_value() );
  EXPECT_FALSE( participant_cpp.route.has_value() );

  auto msg_back = conv::to_ros_msg( participant_cpp );

  EXPECT_EQ( msg_back.v2x_station_id, 0u );
  EXPECT_NEAR( msg_back.goal_point.x, 0.0, k_eps );
  EXPECT_NEAR( msg_back.goal_point.y, 0.0, k_eps );
  EXPECT_TRUE( msg_back.predicted_trajectory.states.empty() );
  EXPECT_TRUE( msg_back.route.center_points.empty() );
}

TEST( AdoreDynamicsConversions, traffic_participant_set_round_trip )
{
  dyn::TrafficParticipantSet set_cpp;

  auto                    state = make_sample_vehicle_state();
  auto                    phys  = make_sample_physical_params();
  auto                    cls   = static_cast<dyn::TrafficParticipantClassification>( 2 );
  dyn::TrafficParticipant p( state, 7u, cls, phys );

  p.id                       = 7u;
  set_cpp.participants[p.id] = p;

  // validity area: simple quad, enough points to be kept
  adore::math::Polygon2d poly;
  adore::math::Point2d   pt;
  pt.x = 0.0;
  pt.y = 0.0;
  poly.points.push_back( pt );
  pt.x = 1.0;
  pt.y = 0.0;
  poly.points.push_back( pt );
  pt.x = 1.0;
  pt.y = 1.0;
  poly.points.push_back( pt );
  pt.x = 0.0;
  pt.y = 1.0;
  poly.points.push_back( pt );
  set_cpp.validity_area = poly;

  auto msg      = conv::to_ros_msg( set_cpp );
  auto set_back = conv::to_cpp_type( msg );

  ASSERT_EQ( set_back.participants.size(), 1u );
  auto it = set_back.participants.find( 7u );
  ASSERT_NE( it, set_back.participants.end() );

  // motion state preserved round-trip
  expect_vehicle_state_equal( it->second.state, state );

  // validity area present
  ASSERT_TRUE( set_back.validity_area.has_value() );
  EXPECT_EQ( set_back.validity_area->points.size(), poly.points.size() );
}
