/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "magut_eye_on_base");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string group_name="manipulator";
  if (!pnh.getParam("group_name",group_name))
  {
    ROS_ERROR("%s/group_name undefined, set manipulator",pnh.getNamespace().c_str());
  }
  
  moveit::planning_interface::MoveGroupInterface move_group(group_name);

  ROS_INFO("Start node");
  ros::AsyncSpinner as(4);
  as.start();
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;
  std::string robot_base_link;
  std::string robot_end_link;
  std::string camera_base_link;
  std::string camera_marker_link;


  if (!pnh.getParam("robot_base_link",robot_base_link))
  {
    ROS_ERROR("%s/robot_base_link undefined, stop",pnh.getNamespace().c_str());
    return 0;
  }
  if (!pnh.getParam("robot_end_link",robot_end_link))
  {
    ROS_ERROR("%s/robot_end_link undefined, stop",pnh.getNamespace().c_str());
    return 0;
  }
  if (!pnh.getParam("camera_base_link",camera_base_link))
  {
    ROS_ERROR("%s/camera_base_link undefined, stop",pnh.getNamespace().c_str());
    return 0;
  }
  if (!pnh.getParam("camera_marker_link",camera_marker_link))
  {
    ROS_ERROR("%s/camera_marker_link     undefined, stop",pnh.getNamespace().c_str());
    return 0;
  }

  int rotation_samples_number=10;
  if (!pnh.getParam("rotation_samples_number",rotation_samples_number))
  {
    ROS_ERROR("%s/rotation_samples_number     undefined, stop",pnh.getNamespace().c_str());
    return 0;
  }

  double amplitude=0.4;
  double amplitude_rot=20.0/180*M_PI;
  double t_max=5;
  move_group.setMaxVelocityScalingFactor(0.1);
  ROS_INFO("Move the robot to the initial position (the robot should be able to move around 200mm  and 15 degree in each direction)");
  ROS_INFO("Afterthat, be sure the robot is stopped");


  int y;

  while (ros::ok())
  {
    ROS_INFO("Press");
    ROS_INFO("- '1' and ENTER to continue");
    ROS_INFO("- '-10' and ENTER to stop");

    std::string input;
    std::cin >> input;
    try
    {
      y = boost::lexical_cast<int> (input);
    }
    catch (boost::bad_lexical_cast const &)
    {
      ROS_ERROR("Error: input string was not valid.");
      continue;
    }
    if (y==-10)
      return 0;
    if (y==1)
      break;
  }

  tf::StampedTransform transform_camera_marker;
  tf::StampedTransform transform_base_tool;

  ros::Rate rate(10.0);
  while (ros:: ok())
  {
    try
    {
      listener.lookupTransform(camera_base_link,camera_marker_link,
                               ros::Time(0), transform_camera_marker);
      listener.lookupTransform(robot_base_link,robot_end_link,
                               ros::Time(0), transform_base_tool);
      break;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  Eigen::Affine3d T_camera_marker_orig;
  Eigen::Affine3d T_base_tool_orig;
  tf::poseTFToEigen(transform_base_tool,T_base_tool_orig);
  tf::poseTFToEigen(transform_camera_marker,T_camera_marker_orig);

  ROS_DEBUG_STREAM("Transform "<<robot_base_link<<" <= "<<robot_end_link<<"\n"<<T_base_tool_orig.matrix());
  ROS_DEBUG_STREAM("Transform "<<camera_base_link<<" <= "<<camera_marker_link<<"\n"<<T_camera_marker_orig.matrix());

  // X DIRECTION
  ROS_INFO("The robot will move in X direction around +/-%f mm",amplitude*1000);

  geometry_msgs::Pose actual_pose = move_group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose=actual_pose;
  geometry_msgs::Pose initial_pose=actual_pose;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success;

  moveit::core::RobotState initial_state(*move_group.getCurrentState());

  move_group.setStartState(*move_group.getCurrentState());
  const moveit::core::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(group_name);
  ROS_INFO("reference_frame %s",move_group.getPoseReferenceFrame().c_str());

  move_group.setStartState(*move_group.getCurrentState());

  while(ros::ok())
  {
    double delta_x=0;
    while (std::abs(delta_x)<0.5*amplitude)
      delta_x=(-1.0+2.0*(1.0*(double)std::rand()/((double)RAND_MAX)))*amplitude;

    target_pose.position.x=actual_pose.position.x+delta_x;
    ROS_DEBUG_STREAM("Actual\n"<<actual_pose<<"\nTarget=\n"<<target_pose);
    robot_state::RobotState target_state(*move_group.getCurrentState());
    if (!target_state.setFromIK(joint_model_group,target_pose)) continue;
    move_group.setJointValueTarget(target_state);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_WARN("unable to plan");
      continue;
    }
    else
    {
      ROS_INFO("planned!");
    }
    if (plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec()>t_max)
      continue;

    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }
  ros::Duration(5.0).sleep();


  while (ros:: ok())
  {
    try
    {
      listener.lookupTransform(camera_base_link,camera_marker_link,
                               ros::Time(0), transform_camera_marker);
      listener.lookupTransform(robot_base_link,robot_end_link,
                               ros::Time(0), transform_base_tool);
      break;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  Eigen::Affine3d T_camera_marker_x;
  Eigen::Affine3d T_base_tool_x;
  tf::poseTFToEigen(transform_base_tool,T_base_tool_x);
  tf::poseTFToEigen(transform_camera_marker,T_camera_marker_x);
  ROS_DEBUG_STREAM("Transform "<<robot_base_link<<" <= "<<robot_end_link<<"\n"<<T_base_tool_x.matrix());
  ROS_DEBUG_STREAM("Transform "<<camera_base_link<<" <= "<<camera_marker_link<<"\n"<<T_camera_marker_x.matrix());

  double distance_x=T_base_tool_x.translation()(0)-T_base_tool_orig.translation()(0);
  double distance_c=(T_base_tool_x.translation()-T_base_tool_orig.translation()).norm();
  ROS_INFO("Do you moved the robot of %f (%f) mm? if not check the TF and restart the node",distance_x*1e3,distance_c*1e3);
  ROS_INFO_STREAM("delta = " << (T_base_tool_x.translation()-T_base_tool_orig.translation()).transpose());
  double distance_x_camera=(T_camera_marker_x.translation()-T_camera_marker_orig.translation()).norm();
  ROS_INFO("The amplitude of the movement recorded by the camera is %f mm",distance_x_camera*1e3);
  Eigen::Vector3d x_robot_in_camera=(T_camera_marker_x.translation()-T_camera_marker_orig.translation())/distance_x;


  // Y DIRECTION
  ROS_INFO("The robot will move in Y direction around +/-%f mm",amplitude*1000);
  target_pose=initial_pose;
  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());
    double delta_y=0;
    while (std::abs(delta_y)<0.5*amplitude)
      delta_y=(-1.0+2.0*(1.0*(double)std::rand()/((double)RAND_MAX)))*amplitude;

    target_pose.position.y=initial_pose.position.y+delta_y;
    ROS_DEBUG_STREAM("Actual\n"<<actual_pose<<"\nTarget=\n"<<target_pose);
    robot_state::RobotState target_state(*move_group.getCurrentState());
    if (!target_state.setFromIK(joint_model_group,target_pose)) continue;
    move_group.setJointValueTarget(target_state);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_WARN("unable to plan");
      continue;
    }
    else
    {
      ROS_INFO("planned!");
    }
    if (plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec()>t_max)
      continue;
    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }
  ros::Duration(5.0).sleep();


  while (ros:: ok())
  {
    try
    {
      listener.lookupTransform(camera_base_link,camera_marker_link,
                               ros::Time(0), transform_camera_marker);
      listener.lookupTransform(robot_base_link,robot_end_link,
                               ros::Time(0), transform_base_tool);
      break;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  Eigen::Affine3d T_camera_marker_y;
  Eigen::Affine3d T_base_tool_y;
  tf::poseTFToEigen(transform_base_tool,T_base_tool_y);
  tf::poseTFToEigen(transform_camera_marker,T_camera_marker_y);
  ROS_DEBUG_STREAM("Transform "<<robot_base_link<<" <= "<<robot_end_link<<"\n"<<T_base_tool_y.matrix());
  ROS_DEBUG_STREAM("Transform "<<camera_base_link<<" <= "<<camera_marker_link<<"\n"<<T_camera_marker_y.matrix());

  double distance_y=T_base_tool_y.translation()(1)-T_base_tool_orig.translation()(1);
  ROS_INFO("Do you moved the robot of %f mm? if not check the TF and restart the node",distance_y*1e3);
  double distance_y_camera=(T_camera_marker_y.translation()-T_camera_marker_orig.translation()).norm();
  ROS_INFO("The amplitude of the movement recorded by the camera is %f mm",distance_y_camera*1e3);


  Eigen::Vector3d y_robot_in_camera=(T_camera_marker_y.translation()-T_camera_marker_orig.translation())/distance_y;
  double angle=std::acos(x_robot_in_camera.normalized().dot(y_robot_in_camera.normalized()))*180.0/M_PI;
  ROS_INFO_STREAM("x robot in camera = " << x_robot_in_camera.normalized().transpose());
  ROS_INFO_STREAM("y robot in camera = " << y_robot_in_camera.normalized().transpose());

  ROS_INFO("angle between X and Y = %f deg", angle);


  // Z DIRECTION
  ROS_INFO("The robot will move in Z direction around +/-%f mm",amplitude*1000);
  target_pose=initial_pose;
  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());
    double delta_z=0;
    while (std::abs(delta_z)<0.5*amplitude)
      delta_z=(-1.0+2.0*(1.0*(double)std::rand()/((double)RAND_MAX)))*amplitude;

    target_pose.position.z=initial_pose.position.z+delta_z;
    ROS_DEBUG_STREAM("Actual\n"<<actual_pose<<"\nTarget=\n"<<target_pose);
    robot_state::RobotState target_state(*move_group.getCurrentState());
    if (!target_state.setFromIK(joint_model_group,target_pose)) continue;
    move_group.setJointValueTarget(target_state);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_WARN("unable to plan");
      continue;
    }
    else
    {
      ROS_INFO("planned!");
    }
    if (plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec()>t_max)
      continue;

    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }

  ros::Duration(5.0).sleep();

  while (ros:: ok())
  {
    try
    {
      listener.lookupTransform(camera_base_link,camera_marker_link,
                               ros::Time(0), transform_camera_marker);
      listener.lookupTransform(robot_base_link,robot_end_link,
                               ros::Time(0), transform_base_tool);
      break;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }

  Eigen::Affine3d T_camera_marker_z;
  Eigen::Affine3d T_base_tool_z;
  tf::poseTFToEigen(transform_base_tool,T_base_tool_z);
  tf::poseTFToEigen(transform_camera_marker,T_camera_marker_z);
  ROS_DEBUG_STREAM("Transform "<<robot_base_link<<" <= "<<robot_end_link<<"\n"<<T_base_tool_z.matrix());
  ROS_DEBUG_STREAM("Transform "<<camera_base_link<<" <= "<<camera_marker_link<<"\n"<<T_camera_marker_z.matrix());

  double distance_z=T_base_tool_z.translation()(2)-T_base_tool_orig.translation()(2);
  ROS_INFO("Do you moved the robot of %f mm? if not check the TF and restart the node",distance_z*1e3);
  double distance_z_camera=(T_camera_marker_z.translation()-T_camera_marker_orig.translation()).norm();
  ROS_INFO("The amplitude of the movement recorded by the camera is %f mm",distance_z_camera*1e3);

  Eigen::Vector3d z_robot_in_camera_validation=(T_camera_marker_z.translation()-T_camera_marker_orig.translation())/distance_z;
  x_robot_in_camera.normalize();
  y_robot_in_camera=y_robot_in_camera-y_robot_in_camera.dot(x_robot_in_camera)*x_robot_in_camera;
  y_robot_in_camera.normalize();
  Eigen::Vector3d z_robot_in_camera=x_robot_in_camera.cross(y_robot_in_camera);
  z_robot_in_camera.normalize();
  z_robot_in_camera_validation.normalize();
  double angular_error=std::acos(z_robot_in_camera_validation.dot(z_robot_in_camera))*180.0/M_PI;
  ROS_INFO("Perpendicular error on Z axis %f degree",angular_error);
  ROS_INFO_STREAM("x_robot_in_camera = "<<x_robot_in_camera.transpose());
  ROS_INFO_STREAM("y_robot_in_camera = "<<y_robot_in_camera.transpose());
  ROS_INFO_STREAM("z_robot_in_camera = "<<z_robot_in_camera.transpose());
  ROS_INFO_STREAM("z_robot_in_camera VALIDATION = "<<z_robot_in_camera_validation.transpose());

  Eigen::Matrix3d R_camera_base;
  R_camera_base<< x_robot_in_camera, y_robot_in_camera, z_robot_in_camera;
  Eigen::Matrix3d R_base_camera;
  R_base_camera=R_camera_base.transpose();


  // Coming home

  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(initial_state);

  success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_WARN("unable to plan the coming-home movement");
    return 0;
  }
  else
  {
    ROS_INFO("planned!");
  }
  if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
    ROS_WARN("unable to execute the coming-home movement");
  else
  {
    actual_pose=target_pose;
    ROS_INFO("moved");
  }
  ros::Duration(5.0).sleep();
  // Rotation


  Eigen::Matrix3d identity;
  Eigen::Matrix<double,-1,-1> regressor;
  Eigen::VectorXd data;

  identity.setIdentity();
  int irot=0;
  regressor.resize(rotation_samples_number*3,6);
  data.resize(rotation_samples_number*3);
  regressor.setZero();
  data.setZero();

  while (irot<rotation_samples_number)
  {
    target_pose=initial_pose;
    move_group.setStartState(*move_group.getCurrentState());
    Eigen::Affine3d T_initial_pose;
    tf::poseMsgToEigen(initial_pose,T_initial_pose);

    double delta_rot=0;

    while (std::abs(delta_rot)<0.5*amplitude_rot)
    {
      double tmp=(double)std::rand()/((double)RAND_MAX);
      delta_rot=(-1.0+2.0*tmp)*amplitude_rot;
    }

    Eigen::Vector3d axis=Eigen::MatrixXd::Random(3,1);
    axis.normalize();
    Eigen::AngleAxisd q(delta_rot,axis);
    T_initial_pose=T_initial_pose*q;
    tf::poseEigenToMsg(T_initial_pose,target_pose);
    ROS_DEBUG_STREAM("Actual\n"<<actual_pose<<"\nTarget=\n"<<target_pose);
    robot_state::RobotState target_state(initial_state);
    if (!target_state.setFromIK(joint_model_group,target_pose)) continue;
    move_group.setJointValueTarget(target_state);

    success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!success)
    {
      ROS_WARN("unable to plan");
      continue;
    }
    else
    {
      ROS_INFO("planned!");
    }
    if (plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec()>t_max)
      continue;

    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
    {
      ROS_WARN("unable to execute");
      continue;
    }
    actual_pose=target_pose;
    ROS_INFO("moved");

    ros::Duration(5.0).sleep();

    while (ros:: ok())
    {
      try
      {
        listener.lookupTransform(camera_base_link,camera_marker_link,
                                 ros::Time(0), transform_camera_marker);
        listener.lookupTransform(robot_base_link,robot_end_link,
                                 ros::Time(0), transform_base_tool);
        break;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
    }
    Eigen::Affine3d T_camera_marker_rot;
    Eigen::Affine3d T_base_tool_rot;
    tf::poseTFToEigen(transform_base_tool,T_base_tool_rot);
    tf::poseTFToEigen(transform_camera_marker,T_camera_marker_rot);
    ROS_DEBUG_STREAM("Transform "<<robot_base_link<<" <= "<<robot_end_link<<"\n"<<T_base_tool_rot.matrix());
    ROS_DEBUG_STREAM("Transform "<<camera_base_link<<" <= "<<camera_marker_link<<"\n"<<T_camera_marker_rot.matrix());

    regressor.block(irot*3,0,3,3)=T_base_tool_rot.matrix().block(0,0,3,3);
    regressor.block(irot*3,3,3,3)=-identity;

    data.block(irot*3,0,3,1)=R_base_camera*T_camera_marker_rot.translation()-T_base_tool_rot.translation();
    irot++;
  }


  Eigen::VectorXd solution=regressor.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(data);
  ROS_INFO_STREAM("error = \n"<<regressor*solution-data);
  Eigen::VectorXd orig_base_camera=solution.tail(3);
  Eigen::VectorXd orig_tool_marker=solution.head(3);

  Eigen::Quaterniond q_base_camera(R_base_camera);

  ROS_INFO_STREAM("R_base_camera\n"<<R_base_camera);
  ROS_INFO_STREAM("orig_base_camera\n"<<orig_base_camera.transpose());
  ROS_INFO_STREAM("q_base_camera\n"<<q_base_camera.vec().transpose()<<","<<q_base_camera.w());

  ROS_INFO_STREAM("transformation:\n parent = "<< robot_base_link <<"\n" <<
                  " child = " << camera_base_link <<"\n" <<
                  "x y z qx qy qz qw\n"<<orig_base_camera.transpose()<<" "<<q_base_camera.vec().transpose()<<" "<<q_base_camera.w());


  ROS_INFO_STREAM("\n\n\n\norig_tool_marker\n"<<orig_tool_marker);
  Eigen::Affine3d T_base_camera;
  T_base_camera=q_base_camera;
  T_base_camera.translation()=orig_base_camera;
  tf::StampedTransform tf_base_camera;
  tf_base_camera.frame_id_=robot_base_link;
  tf_base_camera.child_frame_id_=camera_base_link+"_test";
  tf::poseEigenToTF(T_base_camera,tf_base_camera);

  Eigen::Affine3d T_tool_marker;
  T_tool_marker.linear()=T_base_tool_orig.linear().inverse()*T_base_camera.linear()*T_camera_marker_orig.linear();
  T_tool_marker.translation()=orig_tool_marker;
  tf::StampedTransform tf_tool_marker;
  tf_tool_marker.frame_id_=robot_end_link;
  tf_tool_marker.child_frame_id_=camera_marker_link+"_test";
  tf::poseEigenToTF(T_tool_marker,tf_tool_marker);


  move_group.setStartState(*move_group.getCurrentState());
  move_group.setJointValueTarget(initial_state);

  success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (!success)
  {
    ROS_WARN("unable to plan the coming-home movement");
    return 0;
  }
  else
  {
    ROS_INFO("planned!");
  }
  if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
    ROS_WARN("unable to execute the coming-home movement");
  else
  {
    actual_pose=target_pose;
    ROS_INFO("moved");
  }

  ros::Duration(5.0).sleep();
  ROS_INFO_STREAM("T_base_camera\n"<<T_base_camera.matrix());
  ros::Time t0=ros::Time::now();
  while (ros::ok())
  {
    if ((ros::Time::now()-t0).toSec()>10)
      break;
    tf_base_camera.stamp_=ros::Time::now();
    broadcaster.sendTransform(tf_base_camera);
    tf_tool_marker.stamp_=ros::Time::now();
    broadcaster.sendTransform(tf_tool_marker);
    ros::Duration(0.05).sleep();
  }

  while (ros::ok())
  {
    ROS_INFO("Press");
    ROS_INFO("- '1' to publish current TF");
    ROS_INFO("- '-10' and ENTER to stop");

    std::string input;
    std::cin >> input;
    try
    {
      y = boost::lexical_cast<int> (input);
    }
    catch (boost::bad_lexical_cast const &)
    {
      ROS_ERROR("Error: input string was not valid.");
      continue;
    }
    if (y==-10)
      return 0;
    if (y==1)
      break;
  }

  tf_base_camera.child_frame_id_=camera_base_link;
  tf_tool_marker.child_frame_id_=camera_marker_link;
  while (ros::ok())
  {
    tf_base_camera.stamp_=ros::Time::now();
    broadcaster.sendTransform(tf_base_camera);
    tf_tool_marker.stamp_=ros::Time::now();
    broadcaster.sendTransform(tf_tool_marker);
    ros::Duration(0.05).sleep();
  }
  return 0;
}
