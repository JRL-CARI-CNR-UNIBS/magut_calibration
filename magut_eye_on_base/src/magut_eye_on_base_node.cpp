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


  ROS_INFO("Move the robot to the initial position (the robot should be able to move around 200mm in each direction)");
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
  ROS_INFO("\n\nMove the robot in the X direction around +/- 200mm . Afterthat");

  geometry_msgs::Pose actual_pose = move_group.getCurrentPose().pose;
  geometry_msgs::Pose target_pose=actual_pose;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success;
  move_group.setStartState(*move_group.getCurrentState());
  const moveit::core::JointModelGroup* joint_model_group =move_group.getCurrentState()->getJointModelGroup(group_name);
  ROS_INFO("reference_frame %s",move_group.getPoseReferenceFrame().c_str());

  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());
    actual_pose = move_group.getCurrentPose(robot_end_link).pose;
    double delta_x=0;
    while (std::abs(delta_x)<0.05)
      delta_x=-0.200+0.4*(1.0*(double)std::rand()/((double)RAND_MAX));
    ROS_INFO("delta_x=%f",delta_x);

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
    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }


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



  // Y DIRECTION
  ROS_INFO("\n\nMove the robot in the Y direction around +/- 200mm . Afterthat");
  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());
    double delta_y=0;
    while (std::abs(delta_y)<0.05)
      delta_y=-0.200+0.4*(1.0*(double)std::rand()/((double)RAND_MAX));
    ROS_INFO("delta_y=%f",delta_y);
    target_pose.position.y=actual_pose.position.y+delta_y;
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
    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }


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

  double distance_y=T_base_tool_y.translation()(1)-T_base_tool_x.translation()(1);
  ROS_INFO("Do you moved the robot of %f mm? if not check the TF and restart the node",distance_y*1e3);
  double distance_y_camera=(T_camera_marker_y.translation()-T_camera_marker_x.translation()).norm();
  ROS_INFO("The amplitude of the movement recorded by the camera is %f mm",distance_y_camera*1e3);


  // Z DIRECTION
  ROS_INFO("\n\nMove the robot in the Z direction around +/- 200mm . Afterthat");
  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());
    double delta_z=0;
    while (std::abs(delta_z)<0.05)
      delta_z=-0.200+0.4*(1.0*(double)std::rand()/((double)RAND_MAX));
    ROS_INFO("delta_z=%f",delta_z);
    target_pose.position.z=actual_pose.position.z+delta_z;
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
    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }



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

  double distance_z=T_base_tool_z.translation()(2)-T_base_tool_y.translation()(2);
  ROS_INFO("Do you moved the robot of %f mm? if not check the TF and restart the node",distance_z*1e3);
  double distance_z_camera=(T_camera_marker_z.translation()-T_camera_marker_y.translation()).norm();
  ROS_INFO("The amplitude of the movement recorded by the camera is %f mm",distance_z_camera*1e3);

  Eigen::Vector3d x_robot_in_camera=(T_camera_marker_x.translation()-T_camera_marker_orig.translation())/distance_x;
  Eigen::Vector3d y_robot_in_camera=(T_camera_marker_y.translation()-T_camera_marker_x.translation())/distance_y;
  Eigen::Vector3d z_robot_in_camera_validation=(T_camera_marker_z.translation()-T_camera_marker_y.translation())/distance_z;
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

  Eigen::Matrix3d R_camera_robot;
  R_camera_robot<< x_robot_in_camera, y_robot_in_camera, z_robot_in_camera;
  Eigen::Matrix3d R_robot_camera;
  R_robot_camera=R_camera_robot.transpose();




  // Rotation

  while(ros::ok())
  {
    move_group.setStartState(*move_group.getCurrentState());
    Eigen::Affine3d T_actual_pose;
    tf::poseMsgToEigen(actual_pose,T_actual_pose);

    double delta_rot=0;
    while (std::abs(delta_rot)<0.025)
      delta_rot=-0.1+0.2*(1.0*(double)std::rand()/((double)RAND_MAX));

    ROS_INFO("delta_rot=%f",delta_rot);
    Eigen::AngleAxisd q(delta_rot,Eigen::Vector3d::UnitZ());
    T_actual_pose=T_actual_pose*q;
    tf::poseEigenToMsg(T_actual_pose,target_pose);
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

    if (!(move_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS))
      ROS_WARN("unable to execute");
    else
    {
      actual_pose=target_pose;
      ROS_INFO("moved");
      break;
    }
  }





  Eigen::Matrix3d identity;
  Eigen::Matrix<double,-1,-1> regressor;
  Eigen::VectorXd data;

  identity.setIdentity();
  int irot=0;
  regressor.resize(rotation_samples_number*3,6);
  data.resize(rotation_samples_number*3);

  while (irot<rotation_samples_number)
  {
    if (irot>0)
    {
      ROS_INFO("\n\nIteration %d of %d. Rotate the tool. Afterthat",irot+1,rotation_samples_number);
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
    }
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

    Eigen::Matrix<double,3,6> row;
    row << T_base_tool_rot.matrix().block(0,0,3,3), identity;
    regressor.block(irot*3,0,3,6)=row;
    data.block(irot*3,0,3,1)=R_robot_camera*T_camera_marker_rot.translation()-T_base_tool_rot.translation();
    irot++;
  }

  Eigen::VectorXd solution=regressor.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(data);
  Eigen::VectorXd orig_robot_camera=solution.tail(3);
  Eigen::VectorXd orig_tool_marker=solution.head(3);

  Eigen::Quaterniond q_robot_camera(R_robot_camera);

  ROS_INFO_STREAM("R_robot_camera\n"<<R_robot_camera);
  ROS_INFO_STREAM("q_robot_camera\n"<<q_robot_camera.vec()<<","<<q_robot_camera.w());
  ROS_INFO_STREAM("orig_robot_camera\n"<<orig_robot_camera);
  ROS_INFO_STREAM("orig_tool_marker\n"<<orig_tool_marker);
  Eigen::Affine3d T_robot_camera;
  T_robot_camera=q_robot_camera;
  T_robot_camera.translation()=orig_robot_camera;
  tf::StampedTransform t;
  t.frame_id_=robot_base_link;
  t.child_frame_id_=camera_base_link;
  tf::poseEigenToTF(T_robot_camera,t);
  ROS_INFO_STREAM("T_robot_camera\n"<<T_robot_camera.matrix());
  while (ros::ok())
  {
    t.stamp_=ros::Time::now();
    broadcaster.sendTransform(t);
    ros::Duration(0.05).sleep();
  }
  return 0;
}
