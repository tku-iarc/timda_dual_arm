/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
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
*******************************************************************************/

/*
 * Link.cpp
 *
 *  Created on: Jan 11, 2016
 *      Author: SCH
 */
#include <assert.h>
#include <iostream>
#include "manipulator_h_kinematics_dynamics/manipulator_h_kinematics_dynamics.h"

namespace robotis_manipulator_h
{

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics()
{
}
ManipulatorKinematicsDynamics::~ManipulatorKinematicsDynamics()
{
}

ManipulatorKinematicsDynamics::ManipulatorKinematicsDynamics(TreeSelect tree)
{
  for (int id = 0; id <= ALL_JOINT_ID; id++)
    manipulator_link_data_[id] = new LinkData();

  if (tree == ARM)
  {
    manipulator_link_data_[0]->name_    = "base";
    manipulator_link_data_[0]->parent_  = -1;
    manipulator_link_data_[0]->sibling_ = -1;
    manipulator_link_data_[0]->child_   = 1;
    manipulator_link_data_[0]->mass_    = 0.0;
    manipulator_link_data_[0]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[0]->joint_limit_max_   = 0;
    manipulator_link_data_[0]->joint_limit_min_   = -0.8;
    manipulator_link_data_[0]->inertia_           = robotis_framework::getInertiaXYZ(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    manipulator_link_data_[1]->name_    = "joint1";
    manipulator_link_data_[1]->parent_  = 0;
    manipulator_link_data_[1]->sibling_ = -1;
    manipulator_link_data_[1]->child_   = 2;
    manipulator_link_data_[1]->mass_    = 0.85644;
    manipulator_link_data_[1]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[1]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[1]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[1]->joint_limit_max_   = 180 * M_PI/180;
    manipulator_link_data_[1]->joint_limit_min_   = -180 * M_PI/180;
    manipulator_link_data_[1]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[2]->name_    = "joint2";
    manipulator_link_data_[2]->parent_  = 1;
    manipulator_link_data_[2]->sibling_ = -1;
    manipulator_link_data_[2]->child_   = 3;
    manipulator_link_data_[2]->mass_    = 0.94658;
    manipulator_link_data_[2]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.16);
    manipulator_link_data_[2]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    manipulator_link_data_[2]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[2]->joint_limit_max_   = 30 * M_PI/180;
    manipulator_link_data_[2]->joint_limit_min_   = -210 * M_PI/180;
    manipulator_link_data_[2]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[3]->name_    = "joint3";
    manipulator_link_data_[3]->parent_  = 2;
    manipulator_link_data_[3]->sibling_ = -1;
    manipulator_link_data_[3]->child_   = 4;
    manipulator_link_data_[3]->mass_    = 1.30260;
    manipulator_link_data_[3]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[3]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[3]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[3]->joint_limit_max_   = 180 * M_PI/180;
    manipulator_link_data_[3]->joint_limit_min_   = -180 * M_PI/180;
    manipulator_link_data_[3]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[4]->name_    = "joint4";
    manipulator_link_data_[4]->parent_  = 3;
    manipulator_link_data_[4]->sibling_ = -1;
    manipulator_link_data_[4]->child_   = 5;
    manipulator_link_data_[4]->mass_    = 1.236;
    manipulator_link_data_[4]->relative_position_ = robotis_framework::getTransitionXYZ(-0.03, 0, 0.29);
    manipulator_link_data_[4]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[4]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[4]->joint_limit_max_   = 175 * M_PI/180;
    manipulator_link_data_[4]->joint_limit_min_   = -40 * M_PI/180;
    manipulator_link_data_[4]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[5]->name_    = "joint5";
    manipulator_link_data_[5]->parent_  = 4;
    manipulator_link_data_[5]->sibling_ = -1;
    manipulator_link_data_[5]->child_   = 6;
    manipulator_link_data_[5]->mass_    = 0.491;
    manipulator_link_data_[5]->relative_position_ = robotis_framework::getTransitionXYZ(-0.03, 0.0, 0.0);
    manipulator_link_data_[5]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[5]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[5]->joint_limit_max_   = 180 * M_PI/180;
    manipulator_link_data_[5]->joint_limit_min_   = -180 * M_PI/180;
    manipulator_link_data_[5]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);

    manipulator_link_data_[6]->name_    = "joint6";
    manipulator_link_data_[6]->parent_  = 5;
    manipulator_link_data_[6]->sibling_ = -1;
    manipulator_link_data_[6]->child_   = 7;
    manipulator_link_data_[6]->mass_    = 0.454;
    manipulator_link_data_[6]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.263);
    manipulator_link_data_[6]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    manipulator_link_data_[6]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[6]->joint_limit_max_   = 130 * M_PI/180;
    manipulator_link_data_[6]->joint_limit_min_   = -130 * M_PI/180;
    manipulator_link_data_[6]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    manipulator_link_data_[7]->name_    = "joint7";
    manipulator_link_data_[7]->parent_  = 6;
    manipulator_link_data_[7]->sibling_ = -1;
    manipulator_link_data_[7]->child_   = 8;
    manipulator_link_data_[7]->mass_    = 0.454;
    manipulator_link_data_[7]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[7]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, 1.0);
    manipulator_link_data_[7]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[7]->joint_limit_max_   = 270 * M_PI/180;
    manipulator_link_data_[7]->joint_limit_min_   = -270 * M_PI/180;
    manipulator_link_data_[7]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    manipulator_link_data_[8]->name_    = "end";
    manipulator_link_data_[8]->parent_  = 7;
    manipulator_link_data_[8]->sibling_ = -1;
    manipulator_link_data_[8]->child_   = -1;
    manipulator_link_data_[8]->mass_    = 0.0;
    manipulator_link_data_[8]->relative_position_ = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.3985);
    manipulator_link_data_[8]->joint_axis_        = robotis_framework::getTransitionXYZ(0.0, 0.0, -1.0);
    manipulator_link_data_[8]->center_of_mass_    = robotis_framework::getTransitionXYZ(0.0, 0.0, 0.0);
    manipulator_link_data_[8]->joint_limit_max_   = 100.0;
    manipulator_link_data_[8]->joint_limit_min_   = -100.0;
    manipulator_link_data_[8]->inertia_           = robotis_framework::getInertiaXYZ(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
  
    load_LinkParam();
  }
}

void ManipulatorKinematicsDynamics::load_LinkParam()
{
    ros::NodeHandle nh_private("~");
    std::string file_path  = nh_private.param<std::string>("link_file_path", "");
    
    YAML::Node doc;
    try
    {
        // load yaml
        doc = YAML::LoadFile(file_path.c_str());
    }
    catch (const std::exception &e)
    {
        std::cout << "Fail to load yaml file." << std::endl;
        return;
    }

    DHTABLE.resize(8, 4);

    // parse dh links
    YAML::Node _dh_links = doc["dh_links"];
    for (YAML::iterator _it = _dh_links.begin(); _it != _dh_links.end(); ++_it)
    {
        int link_num = _it->first.as<int>();

        std::map<std::string, double> param = _it->second.as<std::map<std::string, double> >();
        DHTABLE(link_num , 0) = param["a"];
        DHTABLE(link_num , 1) = param["alpha"] * M_PI / 180.0;
        DHTABLE(link_num , 2) = param["d"];
        DHTABLE(link_num , 3) = param["theta"] * M_PI / 180.0;
    }
    d1 = DHTABLE(1,2);
    d2 = DHTABLE(3,2);
    d3 = DHTABLE(5,2);
    d4 = DHTABLE(7,2);
    a1 = DHTABLE(4,0);
    a2 = DHTABLE(4,0);
    Lse = sqrt(pow(a1,2)+pow(d2,2));
    Lew = sqrt(pow(a2,2)+pow(d3,2));
    RL_prm = cos(DHTABLE(0, 3));    

    JointAngle     = Eigen::MatrixXd::Zero(8, 1);
    tmp_JointAngle = Eigen::MatrixXd::Zero(8, 1);

    R03 = Eigen::MatrixXd::Zero(4, 4);
    R04 = Eigen::MatrixXd::Zero(4, 4);
    R07 = Eigen::MatrixXd::Zero(4, 4);
    R47 = Eigen::MatrixXd::Zero(4, 4);
    R57 = Eigen::MatrixXd::Zero(4, 4);
    R72 = Eigen::MatrixXd::Zero(4, 4);
    R05_notheta3 = Eigen::MatrixXd::Zero(4, 4);
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute(int to)
{
  int id = manipulator_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == 0)
  {
    idx.push_back(0);
    idx.push_back(to);
  }
  else
  {
    idx = findRoute(id);
    idx.push_back(to);
  }

  return idx;
}

std::vector<int> ManipulatorKinematicsDynamics::findRoute(int from, int to)
{
  int id = manipulator_link_data_[to]->parent_;

  std::vector<int> idx;

  if (id == from)
  {
    idx.push_back(from);
    idx.push_back(to);
  }
  else if (id != 0)
  {
    idx = findRoute(from, id);
    idx.push_back(to);
  }

  return idx;
}

double ManipulatorKinematicsDynamics::totalMass(int joint_ID)
{
  double mass;

  if (joint_ID == -1)
    mass = 0.0;
  else
    mass = manipulator_link_data_[joint_ID]->mass_ + totalMass(manipulator_link_data_[joint_ID]->sibling_)
                                                   + totalMass(manipulator_link_data_[joint_ID]->child_);

  return mass;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcMC(int joint_ID)
{
  Eigen::MatrixXd mc(3, 1);

  if (joint_ID == -1)
    mc = Eigen::MatrixXd::Zero(3, 1);
  else
  {
    mc = manipulator_link_data_[joint_ID]->mass_ * (manipulator_link_data_[joint_ID]->orientation_
        * manipulator_link_data_[joint_ID]->center_of_mass_ + manipulator_link_data_[joint_ID]->position_);
    mc = mc + calcMC(manipulator_link_data_[joint_ID]->sibling_) + calcMC(manipulator_link_data_[joint_ID]->child_);
  }

  return mc;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcCOM(Eigen::MatrixXd MC)
{
  double mass;
  Eigen::MatrixXd COM(3, 1);

  mass = totalMass(0);

  COM = MC / mass;

  return COM;
}

void ManipulatorKinematicsDynamics::forwardKinematics(int joint_ID)
{
  if (joint_ID == -1)
    return;

  manipulator_link_data_[0]->position_ = Eigen::MatrixXd::Zero(3, 1);
  manipulator_link_data_[0]->orientation_ = robotis_framework::calcRodrigues(
                                                robotis_framework::calcHatto(manipulator_link_data_[0]->joint_axis_),
                                                manipulator_link_data_[0]->joint_angle_);

  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd A(4,4);
  Eigen::VectorXd DH_row(4);
  DHTABLE(0,2) = manipulator_link_data_[0]->slide_position_;
 
  for ( int i=0; i<=joint_ID; i++ )
  {
    DH_row = DHTABLE.row(i);
    A = transform(manipulator_link_data_[i]->joint_angle_, DH_row);

    T = T*A;
    manipulator_link_data_[i+1]->position_ = T.block(0,3,3,1);
    manipulator_link_data_[i+1]->orientation_ = T.block(0,0,3,3);
    
    manipulator_link_data_[i+1]->transformation_.block<3, 1>(0, 3) = manipulator_link_data_[i+1]->position_;
    manipulator_link_data_[i+1]->transformation_.block<3, 3>(0, 0) = manipulator_link_data_[i+1]->orientation_;
  }
  getPhiAngle();
  manipulator_link_data_[END_LINK]->euler = rotation2rpy(manipulator_link_data_[8]->orientation_);
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobian(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd tar_position = manipulator_link_data_[idx[end]]->position_;
  Eigen::MatrixXd Jacobian = Eigen::MatrixXd::Zero(6, idx_size);

  for (int index = 0; index < idx_size; index++)
  {
    int id = idx[index];

    Eigen::MatrixXd tar_orientation = manipulator_link_data_[id]->orientation_ * manipulator_link_data_[id]->joint_axis_;

    Jacobian.block(0, index, 3, 1) = robotis_framework::calcCross(tar_orientation,
                                                                  tar_position - manipulator_link_data_[id]->position_);
    Jacobian.block(3, index, 3, 1) = tar_orientation;
  }

  return Jacobian;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcJacobianCOM(std::vector<int> idx)
{
  int idx_size = idx.size();
  int end = idx_size - 1;

  Eigen::MatrixXd target_position = manipulator_link_data_[idx[end]]->position_;
  Eigen::MatrixXd jacobianCOM = Eigen::MatrixXd::Zero(6, idx_size);

  for (int index = 0; index < idx_size; index++)
  {
    int     id    = idx[index];
    double  mass  = totalMass(id);

    Eigen::MatrixXd og = calcMC(id) / mass - manipulator_link_data_[id]->position_;
    Eigen::MatrixXd target_orientation = manipulator_link_data_[id]->orientation_ * manipulator_link_data_[id]->joint_axis_;

    jacobianCOM.block(0, index, 3, 1) = robotis_framework::calcCross(target_orientation, og);
    jacobianCOM.block(3, index, 3, 1) = target_orientation;
  }

  return jacobianCOM;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                                                         Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation)
{
  Eigen::MatrixXd pos_err = tar_position - curr_position;
  Eigen::MatrixXd ori_err1 = curr_orientation.inverse() * tar_orientation;
  Eigen::MatrixXd ori_err2 = curr_orientation * robotis_framework::convertRotToOmega(ori_err1);

  Eigen::MatrixXd err = Eigen::MatrixXd::Zero(6, 1);
  err.block(0, 0, 3, 1) = pos_err;
  err.block(3, 0, 3, 1) = ori_err2;

  return err;
}

bool ManipulatorKinematicsDynamics::inverseKinematics(int to, Eigen::MatrixXd tar_position,
    Eigen::MatrixXd tar_orientation, double tar_phi, double tar_slide_pos, bool is_p2p)
{
  bool ik_success     = false;
  bool limit_success  = false;
  
  forwardKinematics(7);

  std::vector<int> idx = findRoute(to);
  Eigen::VectorXd Old_JointAngle(8);
  Eigen::VectorXd angle;

  if(is_p2p)
    Old_JointAngle << 0,0,0,0,0,0,0,0;
  else
  {
    for (int id = 0; id < idx.size(); id++)
    Old_JointAngle[idx[id]] = manipulator_link_data_[idx[id]]->joint_angle_;
  }

  ik_success = inverseKinematics_(tar_position, tar_orientation, tar_phi, tar_slide_pos, Old_JointAngle, is_p2p);
  if( !ik_success )
  {
    Eigen::MatrixXd rpy = rotation2rpy(tar_orientation);
    std::cout<<"IK Failed: pos = "<< tar_position << ", " << rpy << ", " << tar_slide_pos << std::endl;
  }
  forwardKinematics(7);
  
  int joint_num;

  for (int id = 0; id < idx.size(); id++)
  {
     joint_num = idx[id];

    if (manipulator_link_data_[joint_num]->joint_angle_ > manipulator_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;   
      break;
    }
    else if (manipulator_link_data_[joint_num]->joint_angle_ < manipulator_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;   
      break;
    }
    else
      limit_success = true;
  }
  
  if (ik_success == true && limit_success == true)
    return true;
  else if(!limit_success)
  {
    std::cout<<"Out of Joint \""<<joint_num<<"\" limit!!!"<<std::endl;
    return false;
  }
  else
  {
    Eigen::MatrixXd rpy = rotation2rpy(tar_orientation);
    std::cout<<"Failed pose: "<<tar_position<<", "<< rpy << ", " << tar_slide_pos<<std::endl;
    return false;
  }
}

bool ManipulatorKinematicsDynamics::inverseKinematics_test(Eigen::MatrixXd tar_position,
                  Eigen::MatrixXd tar_orientation, double tar_phi, double tar_slide_pos)
{
  bool ik_success     = false;
  bool limit_success  = false;
  
  Eigen::VectorXd Old_JointAngle(8);
  
  Old_JointAngle << 0,0,0,0,0,0,0,0;

  // ik_success = inverseKinematics_old(tar_position, tar_orientation, tar_phi, tar_slide_pos, Old_JointAngle, true, false);
  ik_success = inverseKinematics_(tar_position, tar_orientation, tar_phi, tar_slide_pos, Old_JointAngle, true, false);
  if(!ik_success)
  {
    Eigen::MatrixXd rpy = rotation2rpy(tar_orientation);
    std::cout<<"IK Failed: pos = "<< tar_position << ", " << rpy << ", " << tar_slide_pos << std::endl;
  }
  int joint_num;
  std::vector<int> idx = findRoute(8);

  for (int id = 0; id < idx.size(); id++)
  {
     joint_num = idx[id];

    if (manipulator_link_data_[joint_num]->joint_angle_test > manipulator_link_data_[joint_num]->joint_limit_max_)
    {
      limit_success = false;   
      break;
    }
    else if (manipulator_link_data_[joint_num]->joint_angle_test < manipulator_link_data_[joint_num]->joint_limit_min_)
    {
      limit_success = false;   
      break;
    }
    else
      limit_success = true;
  }
  
  if (ik_success == true && limit_success == true)
    return true;
  else if(!limit_success)
  {
    std::cout<<"Out of Joint \""<<joint_num<<"\" limit!!!"<<std::endl;
    return false;
  }
  else
    return false;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::transform( double &Theta, Eigen::VectorXd &DH )
{
  double c_th = cos(Theta + DH(3));
  double s_th = sin(Theta + DH(3));
  double c_alp = cos(DH(1));
  double s_alp = sin(DH(1));
  Eigen::MatrixXd A(4,4);
  A << c_th, -s_th*c_alp,  s_th*s_alp,  DH(0)*c_th,
       s_th,  c_th*c_alp, -c_th*s_alp,  DH(0)*s_th,
       0,     s_alp,       c_alp,       DH(2),
       0,     0,           0,           1;  
  return A;
}
Eigen::Vector3d ManipulatorKinematicsDynamics::forwardKinematics_7(int joint_ID, Eigen::VectorXd angle)
{
  Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4,4);
  Eigen::MatrixXd A(4,4);
  Eigen::VectorXd DH_row(4);

  for ( int i=0; i<=joint_ID; i++ )
  {
    DH_row = DHTABLE.row(i);
    A = transform(angle(i), DH_row);
    T = T*A;
  }
  return T.block(0,3,3,1);
}

bool ManipulatorKinematicsDynamics::inverseKinematics_( Eigen::VectorXd goal_position, Eigen::Matrix3d rotation, double Phi, 
                                                           double slide_position, Eigen::VectorXd Old_JointAngle, bool is_p2p, bool is_run)
{
  bool ik_success = false;
  manipulator_link_data_[0]->singularity_ = false;

  int isMatch;
  double theta_e, modify_euler_theta;
  double theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7;
  double Deviation, D_Joint_1, D_Joint_2, D_Joint_1_2, D_Joint_2_2;
  double Lsw, Lec, Lsc;
  double theta1_start = manipulator_link_data_[1]->joint_angle_;
  double theta5_start = manipulator_link_data_[5]->joint_angle_;
  
  Eigen::VectorXd Angle(5, 1);
  Eigen::VectorXd Oc(3, 1);
  Eigen::VectorXd testOc(3, 1);
  Eigen::VectorXd Ps(3, 1);
  Eigen::VectorXd Vsw(3, 1);
  Eigen::VectorXd eRc(3, 1);
  Eigen::VectorXd Distance(8, 1);

  Eigen::MatrixXd A(4,4);
  Eigen::MatrixXd T(4,4);
  Eigen::MatrixXd DH(5, 4);
  Eigen::VectorXd DH_row(4);
  Eigen::Matrix3d Modify_euler;

  modify_euler_theta = (-pi/2)*RL_prm;
  Modify_euler << cos(modify_euler_theta), -sin(modify_euler_theta), 0,
                  sin(modify_euler_theta),  cos(modify_euler_theta), 0,
                  0,                        0,                       1;

  R07 = Eigen::MatrixXd::Identity(4,4);
  R07.block(0,0,3,3) = rotation*Modify_euler;
  R07.block(0,3,3,1) = goal_position;
  Oc << goal_position(0)-d4*R07(0,2), goal_position(1)-d4*R07(1,2), goal_position(2)-d4*R07(2,2);

  DHTABLE(0,2) = slide_position;
  ROS_INFO("DHTABLE(0,2) in IK begin: %f", DHTABLE(0,2));

  Ps << 0, d1*cos(DHTABLE(0,3)), slide_position;   
  Vsw = Oc - Ps;
  Lsw = Vsw.norm();   //肩腕
  theta_e = acos((Lse*Lse + Lsw*Lsw - Lew*Lew) / (2*Lse*Lsw));  //Lse 肩肘 Lew 肘腕
  eRc = Ps + (Vsw * Lse * cos(theta_e) / Lsw);    //Phi旋轉中心位置
  Lsc = Lse * cos(theta_e); 
  Lec = Lse * sin(theta_e); 

  DH << 0,   -pi/2, slide_position, DHTABLE(0,3),    
        0,   -pi/2, d1,             pi/2,
        0,   -pi/2, 0,             -pi/2, 
        Lec,  pi/2, Lsc,            0,
        0,    0,    0,              0;
  double theta1_tmp = atan2(-RL_prm *Oc(0) , -(Oc(2)-slide_position));
  theta_1 = atan2(-RL_prm *eRc(0) , -(eRc(2)-slide_position));
  theta_2 = asin((d1 - RL_prm *eRc(1)) / Lsc);        
  theta_3 = Phi;
  theta_4 = -(theta_e + atan(a1/d2));

  double theta7_tmp;

  Angle << 0, theta_1, theta_2, theta_3, theta_4;
  Eigen::VectorXd tmpAngle = Angle;
  T = Eigen::MatrixXd::Identity(4,4);
  for ( int i=0; i<5; i++ )
  {
    DH_row = DH.row(i);
    A = transform( Angle(i), DH_row );
    T = T*A;
  }
  R03 = T;


  theta_4 = M_PI - acos((Lse*Lse + Lew*Lew - Lsw*Lsw) / (2*Lse*Lew)) + atan(a1/d2) + atan(a2/d3);
  
  DH_row = DHTABLE.row(4);
  A = transform(theta_4, DH_row);   
  R04 = R03 * A;

  theta_1 = atan((-RL_prm *R03(0,1)) / -R03(2,1));
  theta_2 = asin(-RL_prm *R03(1,1));
  theta_3 = atan( (RL_prm *R03(1,2)) / (RL_prm *R03(1,0)));

  R47 = R04.inverse() * R07;

  theta_5 = atan(-R47(1,2) / -R47(0,2));
  theta_6 = acos(R47(2,2));
  theta_7 = atan(-R47(2,1) / R47(2,0));

  bool AA = false;
  bool BB = false;
  double dis1 = 0;
  double dis2 = 0;
  double dis3 = 0;
  double dis4 = 0;
  double dis5 = 0;
  double dis6 = 0;
  for ( int i = 1; i>= -1; i-=2 )
  { 
    bool theta_1_flag = false;
    double t_3;
    if( -R03(2,1)*i>= 0)
    {  
        JointAngle(1) = theta_1;
        theta_1_flag = true;
    }
    else if(theta_1>=0)  
        JointAngle(1) = theta_1 - pi;
    else                                 
        JointAngle(1) = pi + theta_1;
    
    if ( i == 1 )
        JointAngle(2) = theta_2;
    else
        JointAngle(2) = (theta_2/fabs(theta_2))*(pi - fabs(theta_2));    
    if( RL_prm *R03(1,0)*i >= 0){                     
        JointAngle(3) = theta_3;
        t_3 = 0;
    }
    else if(theta_3>=0){
        JointAngle(3) = theta_3 - pi;
        t_3 = pi;
    }
    else{
        JointAngle(3) = pi + theta_3;
        t_3 = pi;
    }
    if(i==1)
    {
      Angle.resize(JointAngle.size());
      Angle = JointAngle;
    }
    else
    {
      dis1 = fabs(Angle(1) - theta1_tmp);
      dis2 = fabs(JointAngle(1) - theta1_tmp);
      dis3 = fabs(Angle(1) - theta1_start);
      dis4 = fabs(JointAngle(1) - theta1_start);
      if(fabs(2*M_PI-dis1)<0.00001)
        dis1 = 0;
      if(fabs(2*M_PI-dis2)<0.00001)
        dis2 = 0;
      if((dis1<dis2 && dis4>M_1_PI) || dis3<M_1_PI)
      {
        JointAngle = Angle;
        AA = true;
      }
    }
  }

  JointAngle(4) = theta_4;

  for ( int i = 1; i>=-1; i-=2 ) 
  {
    bool theta_5_flag = false;
    if(-R47(0,2)*i>=0)
    {
      JointAngle(5) = theta_5;
      theta_5_flag = true;
    }
    else if(theta_5>=0)  
      JointAngle(5) = theta_5 - pi;
    else
      JointAngle(5) = pi + theta_5; 
    
    if (i == 1)
      JointAngle(6) = theta_6;
    else
      JointAngle(6) = -theta_6;
  
    if(R47(2,0)*i>=0)
      JointAngle(7) = theta_7;
    else if(theta_7>=0)
      JointAngle(7) = theta_7 - pi;
    else
      JointAngle(7) = pi + theta_7;

    if(i==1)
    {
      Angle.resize(JointAngle.size());
      Angle = JointAngle;
    }
    else
    {
      dis5 = fabs(Angle(5) - theta5_start/2);
      dis6 = fabs(JointAngle(5) - theta5_start/2);
      if(dis5<dis6)
      {
        JointAngle = Angle;
        BB = true;
      }
    }    
  }
  Angle = JointAngle;
  Eigen::VectorXd testPos = forwardKinematics_7(7,JointAngle);
  testPos = testPos - goal_position;
  Deviation = testPos.norm();
  if ( Deviation < 0.001 )
  {
    ik_success = true;
  }
  else
  {
    if(AA)
    {
      if(BB)
        ROS_ERROR("[FUCK IKFIAL] Solution A1B1");
      else
        ROS_ERROR("[FUCK IKFIAL] Solution A1B2");
    }
    else
    {
      if(BB)
        ROS_ERROR("[FUCK IKFIAL] Solution A2B1");
      else
        ROS_ERROR("[FUCK IKFIAL] Solution A2B2");
    }
  }

  if(!is_p2p && ik_success) //Tough to do singularity
  {
    //=========Not sure if this is necessary==============
    // if(fabs(JointAngle(2)-(pi/2)) < 0.05) 
    // {
    //   JointAngle(1) = Old_JointAngle(1);
    //   JointAngle(3) = Old_JointAngle(3);
    // }
    // if(fabs(JointAngle(6)) < 0.05)
    // {
    //   JointAngle(5) = Old_JointAngle(5);
    //   JointAngle(7) = Old_JointAngle(7);
    // }
    //=====================================================comment for training
    double dis_max = 0;
    double move_max = 0.01 + 0.04 * (manipulator_link_data_[0]->mov_speed_ / 100);
    Distance = JointAngle - Old_JointAngle;
    Distance(2) *= 3;
    Distance(4) *= 3;
    Distance(6) *= 3;
    dis_max = Distance.block(0,0,1,7).cwiseAbs().maxCoeff();
    Distance(2) /= 3;
    Distance(4) /= 3;
    Distance(6) /= 3;
    // if(fabs(Distance(5)) > 0.1) //Reduce end point displacemen
    //   JointAngle(6) = 0;comment for training
    if(dis_max > move_max)
    {
      manipulator_link_data_[0]->singularity_ = true;
      for (int id = 1; id <= MAX_JOINT_ID; id++)
      {
        JointAngle(id) = Old_JointAngle(id) + Distance(id)*(move_max/dis_max);
      }
    }
    else
      manipulator_link_data_[0]->singularity_ = false;
  }
  if(ik_success && is_run)
  {
    manipulator_link_data_[END_LINK]->phi_ = Phi;
    manipulator_link_data_[0]->slide_position_ = slide_position;
    for (int id = 0; id <= MAX_JOINT_ID; id++)
      manipulator_link_data_[id]->joint_angle_ = JointAngle.coeff(id);
  }
  else if(ik_success && !is_run)
  {
    for (int id = 0; id <= MAX_JOINT_ID; id++)
      manipulator_link_data_[id]->joint_angle_test = JointAngle.coeff(id);
  }
  ROS_INFO("DHTABLE(0,2) in IK begin: %f", DHTABLE(0,2));
  return ik_success;
}

bool ManipulatorKinematicsDynamics::inverseKinematics_old( Eigen::VectorXd goal_position, Eigen::Matrix3d rotation, 
                                                            double Phi, double slide_position, Eigen::VectorXd Old_JointAngle, bool is_p2p, bool is_run)
{
  bool ik_success = false;

  int isMatch;
  double theta_e, modify_euler_theta, tmp_slide_position;
  double theta_1, theta_2, theta_3, theta_4, theta_5, theta_6, theta_7;
  double Deviation, D_Joint_1, D_Joint_2, D_Joint_1_2, D_Joint_2_2;
  double Lsw, Lec, Lsc;
  
  Eigen::VectorXd Angle(5, 1);
  Eigen::VectorXd Oc(3, 1);
  Eigen::VectorXd testOc(3, 1);
  Eigen::VectorXd Ps(3, 1);
  Eigen::VectorXd Vsw(3, 1);
  Eigen::VectorXd eRc(3, 1);
  Eigen::VectorXd Distance(8, 1);

  Eigen::MatrixXd A(4,4);
  Eigen::MatrixXd T(4,4);
  Eigen::MatrixXd DH(5, 4);
  Eigen::VectorXd DH_row(4);
  Eigen::Matrix3d Modify_euler;
  Eigen::MatrixXd DHTABLE_IK = DHTABLE;

  Eigen::VectorXd JointAngle     = Eigen::MatrixXd::Zero(8, 1);
  Eigen::VectorXd tmp_JointAngle = Eigen::MatrixXd::Zero(8, 1);
  Eigen::MatrixXd R03 = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd R04 = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd R07 = Eigen::MatrixXd::Zero(4, 4);
  Eigen::MatrixXd R47 = Eigen::MatrixXd::Zero(4, 4);

  modify_euler_theta = (-pi/2)*RL_prm;
  Modify_euler << cos(modify_euler_theta), -sin(modify_euler_theta), 0,
                  sin(modify_euler_theta),  cos(modify_euler_theta), 0,
                  0,                        0,                       1;

  R07 = Eigen::MatrixXd::Identity(4,4);
  R07.block(0,0,3,3) = rotation*Modify_euler;
 
  Oc << goal_position(0)-d4*R07(0,2), goal_position(1)-d4*R07(1,2), goal_position(2)-d4*R07(2,2);
  tmp_slide_position = DHTABLE(0,2);
  DHTABLE(0,2) = slide_position;    
  DHTABLE_IK(0,2) = slide_position; 
          
  ////////////////////////////////////////////////////////////////////////////////////////////////

  Ps << 0, d1*cos(DHTABLE_IK(0,3)), slide_position;   
  Vsw = Oc - Ps;     
  Lsw = Vsw.norm();  
  theta_e = acos((Lse*Lse + Lsw*Lsw - Lew*Lew) / (2*Lse*Lsw));  
  eRc = Ps + (Vsw * Lse * cos(theta_e) / Lsw);    //Phi旋轉中心位置
  //////////////////////////////////////////////////////////////////////////////////////////
  Lsc = Lse * cos(theta_e); 
  Lec = Lse * sin(theta_e); 

  DH << 0,   -pi/2, slide_position, DHTABLE_IK(0,3),    
        0,   -pi/2, d1,             pi/2,
        0,   -pi/2, 0,             -pi/2, 
        Lec,  pi/2, Lsc,            0,    
        0,    0,    0,              0;
  
  theta_1 = atan2(-RL_prm *eRc(0) , -(eRc(2)-slide_position));   
  theta_2 = asin((d1 - RL_prm *eRc(1)) / Lsc);        
  theta_3 = Phi;
  theta_4 = -(theta_e + atan(a1/d2));
 
  Angle << 0, theta_1, theta_2, theta_3, theta_4;
  T = Eigen::MatrixXd::Identity(4,4);

  for ( int i=0; i<5; i++ )
  {
    DH_row = DH.row(i);
    A = transform( Angle(i), DH_row );
    T = T*A;
  }
  R03 = T;

 
  theta_4 = M_PI - acos((Lse*Lse + Lew*Lew - Lsw*Lsw) / (2*Lse*Lew)) + atan(a1/d2) + atan(a2/d3);
  
  DH_row = DHTABLE_IK.row(4);
  A = transform(theta_4, DH_row);   
  R04 = R03 * A;

  JointAngle << 0, 0, 0, 0, 0, 0, 0, 0;

  theta_1 = atan((-RL_prm *R03(0,1)) / -R03(2,1));
  theta_2 = asin(-RL_prm *R03(1,1));
  theta_3 = atan( (RL_prm *R03(1,2)) / (RL_prm *R03(1,0)));

  R47 = R04.inverse() * R07;

  theta_5 = atan(-R47(1,2) / -R47(0,2));
  theta_6 = acos(R47(2,2));
  theta_7 = atan(-R47(2,1) / R47(2,0));
  ////////////////////////////////////////////////////////////////////////////////////////////
  isMatch = 0;
  D_Joint_1 = -1;
  D_Joint_2 = -1;
  D_Joint_1_2 = -1;
  D_Joint_2_2 = -1;

  for ( int i = 1; i>= -1; i-=2 )
  { 
    if( -R03(2,1)*i>= 0)  
        JointAngle(1) = theta_1;
    else if( -R03(2,1)*i<0 && theta_1>=0 )  
        JointAngle(1) = theta_1 - pi;
    else                                 
        JointAngle(1) = pi + theta_1;
    
    if ( i == 1 )
        JointAngle(2) = theta_2;
    else
        JointAngle(2) = (theta_2/fabs(theta_2))*(pi - fabs(theta_2));    
    if( RL_prm *R03(1,0)*i >= 0)                         
        JointAngle(3) = theta_3;
    else if( RL_prm *R03(1,0)*i<0 && theta_3>=0)       
        JointAngle(3) = theta_3 - pi;
    else
        JointAngle(3) = pi + theta_3;
    

    JointAngle(4) = theta_4;

    Eigen::VectorXd testPos = forwardKinematics_7(5, JointAngle);

    double tmp_joint1;
    tmp_joint1 = JointAngle(1);
    if(JointAngle(1)*Old_JointAngle(1)<0 && fabs(Old_JointAngle(1))>3)
    {
        JointAngle(1) = (JointAngle(1)>Old_JointAngle(1))?-pi:pi;
    }
    
    testPos = testPos - Oc;
   
    Deviation = testPos.norm();
    if ( Deviation < 0.0001 )
    {
        isMatch = isMatch + 1;
        if( i == 1 )
        {
            D_Joint_1 = fabs(JointAngle(3) - Old_JointAngle(3));
            D_Joint_1_2 = (JointAngle.block(0, 1, 1, 2) - Old_JointAngle.block(0, 1, 1, 2)).cwiseAbs().maxCoeff();
            JointAngle(1) = tmp_joint1;
            tmp_JointAngle = JointAngle;
        }        
        else
        {
            D_Joint_2 = fabs(JointAngle(3) - Old_JointAngle(3));
            D_Joint_2_2 = (JointAngle.block(0, 1, 1, 2) - Old_JointAngle.block(0, 1, 1, 2)).cwiseAbs().maxCoeff();
            JointAngle(1) = tmp_joint1;
        }
    }

    if(isMatch == 2)
    {
        D_Joint_1 = (fabs(D_Joint_1 - D_Joint_2)>0.01) ? D_Joint_1 : D_Joint_1_2;
        D_Joint_2 = (fabs(D_Joint_1 - D_Joint_2)>0.01) ? D_Joint_2 : D_Joint_2_2;
    } 
    if( isMatch == 0 && i == -1)
    {
        JointAngle = Old_JointAngle;
        // std::cout<<"====D_Joint_1==== "<<std::endl<<D_Joint_1<<std::endl;
        // std::cout<<"====D_Joint_2==== "<<std::endl<<D_Joint_2<<std::endl;
        // std::cout<<"====JointAngle====="<<std::endl<<JointAngle<<std::endl;
        std::cout<<"No solution 1 !!!"<<std::endl;
        ik_success = false;
    }
    else if( isMatch == 1 && D_Joint_2 == -1)
    {
        JointAngle = tmp_JointAngle;
    }
    else if( isMatch == 2 && D_Joint_1 < D_Joint_2 )
    {
        JointAngle = tmp_JointAngle;
    }
  }

  isMatch = 0;
  D_Joint_1 = -1;
  D_Joint_2 = -1;
  D_Joint_1_2 = -1;
  D_Joint_2_2 = -1;
  tmp_JointAngle = JointAngle;

  for ( int i = 1; i>=-1; i-=2 ) 
  {
    if( -R47(0,2)*i>=0 )
        JointAngle(5) = theta_5;
    else if( -R47(0,2)*i<0 && theta_5>=0 )  
        JointAngle(5) = theta_5 - pi;
    else
        JointAngle(5) = pi + theta_5;   
    
    if (i == 1)
        JointAngle(6) = theta_6;
    else
        JointAngle(6) = -theta_6;
    
            
    if( R47(2,0)*i>=0 )
        JointAngle(7) = theta_7;
    else if( R47(2,0)*i<0 && theta_7>=0 )
        JointAngle(7) = theta_7 - pi;
    else
        JointAngle(7) = pi + theta_7;
    double dof2 = 8;

    Eigen::VectorXd testPos = forwardKinematics_7(7, JointAngle);
  
    testPos = testPos - goal_position;

    if(JointAngle(1)*Old_JointAngle(1)<0 && fabs(Old_JointAngle(1))>3)
    {
      JointAngle(1) = (JointAngle(1)>Old_JointAngle(1))?-pi:pi;
    }
    // if(JointAngle(7)*Old_JointAngle(7)<0 && fabs(JointAngle(7) - Old_JointAngle(7)) > pi)
    // {
    //   // JointAngle(7) = (JointAngle(7)>Old_JointAngle(7))?(JointAngle(7)-2*pi):(JointAngle(7)+2*pi);
    //   JointAngle(7) = (JointAngle(7)>Old_JointAngle(7))?-pi:pi;
    // }
    // std::cout<<"====JointAngle567===== "<<JointAngle(5)<<" "<<JointAngle(6)<<" "<<JointAngle(7)<<std::endl;
    Eigen::VectorXd Joint_dev(8);
    Deviation = testPos.norm();
    if ( Deviation < 0.0001 )
    {
        isMatch = isMatch + 1;
        if( i == 1 )
        {
            D_Joint_1 = fabs(JointAngle(5) - Old_JointAngle(5));
            D_Joint_1_2 = (JointAngle.block(0, 6, 1, 2) - Old_JointAngle.block(0, 6, 1, 2)).cwiseAbs().maxCoeff();
            tmp_JointAngle = JointAngle;
        }
        else
        {
            D_Joint_2 = fabs(JointAngle(5) - Old_JointAngle(5));
            D_Joint_2_2 = (JointAngle.block(0, 6, 1, 2) - Old_JointAngle.block(0, 6, 1, 2)).cwiseAbs().maxCoeff();
        }
    }
    if(isMatch == 2)
    {
        D_Joint_1 = (fabs(D_Joint_1 - D_Joint_2)>0.01) ? D_Joint_1 : D_Joint_1_2;
        D_Joint_2 = (fabs(D_Joint_1 - D_Joint_2)>0.01) ? D_Joint_2 : D_Joint_2_2;
    } 
    if( isMatch == 0 && i == -1)
    {
        JointAngle = Old_JointAngle;
        // std::cout<<"====D_Joint_1===== "<<std::endl<<D_Joint_1<<std::endl;
        // std::cout<<"====D_Joint_2===== "<<std::endl<<D_Joint_2<<std::endl;
        // std::cout<<"====JointAngle====="<<std::endl<<JointAngle<<std::endl;
        std::cout<<"No solution 2 !!!"<<std::endl;
        ik_success = false;
    }
    else if( isMatch == 1 && D_Joint_2 == -1)
    {
        JointAngle = tmp_JointAngle;
        ik_success = true;
    }
    else if( isMatch == 2 && D_Joint_1 < D_Joint_2 )
    {
        JointAngle = tmp_JointAngle;
        ik_success = true;
    }
  }
  // if(D_Joint_1>2 && D_Joint_2>2)
  // {
  //   std::cout<<"====D_Joint_1===== "<<std::endl<<D_Joint_1<<std::endl;
  //   std::cout<<"====D_Joint_2===== "<<std::endl<<D_Joint_2<<std::endl;
  //   std::cout<<"==OLD_JointAngle==="<<std::endl<<Old_JointAngle<<std::endl;
  //   std::cout<<"====JointAngle====="<<std::endl<<JointAngle<<std::endl;
  // }
  // std::cout<<"====JointAngle5====="<<std::endl<<JointAngle(5)<<" ";
  if(!is_p2p)
  {
    if(fabs(JointAngle(2)-(pi/2)) < 0.05)
    {
      JointAngle(1) = Old_JointAngle(1);
      JointAngle(3) = Old_JointAngle(3);
    }
    if(fabs(JointAngle(6)) < 0.05)
    {
      JointAngle(5) = Old_JointAngle(5);
      JointAngle(7) = Old_JointAngle(7);
    }
    double dis_max = 0;
    double move_max = 0.01 + 0.04 * (manipulator_link_data_[0]->mov_speed_ / 100);
    Distance = JointAngle - Old_JointAngle;
    Distance(2) *= 3;
    Distance(4) *= 3;
    Distance(6) *= 3;
    dis_max = Distance.cwiseAbs().maxCoeff();
    Distance(2) /= 3;
    Distance(4) /= 3;
    Distance(6) /= 3;
    if(fabs(Distance(5)) > 0.1)
      JointAngle(6) = 0;
    if(dis_max > move_max)
    {
      manipulator_link_data_[0]->singularity_ = true;
      // std::cout<<"====JointAngle5====="<<JointAngle(5)<<" "<<dis_max<<std::endl;
      for (int id = 1; id <= MAX_JOINT_ID; id++)
      {
        JointAngle(id) = Old_JointAngle(id) + Distance(id)*(move_max/dis_max);
      }
    }
    else
      manipulator_link_data_[0]->singularity_ = false;
    
    // for (int id = 1; id <= MAX_JOINT_ID; id++)
    // {
    //   Distance(id) = JointAngle(id)-Old_JointAngle(id);
    //   dis_max = (fabs(Distance(id)) > dis_max) ? fabs(Distance(id) : dis_max;

    //   Deviation = JointAngle(id)-Old_JointAngle(id);
    //   if(fabs(Deviation) > 0.2)
    //   {
    //     std::cout<<"====111Modify111====="<<Deviation<<std::endl;
    //     JointAngle(id) = Old_JointAngle(id) + Deviation/fabs(Deviation)/5000;
    //   }
    //   else if(fabs(Deviation) > 0.05)
    //   {
    //     std::cout<<"====222Modify222====="<<id<<" "<<Deviation<<std::endl;
    //     // JointAngle(id) = Old_JointAngle(id) + Deviation/fabs(Deviation)*0.05;
    //   }
    // }
  }
  // for (int id = 0; id <= MAX_JOINT_ID; id++){
  //   manipulator_link_data_[id]->joint_angle_ = JointAngle.coeff(id);
  // }
  if(ik_success && is_run)
  {
    manipulator_link_data_[END_LINK]->phi_ = Phi;
    // manipulator_link_data_[0]->slide_position_ = slide_position;
    std::cout<<"joint_angle_:";
    for (int id = 0; id <= MAX_JOINT_ID; id++)
    {
      manipulator_link_data_[id]->joint_angle_ = JointAngle.coeff(id);
      std::cout<<manipulator_link_data_[id]->joint_angle_<<", ";
    }
      std::cout<<"slide_position: "<<slide_position<<std::endl;
  }
  else if(!is_run)
  {
    for (int id = 0; id <= MAX_JOINT_ID; id++)
      manipulator_link_data_[id]->joint_angle_test = JointAngle.coeff(id);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////
  DHTABLE(0,2) = tmp_slide_position;
  return ik_success;
}

bool ManipulatorKinematicsDynamics::slideInverseKinematics(Eigen::Vector3d& goal_position, Eigen::Matrix3d& rotation, 
                                                            double& slide_pos, double& goal_slide_pos, bool is_p2p)
{
  /*
  Lgs:goal pos to slide line segment
  Lgl:goal pos to slide line
  Lmax:max length of arm
  Lmin:min length of arm
  Smax:max pos of slide
  Smin:min pos of slide
  Sz:target slide pos
  Zmax:z value when theta_max
  Zmin:z value when theta_min
  */
  double slide_position;
  double Lsw, Lgs, Lgl, Lmax, Lmin, Smax, Smin, Zmax, Zmin, fun1, fun2, weight, Zup, Zdown;
  Eigen::Vector3d Oc;
  Eigen::Vector3d test_pos;
 
  Oc << goal_position(0)-d4*rotation(0,2), goal_position(1)-d4*rotation(1,2), goal_position(2)-d4*rotation(2,2);
  Smin = -0.8;
  Smax = 0;
  test_pos = Oc;
  test_pos(1) = test_pos(1) - (d1*RL_prm);
  test_pos(2) = (test_pos(2) <= Smax && test_pos(2) >= Smin) ? Smax : (test_pos(2) < Smin) ? test_pos(2) - Smin : test_pos(2) - Smax;
  Lgs = test_pos.norm();
  test_pos(2) = 0;
  Lgl = test_pos.norm();
  Lmax = d2+d3 - 0.01;
  Lmin = 0.15;
  Smax = 0;
  Smin = -0.8;
  ROS_INFO("Lgs in slide IK: %f", Lgs);
  if(Lgs > d2+d3-0.01)
  {
    std::cout<<"Out of range 1 !!!"<<std::endl;
    return false;
  }

  if(Lgs >= Lmin)
  {
    if(Smin <= Oc(2) && Oc(2) <= Smax)
    {
      Zmax = Oc(2) + sqrt(Lmax*Lmax - Lgs*Lgs);
      Zmin = Oc(2);
    }
    else if(Smax < Oc(2))
    {
      Zmax = Oc(2) - sqrt(Lmax*Lmax - Lgl*Lgl);
      Zmin = Smax;
    }
    else
    {
      Zmax = Oc(2) + sqrt(Lmax*Lmax - Lgl*Lgs);
      Zmin = Smin;
    }
  }
  else
  {
    if(Smin <= Oc(2) && Oc(2) <= Smax)
    {
      Zmin = Oc(2) + sqrt(Lmin*Lmin - Lgs*Lgs);
      if(Zmin > Smax)
      {
        Zmax = Oc(2) - sqrt(Lmax*Lmax - Lgs*Lgs);
        Zmin = Oc(2) - sqrt(Lmin*Lmin - Lgs*Lgs);
      }
      else
        Zmax = Oc(2) + sqrt(Lmax*Lmax - Lgs*Lgs);
    }
    else if(Smax < Oc(2))
    {
      Zmax = Oc(2) - sqrt(Lmax*Lmax - Lgl*Lgl);
      Zmin = Oc(2) - sqrt(Lmin*Lmin - Lgl*Lgl);
    }
    else
    {
      Zmax = Oc(2) + sqrt(Lmax*Lmax - Lgl*Lgl);
      Zmin = Oc(2) + sqrt(Lmin*Lmin - Lgl*Lgl);
    }
  }
  Zmax = (Zmax <= Smax) ? (Zmax >= Smin) ? Zmax : Smin : Smax;
  Zmin = (Zmin <= Smax) ? (Zmin >= Smin) ? Zmin : Smin : Smax;
  Zup = (Zmax > Zmin) ? Zmax : Zmin;
  Zdown = (Zmax > Zmin) ? Zmin : Zmax;
  fun1 = Lgs * (Zmin - Zmax) / Lmax + Zmax;
  fun2 = -1 * fabs(Oc(2)) * ((Zup + Zdown) / 2 - Zup) / (Lmax - Smin) + (Zup + Zdown) / 2;
  weight = Lgs / Lmax;
  slide_position = weight * fun1 + (1-weight) * fun2;
  goal_slide_pos = (slide_position <= Smax) ? (slide_position >= Smin) ? slide_position : Smin : Smax;
  ROS_INFO("goal_slide_pos in slide IK: %f", goal_slide_pos);
  if(fabs(goal_slide_pos - slide_pos) > 0.001 && !is_p2p)
    goal_slide_pos = slide_pos + 0.001 * (goal_slide_pos - slide_pos > 0) ? 1 : -1;
  if(slide_position > Smax || slide_position < Smin)
  {
    std::cout<<"Out of range 1, Slide IK FAIL !!!"<<std::endl;
    return false;
  }
  return true;
}

//Old Slide Inverse Kinematics
// bool ManipulatorKinematicsDynamics::slideInverseKinematics(Eigen::Vector3d& goal_position, Eigen::Matrix3d& rotation, 
//                                                             double& slide_pos, double& goal_slide_pos)
// {
//   double slide_position;
//   double Lsw;
//   Eigen::Vector3d Oc;
//   Eigen::Vector3d test_pos;
 
//   Oc << goal_position(0)-d4*rotation(0,2), goal_position(1)-d4*rotation(1,2), goal_position(2)-d4*rotation(2,2);
  
//   test_pos = Oc;
//   test_pos(1) = test_pos(1) - (d1*RL_prm);
//   test_pos(2) = test_pos(2) - slide_pos;
//   Lsw = test_pos.norm();
//   if(Lsw > (d2+d3-0.03) || Lsw < 0.3 || (slide_pos - Oc(2)) < 0.3) 
//   {
//     if(Oc(2) < -0.8){                                         
//         test_pos << test_pos(0), test_pos(1), Oc(2)-(-0.8);
//     }
//     else if(Oc(2) > 0){
//       test_pos << test_pos(0), test_pos(1), Oc(2);
//     }
//     else{
//         test_pos << test_pos(0), test_pos(1), 0;
//     }
//     if(test_pos.norm() > (d2+d3-0.01))
//     {
//       std::cout<<"Out of range 1 !!!"<<std::endl;
//       // std::cout<<"Oc(2) = "<<Oc(2)<<std::endl;
//       // std::cout<<"test_pos.norm() = "<<test_pos.norm()<<std::endl;
//       return false;
//     }

//     for( double i = 0; i <= 5; i++ )
//     {
//       if(pow(d2 + d3 - 0.1 + i/50, 2) > pow(test_pos.block(0,0,1,2).norm(), 2))
//       {
//         slide_position = Oc(2) + sqrt(pow(d2 + d3 - 0.1 + i/50, 2) - pow(test_pos.block(0,0,1,2).norm(), 2));
//         break;
//       }
//       if(i==5)
//       {
//         std::cout<<"Out of range 2 !!!"<<std::endl;
//         return false;
//       }
//     }
//     slide_position = (slide_position < -0.8) ? -0.8 : slide_position;
//     slide_position = (slide_position >  0) ?  0 : slide_position;
//     test_pos(2) = Oc(2) - slide_position;

//     if(test_pos.norm() < (d2+d3-0.005) && test_pos.norm() > 0.1)
//     {
//       goal_slide_pos = slide_position;
//       return true;
//     }
//     else
//     {
//       std::cout<<"Out of range 3 !!!"<<std::endl;
//       return false;
//     }
//   }
//   else
//   {
//     goal_slide_pos = slide_pos;
//     return true;
//   }
// }

void ManipulatorKinematicsDynamics::getPhiAngle()
{
  double theta_e;
  double Lsw, Lec, Lsc;
  
  double slide_position = manipulator_link_data_[0]->slide_position_;

  Eigen::VectorXd Angle(5, 1);
  Eigen::VectorXd Oc(3, 1);
  Eigen::VectorXd testOc(3, 1);
  Eigen::VectorXd Ps(3, 1);
  Eigen::VectorXd Vsw(3, 1);
  Eigen::VectorXd eRc(3, 1);

  Eigen::MatrixXd A(4,4);
  Eigen::MatrixXd T(4,4);
  Eigen::MatrixXd test_T(4,4);
  Eigen::MatrixXd DH(5, 4);
  Eigen::VectorXd DH_row(4);
  Eigen::VectorXd elbow_origin(3);
  Eigen::VectorXd elbow_goal(3);

  Oc = manipulator_link_data_[6]->position_;
  Ps = manipulator_link_data_[2]->position_;

  Vsw = Oc - Ps;     
  Lsw = Vsw.norm();  
  theta_e = acos((Lse*Lse + Lsw*Lsw - Lew*Lew) / (2*Lse*Lsw));
  eRc = Ps + (Vsw * Lse * cos(theta_e) / Lsw);    //Phi旋轉中心位置
  Lsc = Lse * cos(theta_e); 
  Lec = Lse * sin(theta_e); 

  DH << 0,   -pi/2, slide_position, DHTABLE(0,3),    
        0,   -pi/2, d1,             pi/2,
        0,   -pi/2, 0,             -pi/2, 
        Lec,  pi/2, Lsc,            0,    
        0,    0,    0,              0;
  Angle(1) = 0;
  Angle(1) = atan2(-RL_prm *eRc(0) , -(eRc(2)-slide_position));   
  Angle(2) = asin((d1 - RL_prm *eRc(1)) / Lsc);         
  Angle(3) = 0;
  Angle(4) = -(theta_e + atan(a1/d2));
 
  T = Eigen::MatrixXd::Identity(4,4);

  for ( int i=0; i<5; i++ )
  {
    DH_row = DH.row(i);
    A = transform( Angle(i), DH_row );
    T = T*A;
    if(i==2)
      test_T = T; 
  }
  elbow_goal = manipulator_link_data_[4]->position_;
  elbow_origin = T.block(0,3,3,1);
  double a = (elbow_goal - elbow_origin).norm();
  double b = (elbow_goal - eRc).norm();
  double c = (elbow_origin - eRc).norm();
  double Phi = acos((b*b + c*c - a*a)/(2*b*c));

  if(manipulator_link_data_[4]->joint_angle_ < 0.2167)
  {
    // Phi = Phi - M_PI;

    // if(manipulator_link_data_[4]->joint_angle_ > 0.2166)
    manipulator_link_data_[END_LINK]->phi_ = 0;
  }
  else
  {
    for(int k=1; k>=-1; k-=2)
    {
      Angle(3) = k * Phi;
      T = test_T;
      for ( int i=3; i<5; i++ )
      {
        DH_row = DH.row(i);
        A = transform( Angle(i), DH_row );
        T = T*A;
      }
      Eigen::Vector3d elbow_test = T.block(0,3,3,1);
      if((elbow_goal - elbow_test).norm() < 0.0001)
        manipulator_link_data_[END_LINK]->phi_ = Angle(3);
    }
  }
}

double ManipulatorKinematicsDynamics::limit_check(Eigen::Vector3d &goal_position, Eigen::Matrix3d &rotation)
{
  double slide_pos = 0;
  double tar_slide_pos = 0;
  Eigen::Vector3d Oc;
  bool slide_success = slideInverseKinematics(goal_position, rotation, slide_pos, tar_slide_pos, true);
  Oc << goal_position(0)-d4*rotation(0,2), goal_position(1)-d4*rotation(1,2), goal_position(2)-d4*rotation(2,2);
  Oc(1) = Oc(1) - (d1*RL_prm);
  Oc(2) = Oc(2) - tar_slide_pos;
  double Lgs = Oc.norm();
  ROS_INFO("Lgs in linit check: %f", Lgs);
  ROS_INFO("tar_slide_pos in linit check: %f", tar_slide_pos);
  if(slide_success)
  {
    bool ik_success = inverseKinematics_test(goal_position, rotation, 0, tar_slide_pos);
    if(!ik_success)
      return 10;
    double range = Lgs/(d2+d3);
    return range;
  }
  else
    return 10;
  // double Lsw, Low;
  // double tar_slide_pos = 0;
  // Eigen::Vector3d Oc;
  // Eigen::Vector3d test_pos;
 
  // Oc << goal_position(0)-d4*rotation(0,2), goal_position(1)-d4*rotation(1,2), goal_position(2)-d4*rotation(2,2);
  
  // test_pos = Oc;
  // test_pos(1) = test_pos(1) - (d1*RL_prm);
  // if(Oc(2) < -0.8)
  // {
  //   test_pos(2) += 0.8;
  //   tar_slide_pos = -0.8;
  // }
  // else if (Oc(2) < -0.01)
  // {
  //   test_pos(2) = 0.01;
  //   tar_slide_pos = Oc(2)+0.01;
  // }
  // Oc(2) = 0;
  // Lsw = test_pos.norm();
  // Low = Oc.norm();
  // if(Lsw <= (d2+d3 - 0.01) && Low > 0.15)
  // {
  //   bool ik_success = inverseKinematics_test(goal_position, rotation, 0, tar_slide_pos);
  //   if(!ik_success)
  //     return 10;
  //   double range = Lsw/(d2+d3);
  //   return range;
  // }
  // else
  //   std::cout<<"Out of range !!!"<<Oc<<std::endl;
  // return 10;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::rotationX( double angle )
{
  Eigen::MatrixXd _rotation( 3, 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::rotationY( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
                    0.0,     1.0,     0.0,
              -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::rotationZ( double angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::rotation2rpy( Eigen::MatrixXd rotation )
{
  Eigen::MatrixXd _rpy = Eigen::MatrixXd::Zero( 3 , 1 );

  _rpy.coeffRef(2,0) = asin(rotation.coeff(1,2));
  _rpy.coeffRef(1,0) = atan2(rotation.coeff(0, 2), -rotation.coeff(2, 2));
  _rpy.coeffRef(0,0) = atan2(-rotation.coeff(1, 0), -rotation.coeff(1, 1));
  

  return _rpy;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::Matrix3d origin;
  origin << 1, 0, 0,
            0,-1, 0,
            0, 0,-1;
  // Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );
  Eigen::MatrixXd _rotation = origin * rotationY( pitch ) * rotationX( yaw ) * rotationZ( roll );

  return _rotation;
}

Eigen::Quaterniond ManipulatorKinematicsDynamics::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );


  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond ManipulatorKinematicsDynamics::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd ManipulatorKinematicsDynamics::quaternion2rotation( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

  return _rotation;
}
}
