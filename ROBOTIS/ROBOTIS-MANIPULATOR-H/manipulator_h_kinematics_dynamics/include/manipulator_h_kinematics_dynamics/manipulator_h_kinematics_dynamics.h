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

#ifndef MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_
#define MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_

#include <vector>
#include <cmath>
#include <map>
#include <string>
#include "link_data.h"
#include "manipulator_h_kinematics_dynamics_define.h"

#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#define pi M_PI

namespace robotis_manipulator_h
{

enum TreeSelect
{
  ARM
};

class ManipulatorKinematicsDynamics
{
private:
  Eigen::MatrixXd DHTABLE;
  Eigen::MatrixXd INV_DHTABLE;
  double d1,d2,d3,d4,a1,a2;
  double Lse, Lew;
  double RL_prm;

  Eigen::VectorXd JointAngle;
  Eigen::VectorXd tmp_JointAngle;

  Eigen::MatrixXd R03;
  Eigen::MatrixXd R04;
  Eigen::MatrixXd R07;
  Eigen::MatrixXd R47;
  Eigen::MatrixXd R57;
  Eigen::MatrixXd R72;
  Eigen::MatrixXd R05_notheta3;

public:
  ManipulatorKinematicsDynamics();
  ManipulatorKinematicsDynamics(TreeSelect tree);
  ~ManipulatorKinematicsDynamics();

  LinkData *manipulator_link_data_[ ALL_JOINT_ID + 1];

  std::vector<int> findRoute(int to);
  std::vector<int> findRoute(int from, int to);

  double totalMass(int joint_ID);
  Eigen::MatrixXd calcMC(int joint_ID);
  Eigen::MatrixXd calcCOM(Eigen::MatrixXd MC);

  void forwardKinematics(int joint_ID);

  Eigen::MatrixXd calcJacobian(std::vector<int> idx);
  Eigen::MatrixXd calcJacobianCOM(std::vector<int> idx);
  Eigen::MatrixXd calcVWerr(Eigen::MatrixXd tar_position, Eigen::MatrixXd curr_position,
                            Eigen::MatrixXd tar_orientation, Eigen::MatrixXd curr_orientation);

  bool inverseKinematics(int to, Eigen::MatrixXd tar_position, Eigen::MatrixXd tar_orientation,
                         double tar_phi, double tar_slide_pos, bool is_p2p);
  bool inverseKinematics_test(Eigen::MatrixXd tar_position,
                              Eigen::MatrixXd tar_orientation, double tar_phi, double tar_slide_pos);
  Eigen::MatrixXd transform( double &Theta, Eigen::VectorXd &DH );
  Eigen::Vector3d forwardKinematics_7(int joint_ID, Eigen::VectorXd angle);
  bool inverseKinematics_( Eigen::VectorXd goal_position, Eigen::Matrix3d rotation, 
                              double Phi, double slide_position, Eigen::VectorXd Old_JointAngle, bool is_p2p, bool is_run=true);
  bool inverseKinematics_old( Eigen::VectorXd position, Eigen::Matrix3d rotation, double phi, 
                            double slide_position, Eigen::VectorXd Old_JointAngle, bool is_p2p, bool is_run);
  bool slideInverseKinematics(Eigen::Vector3d& goal_position, Eigen::Matrix3d& rotation, 
                              double& slide_pos, double& goal_slide_pos, bool is_p2p);
  double limit_check(Eigen::Vector3d &goal_position, Eigen::Matrix3d &rotation);
  
  void load_LinkParam();
  void getPhiAngle();
  
  static Eigen::MatrixXd rotationX( double angle );
  static Eigen::MatrixXd rotationY( double angle );
  static Eigen::MatrixXd rotationZ( double angle );
  static Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
  static Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );
  static Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
  static Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );
  static Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
  static Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );

};

}

#endif /* MANIPULATOR_KINEMATICS_DYNAMICS_MANIPULATOR_KINEMATICS_DYNAMICS_H_ */
