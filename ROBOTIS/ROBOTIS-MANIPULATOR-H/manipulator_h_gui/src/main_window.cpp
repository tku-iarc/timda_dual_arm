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

/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "manipulator_h_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace manipulator_h_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  joint_name.push_back("joint5");
  joint_name.push_back("joint6");
  joint_name.push_back("joint7");

  /*********************
    ** Logging
    **********************/
  ui.view_logging->setModel(qnode.loggingModel());
  QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  joint_spinbox.append( ui.joint1_spinbox );
  joint_spinbox.append( ui.joint2_spinbox );
  joint_spinbox.append( ui.joint3_spinbox );
  joint_spinbox.append( ui.joint4_spinbox );
  joint_spinbox.append( ui.joint5_spinbox );
  joint_spinbox.append( ui.joint6_spinbox );
  joint_spinbox.append( ui.joint7_spinbox );

  /****************************
    ** Connect
    ****************************/

  qRegisterMetaType<manipulator_h_base_module_msgs::JointPose>("manipulator_h_base_module_msgs::JointPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentJointPose(manipulator_h_base_module_msgs::JointPose)), this, SLOT(updateCurrJointPoseSpinbox(manipulator_h_base_module_msgs::JointPose)));

  qRegisterMetaType<manipulator_h_base_module_msgs::KinematicsPose>("manipulator_h_base_module_msgs::KinematicsPose");
  QObject::connect(&qnode, SIGNAL(updateCurrentKinematicsPose(manipulator_h_base_module_msgs::GetKinematicsPose)), this, SLOT(updateCurrKinematicsPoseSpinbox(manipulator_h_base_module_msgs::GetKinematicsPose)));

  
  /*********************
    ** Auto Start
    **********************/
  qnode.init();
  setWindowTitle(qnode.getName());

  joy_calib_state = false;
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::relative_movement()
{
  manipulator_h_base_module_msgs::P2PPose msg;

  msg.name = "arm";

  msg.speed           = ui.speed_spinbox->value();
  msg.pose.position.x = this->relative_movement_data[0];
  msg.pose.position.y = this->relative_movement_data[1];
  msg.pose.position.z = this->relative_movement_data[2];
  
  double roll  = this->relative_movement_data[3] * M_PI / 180.0;
  double pitch = this->relative_movement_data[4] * M_PI / 180.0;
  double yaw   = this->relative_movement_data[5] * M_PI / 180.0;
  double phi   = this->relative_movement_data[6] * M_PI / 180.0;
  
  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();
  msg.phi = phi;

  qnode.sendP2PPoseMsg( msg );
  return;
}

// void MainWindow::qnode.getCurrPose(this->relative_movement_data)
// {
//   std::string group_name = "arm";
//   qnode.getKinematicsPose( group_name );

//   this->relative_movement_data[0] = ui.pos_x_spinbox->value();
//   this->relative_movement_data[1] = ui.pos_y_spinbox->value();
//   this->relative_movement_data[2] = ui.pos_z_spinbox->value();
  
//   this->relative_movement_data[3] = ui.ori_roll_spinbox->value();
//   this->relative_movement_data[4] = ui.ori_pitch_spinbox->value();
//   this->relative_movement_data[5] = ui.ori_yaw_spinbox->value();
//   this->relative_movement_data[6] = ui.ori_phi_spinbox->value();

//   return;
// }
void MainWindow::on_curr_joint_button_clicked( bool check )
{
  qnode.getJointPose( joint_name );
}

void MainWindow::on_des_joint_button_clicked( bool check )
{
  manipulator_h_base_module_msgs::JointPose msg;

  for ( int _id = 0; _id < joint_spinbox.size(); _id++ )
  {
    msg.name.push_back( joint_name[ _id ] );
    msg.value.push_back( ((QDoubleSpinBox *) joint_spinbox[ _id ])->value() * M_PI / 180.0 );
  }
  msg.slide_pos = ui.slide_spinbox->value();
  msg.speed     = ui.speed_spinbox->value();
  qnode.sendJointPoseMsg( msg );
}

void MainWindow::on_curr_pos_button_clicked( bool check )
{
  std::string group_name = "arm";

  qnode.getKinematicsPose( group_name );
}

void MainWindow::on_des_pos_button_clicked( bool check )
{
  manipulator_h_base_module_msgs::KinematicsPose msg;

  msg.name = "arm";

  msg.speed           = ui.speed_spinbox->value();
  msg.pose.position.x = ui.pos_x_spinbox->value();
  msg.pose.position.y = ui.pos_y_spinbox->value();
  msg.pose.position.z = ui.pos_z_spinbox->value();

  double roll  = ui.ori_roll_spinbox->value()  * M_PI / 180.0;
  double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
  double yaw   = ui.ori_yaw_spinbox->value()   * M_PI / 180.0;
  double phi   = ui.ori_phi_spinbox->value()   * M_PI / 180.0;
  
  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();
  msg.phi = phi;
  
  qnode.sendKinematicsPoseMsg( msg );
}

void MainWindow::on_des_p2p_button_clicked( bool check )
{
  manipulator_h_base_module_msgs::P2PPose msg;

  msg.name = "arm";

  msg.speed           = ui.speed_spinbox->value();
  msg.pose.position.x = ui.pos_x_spinbox->value();
  msg.pose.position.y = ui.pos_y_spinbox->value();
  msg.pose.position.z = ui.pos_z_spinbox->value();
  
  double roll  = ui.ori_roll_spinbox->value()  * M_PI / 180.0;
  double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
  double yaw   = ui.ori_yaw_spinbox->value()   * M_PI / 180.0;
  double phi   = ui.ori_phi_spinbox->value()   * M_PI / 180.0;
  
  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();
  msg.phi = phi;

  qnode.sendP2PPoseMsg( msg );
}

void MainWindow::on_des_moveit_button_clicked( bool check )
{
  manipulator_h_base_module_msgs::P2PPose msg;

  msg.name = "arm";

  msg.speed           = ui.speed_spinbox->value();
  msg.pose.position.x = ui.pos_x_spinbox->value();
  msg.pose.position.y = ui.pos_y_spinbox->value();
  msg.pose.position.z = ui.pos_z_spinbox->value();
  
  double roll  = ui.ori_roll_spinbox->value()  * M_PI / 180.0;
  double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
  double yaw   = ui.ori_yaw_spinbox->value()   * M_PI / 180.0;
  double phi   = ui.ori_phi_spinbox->value()   * M_PI / 180.0;
  
  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();
  msg.phi = phi;

  qnode.sendMoveItPoseMsg( msg );
}

//=============robotiq_2f_gripper=========================
void MainWindow::on_des_Grap_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="Gripper_grap_Alcohol";

  qnode.sendGrapAlcoholMsg( msg );
}
void MainWindow::on_des_Release_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="Gripper_release";

  qnode.sendReleasePoseMsg( msg );
}
//=============================================================
void MainWindow::on_joy_calib_button_clicked( bool check )
{
  bool cmd;
  cmd = (joy_calib_state) ? false : true;
  joy_calib_state = qnode.joyCalib( cmd );
  if( joy_calib_state )
    ui.joy_calib_button->setText("Stop Calib");
  else
    ui.joy_calib_button->setText("Joy Calib");
}

void MainWindow::on_ini_pose_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="ini_pose";

  qnode.sendSpecificPoseMsg( msg );
}

void MainWindow::on_home_pose_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="home_pose";

  qnode.sendSpecificPoseMsg( msg );
}

void MainWindow::on_set_mode_button_clicked( bool check )
{
  std_msgs::String msg;

  msg.data ="set_mode";

  qnode.sendSetModeMsg( msg );
}

void MainWindow::on_relative_xp_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[0] += ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_xm_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[0] -= ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_yp_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[1] += ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_ym_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[1] -= ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_zp_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[2] += ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_zm_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[2] -= ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_ap_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  double roll  = this->relative_movement_data[3] * DEGREE2RADIAN;
  double pitch = this->relative_movement_data[4] * DEGREE2RADIAN;
  double yaw   = this->relative_movement_data[5] * DEGREE2RADIAN;
  Eigen::MatrixXd rotation = rpy2rotation( roll, pitch, yaw );
  this->relative_movement_data[0] += rotation.coeff( 0,2 ) * ui.relative_movement_spinbox->value();
  this->relative_movement_data[1] += rotation.coeff( 1,2 ) * ui.relative_movement_spinbox->value();
  this->relative_movement_data[2] += rotation.coeff( 2,2 ) * ui.relative_movement_spinbox->value();
  std::cout<<rotation.coeff( 0,2 )<<" "<<rotation.coeff( 1,2 )<<" "<<rotation.coeff( 2,2 )<<std::endl;
  relative_movement();
}
void MainWindow::on_relative_am_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  double roll  = this->relative_movement_data[3] * DEGREE2RADIAN;
  double pitch = this->relative_movement_data[4] * DEGREE2RADIAN;
  double yaw   = this->relative_movement_data[5] * DEGREE2RADIAN;
  Eigen::MatrixXd rotation = rpy2rotation( roll, pitch, yaw );  this->relative_movement_data[0] -= rotation.coeff( 0,2 ) * ui.relative_movement_spinbox->value();
  this->relative_movement_data[1] -= rotation.coeff( 1,2 ) * ui.relative_movement_spinbox->value();
  this->relative_movement_data[2] -= rotation.coeff( 2,2 ) * ui.relative_movement_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_rollp_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[3] += ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_rollm_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[3] -= ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_pitchp_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[4] += ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_pitchm_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[4] -= ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_yawp_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[5] += ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_yawm_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[5] -= ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_phip_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[6] += ui.relative_rotation_spinbox->value();
  relative_movement();
}
void MainWindow::on_relative_phim_button_clicked( bool check )
{
  qnode.getCurrPose(this->relative_movement_data);
  this->relative_movement_data[6] -= ui.relative_rotation_spinbox->value();
  relative_movement();
}

void MainWindow::updateCurrJointPoseSpinbox( manipulator_h_base_module_msgs::JointPose msg )
{
  for ( int _name_index = 0; _name_index < msg.name.size(); _name_index++ )
  {
    for ( int _id = 0; _id < joint_name.size();  _id++ )
    {
      if ( msg.name[ _id ] == joint_name[ _name_index ] )
      {
        ((QDoubleSpinBox *) joint_spinbox[ _name_index ])->setValue( msg.value[ _id ] * 180.0 / M_PI );
        break;
      }
    }
  }
  ui.slide_spinbox->setValue( msg.slide_pos );
}

void MainWindow::updateCurrKinematicsPoseSpinbox( manipulator_h_base_module_msgs::GetKinematicsPose srv )
{
  ui.pos_x_spinbox->setValue( srv.response.group_pose.position.x );
  ui.pos_y_spinbox->setValue( srv.response.group_pose.position.y );
  ui.pos_z_spinbox->setValue( srv.response.group_pose.position.z );
  ui.ori_phi_spinbox->setValue( srv.response.phi * RADIAN2DEGREE );
  // Eigen::Quaterniond QR( msg.pose.orientation.w , msg.pose.orientation.x , msg.pose.orientation.y , msg.pose.orientation.z );

  // Eigen::MatrixXd rpy = quaternion2rpy( QR );

  Eigen::Vector3d rpy;
  // rpy << 0, 0, 0;
  rpy << srv.response.euler[0], srv.response.euler[1], srv.response.euler[2];

  double roll  = rpy.coeff( 0, 0 ) * 180.0 / M_PI;
  double pitch = rpy.coeff( 1, 0 ) * 180.0 / M_PI;
  double yaw   = rpy.coeff( 2, 0 ) * 180.0 / M_PI;

  ui.ori_roll_spinbox->setValue( roll );
  ui.ori_pitch_spinbox->setValue( pitch );
  ui.ori_yaw_spinbox->setValue( yaw );
}

Eigen::MatrixXd MainWindow::rotationX( double angle )
{
  Eigen::MatrixXd _rotation( 3, 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationY( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
                    0.0,     1.0,     0.0,
              -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotationZ( double angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::MatrixXd MainWindow::rotation2rpy( Eigen::MatrixXd rotation )
{
  Eigen::MatrixXd _rpy = Eigen::MatrixXd::Zero( 3 , 1 );

  // _rpy.coeffRef( 0 , 0 ) = atan2( rotation.coeff( 2 , 1 ), rotation.coeff( 2 , 2 ) );
  // _rpy.coeffRef( 1 , 0 ) = atan2( -rotation.coeff( 2 , 0 ), sqrt( pow( rotation.coeff( 2 , 1 ) , 2 ) + pow( rotation.coeff( 2 , 2 ) , 2 ) ) );
  // _rpy.coeffRef( 2 , 0 ) = atan2 ( rotation.coeff( 1 , 0 ) , rotation.coeff( 0 , 0 ) );

  // Eigen::Vector3d rpy;// = Eigen::MatrixXd::Zero(3,1);
  /*
  rpy.coeffRef(0,0) = atan2(rotation.coeff(2,1), rotation.coeff(2,2));
  rpy.coeffRef(1,0) = atan2(-rotation.coeff(2,0), sqrt(pow(rotation.coeff(2,1), 2) + pow(rotation.coeff(2,2),2)));
  rpy.coeffRef(2,0) = atan2 (rotation.coeff(1,0), rotation.coeff(0,0));
  */

  //  std::cout<<"rotationrotationrotation"<<std::endl<<rotation<<std::endl<<std::endl;
  _rpy.coeffRef(2,0) = asin(rotation.coeff(2,1));
  if (_rpy.coeffRef(2,0) == M_PI / 2)
  {
      _rpy.coeffRef(1,0) = 0;
      _rpy.coeffRef(0,0) = atan2(-rotation.coeff(1, 0), -rotation.coeff(1, 1));
  }
  else if (_rpy.coeffRef(2,0) == -M_PI / 2)
  {
      _rpy.coeffRef(1,0) = 0;
      _rpy.coeffRef(0,0) = atan2(-rotation.coeff(1, 0), -rotation.coeff(1, 1));
  }
  else
  {
      _rpy.coeffRef(1,0) = atan2(rotation.coeff(0, 2), -rotation.coeff(2, 2));
      _rpy.coeffRef(0,0) = atan2(-rotation.coeff(1, 0), -rotation.coeff(1, 1));
  }

  return _rpy;
}

Eigen::MatrixXd MainWindow::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::Matrix3d origin;
  origin << 1, 0, 0,
            0,-1, 0,
            0, 0,-1;
  // Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );
  Eigen::MatrixXd _rotation = origin * rotationY( pitch ) * rotationX( yaw ) * rotationZ( roll );

  return _rotation;
}

Eigen::Quaterniond MainWindow::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

  // std::cout<<"OLDROTATIONOLDROTATIONOLDROTATION"<<std::endl<<_rotation<<std::endl;

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond MainWindow::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd MainWindow::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd MainWindow::quaternion2rotation( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rotation = quaternion.toRotationMatrix();

  return _rotation;
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
  ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
  QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Robotis</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

}  // namespace manipulator_h_gui

