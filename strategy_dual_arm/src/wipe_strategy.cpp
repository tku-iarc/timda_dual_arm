/*****************************************************************************
** Includes
*****************************************************************************/

//#include <QtGui>
//#include <QMessageBox>
#include <iostream>
#include "strategy_dual_arm/wipe_strategy.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace strategy_dual_arm {

//using namespace Qt;
//using namespace WipeStrategy;

/*****************************************************************************
** Implementation [WipeStrategy]
*****************************************************************************/
WipeStrategy::WipeStrategy(int argc, char** argv)
  //: QMainWindow(parent)
  : strategy_msg(argc,argv)

{
    speed = 10;
    pos_x = 0.000;
    pos_y = 0.000;
    pos_z = 0.000;
    ori_roll   = 0.000;
    ori_pitch  = 0.000;
    ori_yaw    = 0.000;
    ori_phi    = 0.000;


  joint_name.push_back("joint1");
  joint_name.push_back("joint2");
  joint_name.push_back("joint3");
  joint_name.push_back("joint4");
  joint_name.push_back("joint5");
  joint_name.push_back("joint6");
  joint_name.push_back("joint7");

  
  /*********************
    ** Auto Start
    **********************/
  strategy_msg.init();

}

WipeStrategy::~WipeStrategy() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void WipeStrategy::relative_movement()
{
  manipulator_h_base_module_msgs::P2PPose msg;

  msg.name = "arm";

  msg.speed           = speed;
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

  strategy_msg.sendP2PPoseMsg( msg );
  return;
}

// void WipeStrategy::StrategyeMsg.getCurrPose(this->relative_movement_data)
// {
//   std::string group_name = "arm";
//   StrategyeMsg.getKinematicsPose( group_name );

//   this->relative_movement_data[0] = ui.pos_x_spinbox->value();
//   this->relative_movement_data[1] = ui.pos_y_spinbox->value();
//   this->relative_movement_data[2] = ui.pos_z_spinbox->value();
  
//   this->relative_movement_data[3] = ui.ori_roll_spinbox->value();
//   this->relative_movement_data[4] = ui.ori_pitch_spinbox->value();
//   this->relative_movement_data[5] = ui.ori_yaw_spinbox->value();
//   this->relative_movement_data[6] = ui.ori_phi_spinbox->value();

//   return;
// }
void WipeStrategy::on_curr_joint()
{
  strategy_msg.getJointPose( joint_name );
}

// void WipeStrategy::on_des_joint()
// {
//   manipulator_h_base_module_msgs::JointPose msg;

//   for ( int _id = 0; _id < joint_spinbox.size(); _id++ )
//   {
//     msg.name.push_back( joint_name[ _id ] );
//     msg.value.push_back( ((QDoubleSpinBox *) joint_spinbox[ _id ])->value() * M_PI / 180.0 );
//   }
//   msg.slide_pos = ui.slide_spinbox->value();
//   msg.speed     = ui.speed_spinbox->value();
//   StrategyeMsg.sendJointPoseMsg( msg );
// }

void WipeStrategy::on_curr_pos()
{
  std::string group_name = "arm";

  strategy_msg.getKinematicsPose( group_name );
}

void WipeStrategy::on_des_pos()
{
  manipulator_h_base_module_msgs::KinematicsPose msg;

  msg.name = "arm";

  msg.speed           = speed;
  msg.pose.position.x = pos_x;
  msg.pose.position.y = pos_y;
  msg.pose.position.z = pos_z;

  double roll  = ori_roll  * M_PI / 180.0;
  double pitch = ori_pitch * M_PI / 180.0;
  double yaw   = ori_yaw   * M_PI / 180.0;
  double phi   = ori_phi   * M_PI / 180.0;
  
  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();
  msg.phi = phi;
  
  strategy_msg.sendKinematicsPoseMsg( msg );
}

void WipeStrategy::on_des_p2p()
{
  //std::cout<<speed<<std::endl;
  //std::cout<<pos_x,pos_y,pos_z<<std::endl;
  //std::cout<<ori_roll,ori_pitch,ori_yaw<<std::endl;
  manipulator_h_base_module_msgs::P2PPose msg;

  msg.name = "arm";

  msg.speed           = speed;
  msg.pose.position.x = pos_x;
  msg.pose.position.y = pos_y;
  msg.pose.position.z = pos_z;

  double roll  = ori_roll  * M_PI / 180.0;
  double pitch = ori_pitch * M_PI / 180.0;
  double yaw   = ori_yaw   * M_PI / 180.0;
  double phi   = ori_phi   * M_PI / 180.0;
  
  Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

  msg.pose.orientation.x = QR.x();
  msg.pose.orientation.y = QR.y();
  msg.pose.orientation.z = QR.z();
  msg.pose.orientation.w = QR.w();
  msg.phi = phi;

  std::cout<<msg.speed<<std::endl;
  std::cout<<msg.pose.position.x<<std::endl;
  std::cout<<msg.pose.position.y<<std::endl;
  std::cout<<msg.pose.position.z<<std::endl;
  std::cout<<msg.pose.orientation.x<<std::endl;
  std::cout<<msg.pose.orientation.y<<std::endl;
  std::cout<<msg.pose.orientation.z<<std::endl;
  std::cout<<msg.pose.orientation.w<<std::endl;
  std::cout<<msg.phi<<std::endl;


  strategy_msg.sendP2PPoseMsg( msg );
}

// void WipeStrategy::on_des_MoveIt_button_clicked( bool check )
// {
//   manipulator_h_base_module_msgs::P2PPose msg;

//   msg.name = "arm";

//   msg.speed           = ui.speed_spinbox->value();
//   msg.pose.position.x = ui.pos_x_spinbox->value();
//   msg.pose.position.y = ui.pos_y_spinbox->value();
//   msg.pose.position.z = ui.pos_z_spinbox->value();
  
//   double roll  = ui.ori_roll_spinbox->value()  * M_PI / 180.0;
//   double pitch = ui.ori_pitch_spinbox->value() * M_PI / 180.0;
//   double yaw   = ui.ori_yaw_spinbox->value()   * M_PI / 180.0;
//   double phi   = ui.ori_phi_spinbox->value()   * M_PI / 180.0;
  
//   Eigen::Quaterniond QR = rpy2quaternion( roll, pitch, yaw );

//   msg.pose.orientation.x = QR.x();
//   msg.pose.orientation.y = QR.y();
//   msg.pose.orientation.z = QR.z();
//   msg.pose.orientation.w = QR.w();
//   msg.phi = phi;

//   StrategyeMsg.sendMoveItPoseMsg( msg );
// }

//=============robotiq_2f_gripper=========================
void WipeStrategy::on_des_Grap_Alcohol()
{
  std_msgs::String msg;

  msg.data ="Gripper_grap";

  strategy_msg.sendGrapAlcoholMsg( msg );
}
void WipeStrategy::on_des_Release()
{
  std_msgs::String msg;

  msg.data ="Gripper_release";

  strategy_msg.sendReleasePoseMsg( msg );
}
//==============================================================
void WipeStrategy::on_ini_pose()
{
  std_msgs::String msg;

  msg.data ="ini_pose";

  strategy_msg.sendIniPoseMsg( msg );
}

void WipeStrategy::on_set_mode()
{
  std_msgs::String msg;

  msg.data ="set_mode";

  strategy_msg.sendSetModeMsg( msg );
}


Eigen::MatrixXd WipeStrategy::rotationX( double angle )
{
  Eigen::MatrixXd _rotation( 3, 3 );

  _rotation << 1.0,          0.0,           0.0,
      0.0, cos( angle ), -sin( angle ),
      0.0, sin( angle ),  cos( angle );

  return _rotation;
}

Eigen::MatrixXd WipeStrategy::rotationY( double angle )
{
  Eigen::MatrixXd _rotation( 3 , 3 );

  _rotation << cos( angle ), 0.0, sin( angle ),
                    0.0,     1.0,     0.0,
              -sin( angle ), 0.0, cos( angle );

  return _rotation;
}

Eigen::MatrixXd WipeStrategy::rotationZ( double angle )
{
  Eigen::MatrixXd _rotation(3,3);

  _rotation << cos( angle ), -sin( angle ), 0.0,
      sin( angle ),  cos( angle ), 0.0,
      0.0,           0.0, 1.0;

  return _rotation;
}

Eigen::MatrixXd WipeStrategy::rotation2rpy( Eigen::MatrixXd rotation )
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

Eigen::MatrixXd WipeStrategy::rpy2rotation( double roll, double pitch, double yaw )
{
  Eigen::Matrix3d origin;
  origin << 1, 0, 0,
            0,-1, 0,
            0, 0,-1;
  // Eigen::MatrixXd _rotation = rotationZ( yaw ) * rotationY( pitch ) * rotationX( roll );
  Eigen::MatrixXd _rotation = origin * rotationY( pitch ) * rotationX( yaw ) * rotationZ( roll );

  return _rotation;
}

Eigen::Quaterniond WipeStrategy::rpy2quaternion( double roll, double pitch, double yaw )
{
  Eigen::MatrixXd _rotation = rpy2rotation( roll, pitch, yaw );

  // std::cout<<"OLDROTATIONOLDROTATIONOLDROTATION"<<std::endl<<_rotation<<std::endl;

  Eigen::Matrix3d _rotation3d;
  _rotation3d = _rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;

  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::Quaterniond WipeStrategy::rotation2quaternion( Eigen::MatrixXd rotation )
{
  Eigen::Matrix3d _rotation3d;

  _rotation3d = rotation.block( 0 , 0 , 3 , 3 );

  Eigen::Quaterniond _quaternion;
  _quaternion = _rotation3d;

  return _quaternion;
}

Eigen::MatrixXd WipeStrategy::quaternion2rpy( Eigen::Quaterniond quaternion )
{
  Eigen::MatrixXd _rpy = rotation2rpy( quaternion.toRotationMatrix() );

  return _rpy;
}

Eigen::MatrixXd WipeStrategy::quaternion2rotation( Eigen::Quaterniond quaternion )
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
// void WipeStrategy::updateLoggingView() {
//   ui.view_logging->scrollToBottom();
// }

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

// void WipeStrategy::on_actionAbout_triggered() {
//   QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Robotis</p><p>This package needs an about description.</p>"));
// }

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

// void WipeStrategy::closeEvent(QCloseEvent *event)
// {
//   QMainWindow::closeEvent(event);
// }

// int main(int argc, char** argv){
    
//     //MainStrategy MS;
//     // strategy_dual_arm::WipeStrategy (argc,argv);
//     // strategy_dual_arm::StrategyeMsg (argc,argv);
//     // MainStrategy::~MainStrategy (argc,argv);
//     //MainStrategy (argc,argv);
//     //WipeStrategy::~WipeStrategy();

//     WipeStrategy wipe_strategy(argc,argv);
//     //WipeStrategy (argc,argv);
//     //StrategyeMsg (argc,argv);
//     //WipeStrategy wipe;
//     //WipeStrategy{};
//     // speed = 50;
//     // pos_x = -0.2000;
//     // pos_y = 0.0363;
//     // pos_z = -0.4750;
//     // ori_roll   = 44.024;
//     // ori_pitch  = -0.005;
//     // ori_yaw    = -44.998;
//     // ori_phi    = 0.000;
//     // strategy_dual_arm::WipeStrategy::on_des_p2p();
//     //WipeStrategy.on_des_p2p();

//     wipe_strategy.on_des_p2p();

//     return 0;
// }



}  // namespace manipulator_h_gui


