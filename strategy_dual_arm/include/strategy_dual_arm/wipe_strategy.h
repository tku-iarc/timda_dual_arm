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
#ifndef WIPE_STRATEGY_H
#define WIPE_STRATEGY_H

// #ifndef strategy_dual_arm_WIPE_STRATEGY_H
// #define strategy_dual_arm_WIPE_STRATEGY_H

/*****************************************************************************
** Includes
*****************************************************************************/

//#ifndef Q_MOC_RUN

#include <ros/ros.h>
//#include <QMainWindow>
//#include "ui_main_window.h"
#include "strategy_dual_arm/strategy_msg.h"

//#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace strategy_dual_arm {


class WipeStrategy
{
    //Q_OBJECT

public:
    WipeStrategy(int argc, char** argv);
    ~WipeStrategy();

    // std::string speed;
    // std::string pos_x;
    // std::string pos_y;
    // std::string pos_z;
    int speed;
    double pos_x;
    double pos_y;
    double pos_z;
    double ori_roll;
    double ori_pitch;
    double ori_yaw;
    double ori_phi;

    //void closeEvent(QCloseEvent *event); // Overloaded function

    /******************************************
    ** Transformation
    *******************************************/
    Eigen::MatrixXd rotationX( double angle );
    Eigen::MatrixXd rotationY( double angle );
    Eigen::MatrixXd rotationZ( double angle );

    Eigen::MatrixXd rotation2rpy( Eigen::MatrixXd rotation );
    Eigen::MatrixXd rpy2rotation( double roll, double pitch, double yaw );

    Eigen::Quaterniond rpy2quaternion( double roll, double pitch, double yaw );
    Eigen::Quaterniond rotation2quaternion( Eigen::MatrixXd rotation );

    Eigen::MatrixXd quaternion2rpy( Eigen::Quaterniond quaternion );
    Eigen::MatrixXd quaternion2rotation( Eigen::Quaterniond quaternion );

    void relative_movement();
    void get_curr_pos();

//public Q_SLOTS:
    /******************************************
        ** Auto-connections (connectSlotsByName())
        *******************************************/
    void on_actionAbout_triggered();

    void on_curr_joint();
    void on_des_joint();

    void on_curr_pos();
    void on_des_pos();
    void on_des_p2p();
    // void on_des_MoveIt_button_clicked();
    //===========robotiq_2f_gripper===================
    void on_des_Grap_Alcohol();
    void on_des_Release();
    //===============================================

    void on_ini_pose();
    void on_set_mode();
    void on_relative_xp();
    void on_relative_xm();
    void on_relative_yp();
    void on_relative_ym();
    void on_relative_zp();
    void on_relative_zm();
    void on_relative_ap();
    void on_relative_am();
    void on_relative_rollp();
    void on_relative_rollm();
    void on_relative_pitchp();
    void on_relative_pitchm();
    void on_relative_yawp();
    void on_relative_yawm();
    void on_relative_phip();
    void on_relative_phim();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void updateCurrJointPoseSpinbox( manipulator_h_base_module_msgs::JointPose msg );
    void updateCurrKinematicsPoseSpinbox( manipulator_h_base_module_msgs::GetKinematicsPose );


private:
    //Ui::MainWindowDesign ui;
    StrategyMsg strategy_msg;

    // WipeStrategy wipe_strategy;

    std::vector<std::string> joint_name;

    //QList<QAbstractSpinBox *> joint_spinbox;

    double relative_movement_data[7];
};

}  // namespace manipulator_h_gui

#endif // strategy_dual_arm_WIPE_STRATEGY_H
