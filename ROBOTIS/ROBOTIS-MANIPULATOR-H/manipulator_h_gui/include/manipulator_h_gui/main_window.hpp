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
 * @file /include/manipulator_h_gui/main_window.hpp
 *
 * @brief Qt based gui for manipulator_h_gui.
 *
 * @date November 2010
 **/
#ifndef manipulator_h_gui_MAIN_WINDOW_H
#define manipulator_h_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#endif

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace manipulator_h_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();

    void closeEvent(QCloseEvent *event); // Overloaded function

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

public Q_SLOTS:
    /******************************************
        ** Auto-connections (connectSlotsByName())
        *******************************************/
    void on_actionAbout_triggered();

    void on_curr_joint_button_clicked( bool check );
    void on_des_joint_button_clicked( bool check );

    void on_curr_pos_button_clicked( bool check );
    void on_des_pos_button_clicked( bool check );
    void on_des_p2p_button_clicked( bool check );
    void on_des_moveit_button_clicked( bool check );

    //===========robotiq_2f_gripper===================
    void on_des_Grap_button_clicked( bool check );
    void on_des_Release_button_clicked( bool check );
    //===============================================
    
    void on_ini_pose_button_clicked( bool check );
    void on_home_pose_button_clicked( bool check );
    void on_set_mode_button_clicked( bool check );
    void on_joy_calib_button_clicked( bool check );
    void on_relative_xp_button_clicked( bool check );
    void on_relative_xm_button_clicked( bool check );
    void on_relative_yp_button_clicked( bool check );
    void on_relative_ym_button_clicked( bool check );
    void on_relative_zp_button_clicked( bool check );
    void on_relative_zm_button_clicked( bool check );
    void on_relative_ap_button_clicked( bool check );
    void on_relative_am_button_clicked( bool check );
    void on_relative_rollp_button_clicked( bool check );
    void on_relative_rollm_button_clicked( bool check );
    void on_relative_pitchp_button_clicked( bool check );
    void on_relative_pitchm_button_clicked( bool check );
    void on_relative_yawp_button_clicked( bool check );
    void on_relative_yawm_button_clicked( bool check );
    void on_relative_phip_button_clicked( bool check );
    void on_relative_phim_button_clicked( bool check );

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically

    void updateCurrJointPoseSpinbox( manipulator_h_base_module_msgs::JointPose msg );
    void updateCurrKinematicsPoseSpinbox( manipulator_h_base_module_msgs::GetKinematicsPose );

private:
    Ui::MainWindowDesign ui;
    QNode qnode;

    std::vector<std::string> joint_name;

    QList<QAbstractSpinBox *> joint_spinbox;

    double relative_movement_data[7];
    bool joy_calib_state;
};

}  // namespace manipulator_h_gui

#endif // manipulator_h_gui_MAIN_WINDOW_H
