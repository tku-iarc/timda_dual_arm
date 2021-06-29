#ifndef manipulator_h_gui_JOYSTICK
#define manipulator_h_gui_JOYSTICK
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/network.h>
#include <string>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <manipulator_h_base_module_msgs/VectorMove.h>
#include <manipulator_h_joystick/JoyCalibration.h>

class Joystick
{
public:
    Joystick(int argc, char **argv);
    virtual ~Joystick();
    void init(void);

    void joysticMsgCallback(const sensor_msgs::Joy::ConstPtr& msg);
    bool calibrationCallback(manipulator_h_joystick::JoyCalibration::Request &req, 
                             manipulator_h_joystick::JoyCalibration::Response &res );
    void process(void);

    float get_joy_x(void){return joy_x;}
    float get_joy_y(void){return joy_y;}
    float get_joy_z(void){return joy_z;}
    float get_joy_roll(void){return joy_roll;}
    float get_joy_pitch(void){return joy_pitch;}
    float get_joy_yaw(void){return joy_yaw;}
    float get_joy_phi(void){return joy_phi;}
    bool get_joy_gripper_grab(void){return joy_gripper_grab;}
    bool get_joy_gripper_release(void){return joy_gripper_release;}
    bool get_joy_sucker_on(void){return joy_sucker_on;}
    bool get_joy_sucker_off(void){return joy_sucker_off;}
    bool get_joy_set_mode(void){return joy_set_mode;}
    bool get_joy_left_enable(void){return joy_left_enable;}
    bool get_joy_right_enable(void){return joy_right_enable;}
    bool get_is_calib(void){return is_calib;}

private:
    enum InputStatus : int {joy_gripper_grab_, joy_gripper_release_, joy_sucker_on_, joy_sucker_off_, joy_set_mode_, cmd_done, no_input};
    InputStatus input_status;
    bool  tmp_enable;
    bool  right_connect;
    bool  left_connect; 
    bool  joy_gripper_grab;
    bool  joy_gripper_release;
    bool  joy_sucker_on;
    bool  joy_sucker_off;
    bool  joy_set_mode;
    bool  joy_left_enable;
    bool  joy_right_enable;
    bool  is_calib;

    float joy_x;
    float joy_y;
    float joy_z;
    float joy_roll;
    float joy_pitch;
    float joy_yaw;
    float joy_phi;
    
    float max_x;
    float max_y;
    float max_z;
    float min_x;
    float min_y;
    float min_z;
    float cent_x;
    float cent_y;
    float cent_z;

    std::string     joy_limit_path;
    ros::Subscriber joy_sub;
    ros::Publisher  right_vector_move_pub;
    ros::Publisher  left_vector_move_pub;
    ros::Publisher  right_set_mode_pub;
    ros::Publisher  left_set_mode_pub;
    ros::Publisher  robot_stop_pub;
    ros::ServiceServer calib_server;

    manipulator_h_base_module_msgs::VectorMove moving_msg;

};

#endif