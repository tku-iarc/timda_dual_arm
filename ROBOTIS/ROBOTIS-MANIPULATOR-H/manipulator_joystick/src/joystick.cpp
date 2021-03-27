#include "manipulator_joystick/joystick.hpp"

Joystick::Joystick(int argc, char **argv)
{
    ros::init(argc, argv, "manipulator_joystick");
    // ros::start();
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string side;
    nh_private.getParam("/joy_control/side", side);
    joy_sub = nh.subscribe("joy", 5, &Joystick::joysticMsgCallback, this);
    joy_limit_path = ros::package::getPath("manipulator_joystick") + "/config/joy_limit.yaml";
    if(side.compare("both") == 0)
    {
        right_vector_move_pub = nh.advertise<manipulator_h_base_module_msgs::VectorMove>("right_arm/vector_move_rpy_msg", 0);
        left_vector_move_pub = nh.advertise<manipulator_h_base_module_msgs::VectorMove>("left_arm/vector_move_rpy_msg", 0);
    }
    else if(side.compare("right") == 0)
    {
        right_connect = true;
        right_vector_move_pub = nh.advertise<manipulator_h_base_module_msgs::VectorMove>("right_arm/vector_move_rpy_msg", 0);
    }
    else if(side.compare("left") == 0)
    {
        left_connect = true;
        left_vector_move_pub = nh.advertise<manipulator_h_base_module_msgs::VectorMove>("left_arm/vector_move_rpy_msg", 0);
    }
    else
    {
        right_connect = false;
        left_connect  = false;
    }
    joy_x = 0;
    joy_y = 0;
    joy_z = 0;
    joy_roll = 0;
    joy_pitch = 0;
    joy_yaw = 0;
    joy_phi = 0;
    joy_left_enable = false;
    joy_right_enable = false;
    joy_gripper_grab = false;
    joy_gripper_release = false;
    joy_sucker_on = false;
    joy_sucker_off =  false;
    joy_set_mode = false;
    is_calib = false;
    max_x = 1;
    max_y = 1;
    max_z = 1;
    min_x = -1;
    min_y = -1;
    min_z = -1;
    cent_x = 0;
    cent_y = 0;
    cent_z = 0;
    return;
}

Joystick::~Joystick()
{

}

void Joystick::joysticMsgCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(is_calib)
    {
        if(msg->axes[1] > max_x)
            max_x = msg->axes[1];
        else if(msg->axes[1] < min_x)
            min_x = msg->axes[1];
        if(msg->axes[0] > max_y)
            max_y = msg->axes[0];
        else if(msg->axes[0] < min_y)
            min_y = msg->axes[0];
        if(msg->axes[4] > max_z)
            max_z = msg->axes[4];
        else if(msg->axes[4] < min_z)
            min_z = msg->axes[4];
    }
    else
    {
        joy_x = msg->axes[1]- cent_x;
        joy_y = msg->axes[0]- cent_y;
        joy_z = msg->axes[4]- cent_z;
        joy_roll = msg->buttons[1] - msg->buttons[2];
        joy_pitch = msg->axes[7];
        joy_yaw = -1*msg->axes[6];
        joy_phi = msg->buttons[3] - msg->buttons[0];
        joy_left_enable = msg->axes[2] < -0.9;
        joy_right_enable = msg->axes[5] < -0.9;
        joy_gripper_grab = msg->buttons[7] == 1;
        joy_gripper_release = msg->buttons[6] == 1;
        joy_sucker_on = msg->buttons[5] == 1;
        joy_sucker_off =  msg->buttons[4] == 1;
        joy_set_mode = msg->buttons[8] == 1;
    }
    return;
}

bool Joystick::calibrationCallback(manipulator_joystick::JoyCalibration::Request &req,
                                   manipulator_joystick::JoyCalibration::Response &res )
{
    if(req.calib_cmd != is_calib)
        is_calib = !is_calib;
    
    if(is_calib)
    {
        cent_x = joy_x;
        cent_y = joy_y;
        cent_z = joy_z;
    }
    res.calib_status = is_calib;
    return true;
}

void Joystick::process(void)
{
    ros::Rate loop_rate(125);
    while(ros::ok())
    {
        if(is_calib)
        {
            joy_x = 0;
            joy_y = 0;
            joy_z = 0;
            joy_roll = 0;
            joy_pitch = 0;
            joy_yaw = 0;
            joy_phi = 0;
            joy_left_enable = false;
            joy_right_enable = false;
            joy_gripper_grab = false;
            joy_gripper_release = false;
            joy_sucker_on = false;
            joy_sucker_off =  false;
            joy_set_mode = false;
        }
        else if(joy_right_enable && right_connect)
        {
            float x = (joy_x > 0) ? joy_x/max_x : -1*joy_x/min_x;
            float y = (joy_y > 0) ? joy_y/max_y : -1*joy_y/min_y;
            float z = (joy_z > 0) ? joy_z/max_z : -1*joy_z/min_z;
            moving_msg.moving_vector = {x, y, z, joy_roll, joy_pitch, joy_yaw, joy_phi};
            right_vector_move_pub.publish(moving_msg);
        }
        else if(joy_left_enable && left_connect)
        {
            float x = (joy_x > 0) ? joy_x/max_x : -1*joy_x/min_x;
            float y = (joy_y > 0) ? joy_y/max_y : -1*joy_y/min_y;
            float z = (joy_z > 0) ? joy_z/max_z : -1*joy_z/min_z;
            moving_msg.moving_vector = {x, y, z, joy_roll, joy_pitch, joy_yaw, joy_phi};
            left_vector_move_pub.publish(moving_msg);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char **argv)
{
    Joystick joystick(argc, argv);
    joystick.process();
    return 0;
}