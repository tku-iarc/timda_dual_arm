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
    if(side.compare("right") == 0 || side.compare("both") == 0)
    {
        right_connect = true;
        right_vector_move_pub = nh.advertise<manipulator_h_base_module_msgs::VectorMove>("right_arm/vector_move_rpy_msg", 0);
        right_set_mode_pub = nh.advertise<std_msgs::String>("right_arm/set_mode_msg", 0);
    }
    else
        right_connect = false;
    if(side.compare("left") == 0 || side.compare("both") == 0)
    {
        left_connect = true;
        left_vector_move_pub = nh.advertise<manipulator_h_base_module_msgs::VectorMove>("left_arm/vector_move_rpy_msg", 0);
        left_set_mode_pub = nh.advertise<std_msgs::String>("left_arm/set_mode_msg", 0);
    }
    else
        left_connect  = false;
    
    input_status = no_input;
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
        joy_x = msg->axes[1] - cent_x;
        joy_y = msg->axes[0] - cent_y;
        joy_z = msg->axes[4] - cent_z;
        joy_roll = msg->buttons[1] - msg->buttons[2];
        joy_pitch = msg->axes[7];
        joy_yaw = -1*msg->axes[6];
        joy_phi = msg->buttons[3] - msg->buttons[0];
        joy_left_enable = msg->axes[2] < -0.9;
        joy_right_enable = msg->axes[5] < -0.9;

        if(input_status == cmd_done)
        {
            int sum_of_elems = 0;
            for(int indx = 4; indx <= 8; indx++)
                sum_of_elems += msg->buttons[indx];
            if(sum_of_elems < 1)
                input_status = no_input;

        }
        if(input_status == no_input && (joy_left_enable || joy_right_enable))
        {
            if(msg->buttons[7] == 1)
                input_status = joy_gripper_grab_;
            else if(msg->buttons[6] == 1)
                input_status = joy_gripper_release_;
            else if(msg->buttons[5] == 1)
                input_status = joy_sucker_on_;
            else if(msg->buttons[4] == 1)
                input_status = joy_sucker_off_;
            else if(msg->buttons[8] == 1)
                input_status = joy_set_mode_;
        }
        if(joy_left_enable && joy_right_enable)
        {
            joy_x = 0;
            joy_y = 0;
            joy_z = 0;
            joy_roll = 0;
            joy_pitch = 0;
            joy_yaw = 0;
            joy_phi = 0;
            input_status = no_input;
        }
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
            if(input_status != no_input && input_status != cmd_done)
            {
                switch(input_status)
                {
                    case joy_gripper_grab_:
                        input_status = cmd_done;
                        break;
                    case joy_gripper_release_:
                        input_status = cmd_done;
                        break;
                    case joy_sucker_on_:
                        input_status = cmd_done;
                        break;
                    case joy_sucker_off_:
                        input_status = cmd_done;
                        break;
                    case joy_set_mode_:
                        std_msgs::String msg;
                        msg.data ="set_mode";
                        right_set_mode_pub.publish ( msg );
                        input_status = cmd_done;
                        break;
                }
            }
            else
            {
                float x = (joy_x > 0) ? joy_x/max_x : -1*joy_x/min_x;
                float y = (joy_y > 0) ? joy_y/max_y : -1*joy_y/min_y;
                float z = (joy_z > 0) ? joy_z/max_z : -1*joy_z/min_z;
                moving_msg.moving_vector = {x, y, z, joy_roll, joy_pitch, joy_yaw, joy_phi};
                for(int indx = 0; indx <= 6; indx++)
                {
                    if(moving_msg.moving_vector[indx] > 0.2 || moving_msg.moving_vector[indx] < -0.2);
                    {
                        right_vector_move_pub.publish(moving_msg);
                        break;
                    }
                }
            }
        }
        else if(joy_left_enable && left_connect)
        {
            if(input_status != no_input && input_status != cmd_done)
            {
                switch(input_status)
                {
                    case joy_gripper_grab_:
                        input_status = cmd_done;
                        break;
                    case joy_gripper_release_:
                        input_status = cmd_done;
                        break;
                    case joy_sucker_on_:
                        input_status = cmd_done;
                        break;
                    case joy_sucker_off_:
                        input_status = cmd_done;
                        break;
                    case joy_set_mode_:
                        std_msgs::String msg;
                        msg.data ="set_mode";
                        left_set_mode_pub.publish ( msg );
                        input_status = cmd_done;
                        break;
                }
            }
            else
            {
                float x = (joy_x > 0) ? joy_x/max_x : -1*joy_x/min_x;
                float y = (joy_y > 0) ? joy_y/max_y : -1*joy_y/min_y;
                float z = (joy_z > 0) ? joy_z/max_z : -1*joy_z/min_z;
                moving_msg.moving_vector = {x, y, z, joy_roll, joy_pitch, joy_yaw, joy_phi};
                for(int indx = 0; indx <= 6; indx++)
                {
                    if(moving_msg.moving_vector[indx] > 0.2 || moving_msg.moving_vector[indx] < -0.2)
                    {
                        left_vector_move_pub.publish(moving_msg);
                        break;
                    }
                }
            }
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