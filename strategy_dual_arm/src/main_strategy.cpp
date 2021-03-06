#include <iostream>
#include "strategy_dual_arm/wipe_strategy.h"
#include "strategy_dual_arm/strategy_msg.h"
//#include <windows.h>


using namespace strategy_dual_arm;


int main(int argc, char** argv){

    WipeStrategy wipe_strategy(argc,argv);
    //StrategyMsg strategy_msg(argc,argv);

    wipe_strategy.speed = 50;
    wipe_strategy.pos_x = -0.2000;
    wipe_strategy.pos_y = 0.0363;
    wipe_strategy.pos_z = -0.4750;
    wipe_strategy.ori_roll   = 44.024;
    wipe_strategy.ori_pitch  = -0.005;
    wipe_strategy.ori_yaw    = -44.998;
    wipe_strategy.ori_phi    = 0.000;
    wipe_strategy.on_set_mode();
    sleep(3);
    //loop_rate.sleep();
    //wipe_strategy.on_set_mode();
    wipe_strategy.on_set_mode();
    sleep(2);
    wipe_strategy.on_curr_joint();
    wipe_strategy.on_curr_joint();
    wipe_strategy.on_curr_pos();
    wipe_strategy.on_curr_pos();
	wipe_strategy.on_des_p2p();

}

