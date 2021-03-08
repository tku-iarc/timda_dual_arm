#include "robotiq_2f_gripper_control/gripper_control.h"




using namespace robotiq_2f_gripper;


int main(int argc, char **argv){

  GripperControl gripper(argc, argv);
  gripper.init();
}