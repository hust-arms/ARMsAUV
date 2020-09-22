#include <armsauv_docking/usbl_simulator.h>
#include <armsauv_docking/common.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "usbl_simulator");
    
    contentPrint<std::string>("usbl_sim", "create usbl simultaor");
    tf::TransformListener tf(ros::Duration(0.5));
    armsauv_docking::USBLSimulator usbl_sim(tf);

    ros::spin();
}
