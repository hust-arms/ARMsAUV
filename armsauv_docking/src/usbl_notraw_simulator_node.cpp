#include <armsauv_docking/usbl_notraw_simulator.h>
#include <armsauv_docking/common.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "usbl_notraw_simulator");
    
    contentPrint<std::string>("usbl_sim", "create usbl simulator (not raw)");
    tf::TransformListener tf(ros::Duration(0.5));
    armsauv_docking::USBLSimulator usbl_sim(tf);

    ros::spin();
}
