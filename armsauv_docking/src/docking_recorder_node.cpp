#include <armsauv_docking/docking_recorder.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "docking_recorder");

    contentPrint<std::string>("docking_recorder", "create docking recorder");
    armsauv_docking::DockingRecorder docking_rec;

    ros::spin();
}
