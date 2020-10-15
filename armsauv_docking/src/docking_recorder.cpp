#include <armsauv_docking/docking_recorder.h>

namespace armsauv_docking{
    DockingRecorder::DockingRecorder(){
        /* Create node handle */
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

	contentPrint<std::string>("docking_recorder", "Parameters configuration");

        /* Parameters configuration */
        private_nh.param("file_name", filename_, std::string("docking_record"));
        private_nh.param("path", path_, std::string("/home/ros302/Documents/catkin_armsauv/src/ARMsAUV/armsauv_docking/record/"));
        private_nh.param("frequency", rec_freq_, 5);

        std::string auv_odom_t, usbl_msg_t, auv_ctrl_info_t;
        std::string veh_odom_t, armc_odom_t, arm_joints_ang_t, arm_ctrl_info_t;
        private_nh.param("auv_odom_t", auv_odom_t, std::string("/armsauv/pose_gt"));
        private_nh.param("usbl_msg_t", usbl_msg_t, std::string("/usbl/output"));
        private_nh.param("auv_ctrl_info_t", auv_ctrl_info_t, std::string("/armsauv/ctrl_info"));
        private_nh.param("veh_odom_t", veh_odom_t, std::string("/uvsm719/pose_gt")); 
        private_nh.param("armc_odom_t", armc_odom_t, std::string("/uvsm719/pose_gt"));
        private_nh.param("arm_joints_ang_t", arm_joints_ang_t, std::string("/uvsm719/joints_angle"));
        private_nh.param("arm_ctrl_info_t", arm_ctrl_info_t, std::string("/uvsm719/ctrl_info"));

	/* Initialize file writer */
	contentPrint<std::string>("docking_recorder", "Create file writer");
	std::stringstream trans_ss;
	// trans_ss << ros::Time::now();
	trans_ss << boost::gregorian::to_simple_string(boost::gregorian::day_clock::local_day());
	std::string time_stamp = trans_ss.str();
	filename_ = filename_ +"_"+time_stamp+".txt";

	contentPrint<std::string>("docking_recorder", filename_);
	fw_ = new FileWriter(filename_, path_);

        /* Initialize ROS Components */
	contentPrint<std::string>("docking_recorder", "Initialize ROS components");
        auv_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(auv_odom_t, 1, boost::bind(&DockingRecorder::auvOdometryCb, this, _1));
        usbl_msg_sub_ = nh.subscribe<armsauv_msgs::Usbl>(usbl_msg_t, 1, boost::bind(&DockingRecorder::usblMessageCb, this, _1));
        // auv_ctrl_info_sub_ = nh.subscribe<armsauv_msgs::CtrlInfo>(auv_ctrl_info_t, 1, boost::bind(&DockingRecorder::auvControlInfoCb, this, _1));
        veh_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(veh_odom_t, 1, boost::bind(&DockingRecorder::vehicleOdometryCb, this, _1));
        armc_odom_sub_ = nh.subscribe<nav_msgs::Odometry>(armc_odom_t, 1, boost::bind(&DockingRecorder::armCentreOdometryCb, this, _1));
        // arm_joints_sub_ = nh.subscribe<armsauv_msgs::JointsAngle>(arm_joints_ang_t, 1, boost::bind(&DockingRecorder::armJointsAngleCb, this, 1));
        // arm_ctrl_info_sub_ = nh.subscribe<armsauv_msgs::ArmCtrlInfo>(arm_ctrl_info_t, 1, boost::bind(&DockingRecorder::armControlInfoCb, this, _1));
        /* Start Thread */
	contentPrint<std::string>("docking_recorder", "Start recording thread");
        time0_ = ros::Time::now();
        rec_thread_ = new boost::thread(boost::bind(&DockingRecorder::recordThread, this));
    }

    DockingRecorder::~DockingRecorder(){
        if(rec_thread_){
            delete rec_thread_;
            rec_thread_ = nullptr;
        }

	if(fw_){
	    delete fw_;
	    fw_ = nullptr;
	}
    }

    void DockingRecorder::recordThread(){
        ros::NodeHandle nh;
        while(nh.ok()){
            /* File writing */
	    contentPrint<std::string>("docking_recorder", "Recording data");
            if(fw_->writeData(serializeData())){
	        contentPrint<std::string>("docking_recorder", "Record data successfully"); 
	    }
	    else{
	        contentPrint<std::string>("docking_recorder", "Failed to record data"); 
	    }

            boost::this_thread::sleep(boost::posix_time::milliseconds(1.0/rec_freq_ * 1000));
        }
    }

    std::string DockingRecorder::serializeData(){
        double d_time = (ros::Time::now() - time0_).toSec();

        /* Add time stamp*/
        std::stringstream ss;
	std::string space = " ";
        ss << d_time << space;
        
        /* Add auv odom */
        pthread_rwlock_rdlock(&auv_odom_lock_);
        tf::Quaternion temp_quat;
        tf::quaternionMsgToTF(auv_odom_.pose.pose.orientation, temp_quat);
        double auv_roll, auv_pitch, auv_yaw;
        tf::Matrix3x3(temp_quat).getRPY(auv_roll, auv_pitch, auv_yaw);
        ss << auv_odom_.pose.pose.position.x << space << auv_odom_.pose.pose.position.y << space << auv_odom_.pose.pose.position.z << space
           << auv_roll << space << auv_pitch << space << auv_yaw << space
           << auv_odom_.twist.twist.linear.x << space << auv_odom_.twist.twist.linear.y << space << auv_odom_.twist.twist.linear.z << space 
           << auv_odom_.twist.twist.angular.x << space << auv_odom_.twist.twist.angular.y << space << auv_odom_.twist.twist.angular.z << space ;
        pthread_rwlock_unlock(&auv_odom_lock_);

        /* Add USBL messages */
        pthread_rwlock_rdlock(&usbl_msg_lock_);
        ss << usbl_msg_.phi << space << usbl_msg_.psi << space << usbl_msg_.h_r << space << usbl_msg_.s_r << space;
        pthread_rwlock_unlock(&usbl_msg_lock_);

        /* Add auv control informations */


        /* Add vehicle odom */
        pthread_rwlock_rdlock(&veh_odom_lock_);
        tf::Quaternion temp_quat_ano;
        tf::quaternionMsgToTF(veh_odom_.pose.pose.orientation, temp_quat_ano);
        double veh_roll, veh_pitch, veh_yaw;
        tf::Matrix3x3(temp_quat).getRPY(veh_roll, veh_pitch, veh_yaw);
        ss << veh_odom_.pose.pose.position.x << space << veh_odom_.pose.pose.position.y << space << veh_odom_.pose.pose.position.z << space
           << veh_roll << space << veh_pitch << space << veh_yaw << space
           << veh_odom_.twist.twist.linear.x << space << veh_odom_.twist.twist.linear.y << space << veh_odom_.twist.twist.linear.z << space 
           << veh_odom_.twist.twist.angular.x << space << veh_odom_.twist.twist.angular.y << space << veh_odom_.twist.twist.angular.z << space ;
        pthread_rwlock_unlock(&veh_odom_lock_);

        /* Add arm centre odom */

        /* Add arm joints angle */

        /* Add arm control informations*/

	contentPrint<std::string>("docking_recorder", ss.str());

        return ss.str(); 
    }

    void DockingRecorder::auvOdometryCb(const nav_msgs::Odometry::ConstPtr& msg){
        /* Update auv odometry */ 
        pthread_rwlock_wrlock(&auv_odom_lock_);
        auv_odom_.header = msg->header;
        auv_odom_.child_frame_id = msg->child_frame_id;
        auv_odom_.pose.pose.position = msg->pose.pose.position;
        auv_odom_.pose.pose.orientation = msg->pose.pose.orientation;
        auv_odom_.twist.twist.linear = msg->twist.twist.linear;
        auv_odom_.twist.twist.angular = msg->twist.twist.angular;
        pthread_rwlock_unlock(&auv_odom_lock_);
    }

    void DockingRecorder::usblMessageCb(const armsauv_msgs::Usbl::ConstPtr& msg){
        /* Update usbl message */
        pthread_rwlock_wrlock(&usbl_msg_lock_);
        usbl_msg_.header = msg->header;
        usbl_msg_.phi = msg->phi;
        usbl_msg_.psi = msg->psi;
        usbl_msg_.h_r = msg->h_r;
        usbl_msg_.s_r = msg->s_r;
        pthread_rwlock_unlock(&usbl_msg_lock_);
    }

    /*
    void DockingRecorder::auvControlInfoCb(const armsauv_msgs::CtrlInfo::ConstPtr& msg){
     
    }
    */

    void DockingRecorder::vehicleOdometryCb(const nav_msgs::Odometry::ConstPtr& msg){
        /* Update vehicle odometry */
        pthread_rwlock_wrlock(&veh_odom_lock_);
        veh_odom_.header = msg->header;
        veh_odom_.child_frame_id = msg->child_frame_id;
        veh_odom_.pose.pose.position = msg->pose.pose.position;
        veh_odom_.pose.pose.orientation = msg->pose.pose.orientation;
        veh_odom_.twist.twist.linear = msg->twist.twist.linear;
        veh_odom_.twist.twist.angular = msg->twist.twist.angular;
        pthread_rwlock_unlock(&veh_odom_lock_);
    }

    void DockingRecorder::armCentreOdometryCb(const nav_msgs::Odometry::ConstPtr& msg){
        /* Update odometry of arm center */
        pthread_rwlock_wrlock(&armc_odom_lock_);
        armc_odom_.header = msg->header;
        armc_odom_.child_frame_id = msg->child_frame_id;
        armc_odom_.pose.pose.position = msg->pose.pose.position;
        armc_odom_.pose.pose.orientation = msg->pose.pose.orientation;
        armc_odom_.twist.twist.linear = msg->twist.twist.linear;
        armc_odom_.twist.twist.angular = msg->twist.twist.angular;
        pthread_rwlock_unlock(&armc_odom_lock_);
    }

    // void DockingRecorder::armJointAngleCb(const armsauv_msgs::JointsAngle::ConstPtr& msg){}

    // void DockingRecorder::armControlInfoCb(const armsauv_msgs::ArmCtrlInfo::ConstPtr& msg){};
    
}; // for ns
