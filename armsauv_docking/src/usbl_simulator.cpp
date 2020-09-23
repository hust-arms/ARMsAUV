#include <armsauv_docking/common.h>
#include <armsauv_docking/usbl_simulator.h> 

namespace armsauv_docking{
    USBLSimulator::USBLSimulator(tf::TransformListener& tf) : tf_(tf){
        /* Create ros node */
	contentPrint<std::string>("usbl_sim", "Cereate node handle");
	
	ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // tf_ = new tf::TransformListener(ros::Duration(5));

	contentPrint<std::string>("usbl_sim", "Set parameters");

        /* Load parameters */
        private_nh.param("transceiver_frame", transc_frame_, std::string("armsauv/base_link"));
        private_nh.param("input_topic", input_topic_, std::string("/qt3/pose_gt"));
        private_nh.param("output_topic", output_topic_, std::string("/usbl/output"));
	// private_nh.param("period", period_, 0.05);

        /* Create components */
        usbl_pub_ = nh.advertise<armsauv_msgs::Usbl>(output_topic_, 1);
        transp_sub_ = nh.subscribe<nav_msgs::Odometry>(input_topic_, 1, boost::bind(&USBLSimulator::transpCb, this, _1));

	contentPrint<std::string>("usbl_sim", "Start thread");

        /* Start publish thread */
        pub_thread_ = new boost::thread(boost::bind(&USBLSimulator::outputPublish, this)); 
    }

    USBLSimulator::~USBLSimulator(){
        if(pub_thread_){
            delete pub_thread_;
            pub_thread_ = nullptr;
        }

	// if(tf_){
	//     delete tf_;
	//     tf_ = nullptr;
	// }
    }

    void USBLSimulator::transpCb(const nav_msgs::Odometry::ConstPtr& input){
        /* Process input */
	/*
        geometry_msgs::PoseStamped transp_to_world;
        std::string transp_frame = input->header.frame_id;
        transp_to_world.header.frame_id = transp_frame; 
        transp_to_world.header.stamp = input->header.stamp;
        transp_to_world.pose.position.x = input->pose.pose.position.x;
        transp_to_world.pose.position.y = input->pose.pose.position.y;
        transp_to_world.pose.position.z = input->pose.pose.position.z;
        transp_to_world.pose.orientation.x = input->pose.pose.orientation.x;
        transp_to_world.pose.orientation.y = input->pose.pose.orientation.y;
        transp_to_world.pose.orientation.z = input->pose.pose.orientation.z;
        transp_to_world.pose.orientation.w = input->pose.pose.orientation.w;
	*/
        geometry_msgs::PointStamped transp_to_world;
        std::string transp_frame = input->header.frame_id;
        transp_to_world.header.frame_id = transp_frame; 
        transp_to_world.header.stamp = input->header.stamp;
        transp_to_world.point.x = input->pose.pose.position.x;
        transp_to_world.point.y = input->pose.pose.position.y;
        transp_to_world.point.z = input->pose.pose.position.z;
        
        // geometry_msgs::PoseStamped transp_to_transc;
	geometry_msgs::PointStamped transp_to_transc; 

        /* Transform transponder pose in world frame to pose in transceiver frame */
        try{
            // tf_.transformPose(transc_frame_, ros::Time(0), transp_to_world, transp_frame, transp_to_transc);
	    tf_.transformPoint(transc_frame_, ros::Time(0), transp_to_world, transp_frame, transp_to_transc); 
	}
        catch(tf::TransformException& ex){
            ROS_ERROR("Received exception trying ot transform transponder pose to transceiver frame: %s", ex.what());
            return ;
        }

        /* Update transceiver message */
	std::cout << "[usbl_sim]:" << "local_frame:" << transp_frame << " target_frame:" << transp_to_transc.header.frame_id << std::endl;

        /*
        double d_x = transp_to_transc.pose.position.x;
        double d_y = transp_to_transc.pose.position.y;
        double d_z = transp_to_transc.pose.position.z;
        */

        double d_x = transp_to_transc.point.x;
        double d_y = transp_to_transc.point.y;
        double d_z = transp_to_transc.point.z;
	
        std::cout << "[usbl_sim]:" << "delta x:" << d_x
		                   << " delta y:" << d_y
				   << " delta z:" << d_z << std::endl;

        /*
	tf::Quaternion temp_q;
        tf::quaternionMsgToTF(transp_to_transc.pose.orientation, temp_q);

        tf::Matrix3x3 temp_m(temp_q);
        */

        double roll, pitch, yaw;
        /*
        temp_m.getRPY(roll, pitch, yaw);
	pitch = -pitch; yaw = -yaw;
        */
        roll = atan2(d_z,d_y); 
        pitch = atan2(d_z,d_x);
        yaw = atan2(d_y,d_x);

	std::cout << "[usbl_sim]:" << "roll:" << roll
		                   << " pitch:" << pitch
				   << " yaw:" << yaw << std::endl;

        std::lock_guard<std::mutex> lckg(transc_mutex_);
        transc_msg_.phi = yaw;
        transc_msg_.s_r = std::sqrt(d_x*d_x + d_y*d_y + d_z*d_z);
        transc_msg_.h_r = std::sqrt(d_x*d_x + d_y*d_y);
        transc_msg_.psi = std::atan(d_z / transc_msg_.h_r); 
    }

    void USBLSimulator::outputPublish(){
        ros::NodeHandle nh;
        while(nh.ok()){
            std::lock_guard<std::mutex> lckg(transc_mutex_);
            usbl_pub_.publish(transc_msg_); // pub

            boost::this_thread::sleep(boost::posix_time::milliseconds(period_ * 1000));
        }
    }
}; // ns
