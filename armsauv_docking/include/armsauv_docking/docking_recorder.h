#ifndef DOCKING_RECORDER_H_
#define DOCKING_RECORDER_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

#include <sstream>

#include <string>

#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time/gregorian/gregorian.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <armsauv_msgs/Usbl.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

#include <armsauv_docking/file_writer.h>
#include <armsauv_docking/common.h>

namespace armsauv_docking{
    /**
     * @brief Recorder for docking 
     */ 
    class DockingRecorder{
    public:
        /**
         * @brief Constructor
         */ 
        DockingRecorder();

        /**
         * @brief Deconstructor
         */ 
        ~DockingRecorder();

    private:
        /**
         * @brief Data writing thread 
         */ 
        void recordThread();

        /**
         * @brief Data writing thread 
         */ 
        void recordAUVThread();

        /**
         * @brief Serialize all data
         */ 
        std::string serializeData();
         
        /**
         * @brief Serialize all data
         */ 
        std::string serializeAUVData();
    
        /**
         * @brief Receive AUV odometry
         * @msg AUV odometry
         */ 
        void auvOdometryCb(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief Receive simulated USBL messages
         * @msg USBL message
         */ 
        void usblMessageCb(const armsauv_msgs::Usbl::ConstPtr& msg);

        /**
         * @brief Receive information of AUV control
         */ 
        // void auvControlInfoCb(const armsauv_msgs::CtrlInfo::ConstPtr& msg);

        /**
         * @brief Receive vehilce odometry
         * @msg Vehicle odometry
         */ 
        void vehicleOdometryCb(const nav_msgs::Odometry::ConstPtr& msg);
 
        /**
         * @brief Receive odometry of the arm center
         * @msg Odometry of arm center
         */ 
        void armCentreOdometryCb(const nav_msgs::Odometry::ConstPtr& msg);

        /**
         * @brief Receive joints angle of the arm
         * @msg Joints angle of arm
         */ 
        // void armJointsAngleCb(const armsauv_msgs::JointsAngle::ConstPtr& msg);

        /**
         * @brief Receive information of arm control
         * @msg Control information of manipulator
         */ 
        // void armControlInfoCb(const armsauv_msgs::ArmCtrlInfo::ConstPtr& msg);


    private:
        /* fundamental property */
	FileWriter* fw_;
        std::string filename_;
        std::string path_;
        int rec_freq_;
	ros::Time time0_;
        boost::thread* rec_thread_;

        /* ros components */
        ros::Subscriber auv_odom_sub_;
        ros::Subscriber usbl_msg_sub_;
        ros::Subscriber auv_ctrl_info_sub_;
        ros::Subscriber veh_odom_sub_;
        ros::Subscriber armc_odom_sub_;
        ros::Subscriber arm_joints_ang_sub_;
        ros::Subscriber arm_ctrl_info_sub_;

        /* messages */
        nav_msgs::Odometry auv_odom_;
        armsauv_msgs::Usbl usbl_msg_;
        // armsauv_msgs::CtrlInfo auv_ctrl_msg_;
        nav_msgs::Odometry veh_odom_;
        nav_msgs::Odometry armc_odom_;
        // armsauv_msgs::JointsAngle arm_joints_ang_;
        // armsauv_msgs::ArmCtrlInfo arm_ctrl_info_;

        /* rwlock */
        pthread_rwlock_t auv_odom_lock_ = PTHREAD_RWLOCK_INITIALIZER;
        pthread_rwlock_t usbl_msg_lock_ = PTHREAD_RWLOCK_INITIALIZER;
        pthread_rwlock_t auv_ctrl_lock_ = PTHREAD_RWLOCK_INITIALIZER;
        pthread_rwlock_t veh_odom_lock_ = PTHREAD_RWLOCK_INITIALIZER;
        pthread_rwlock_t armc_odom_lock_ = PTHREAD_RWLOCK_INITIALIZER;
        pthread_rwlock_t arm_joints_ang_lock_ = PTHREAD_RWLOCK_INITIALIZER;
        pthread_rwlock_t arm_ctrl_info_lock_ = PTHREAD_RWLOCK_INITIALIZER;
    
    }; // for DockingRecorder

}; // ns

#endif
