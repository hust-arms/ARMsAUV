#ifndef USBL_NOTRAW_SIMULATOR_H_
#define USBL_NOTRAW_SIMULATOR_H_

#include <math.h>
#include <string>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <thread>
#include <mutex>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
// #include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
// #include <armsauv_msgs/Usbl.h>
#include <armsauv_msgs/UsblNotRaw.h>


namespace armsauv_docking{

    /**
     * @brief USBL simulator
     */ 
    class USBLSimulator{
    public:
        /**
         * @brief Initialize
         * @param tf Transformation tree 
         */ 
        USBLSimulator(tf::TransformListener& tf);

        /**
         * @brief Destructor
         */ 
        ~USBLSimulator();

    private:
        /**
         * @brief Get input message from dummy USBL transponder
         */
        void transpCb(const nav_msgs::Odometry::ConstPtr& input); 

        /**
         * @brief Publish dummy USBL transceiver message
         */
        void outputPublish();


    private:
        std::string transc_frame_; // target frame

        std::string input_topic_;
        std::string output_topic_;

        boost::thread* pub_thread_;
        std::mutex transc_mutex_;

        tf::TransformListener& tf_; 
        ros::Publisher usbl_pub_;
        ros::Subscriber transp_sub_; // input message subscriber

        armsauv_msgs::UsblNotRaw transc_msg_; // output message

        double period_; // s 
    }; // class

}; // ns

#endif
