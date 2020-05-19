#include <cstddef>
#include <cmath>
#include <thread>
#include <mutex>

#include <unistd.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/JointState.h>

#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

#include <tf/transform_datatypes.h>

using namespace std;

ros::NodeHandle *node;

uuv_gazebo_ros_plugins_msgs::FloatStamped thrusters_msg;
ros::Publisher thruster0_pub;

uuv_gazebo_ros_plugins_msgs::FloatStamped fins_msg;
ros::Publisher fin0_pub;
ros::Publisher fin1_pub;
ros::Publisher fin2_pub;
ros::Publisher fin3_pub;
ros::Publisher fin4_pub;
ros::Publisher fin5_pub;

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr &msg);
void joint_states_cb(const sensor_msgs::JointState::ConstPtr &msg);

bool control_loop_run;
void control_loop();

const uint32_t ms = 1000;
const double rad2degree = 180 / 3.1415926;
const double degree2rad = 1 / rad2degree;

/* imu info */
double roll, pitch, yaw; // rad
mutex imu_mutex;
double getRoll()
{
    lock_guard<std::mutex> guard(imu_mutex);
    return roll;
}
double getPitch()
{
    lock_guard<std::mutex> guard(imu_mutex);
    return pitch;
}
double getYaw()
{
    lock_guard<std::mutex> guard(imu_mutex);
    return yaw;
}

/* pressure info */
double depth; // m
mutex pressure_mutex;
double getDepth()
{
    lock_guard<std::mutex> guard(pressure_mutex);
    return depth;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armsauv_maneuverability");

    node = new ros::NodeHandle;

    ros::Subscriber sub_imu = node->subscribe("/armsauv/imu", 1, imu_cb);
    ros::Subscriber sub_pressure = node->subscribe("/armsauv/pressure", 1, pressure_cb);
    ros::Subscriber sub_joint_states = node->subscribe("/armsauv/joint_states", 1, joint_states_cb);

    thruster0_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/thrusters/0/input", 1);

    fin0_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/0/input", 1);
    fin1_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/1/input", 1);
    fin2_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/2/input", 1);
    fin3_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/3/input", 1);
    fin4_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/4/input", 1);
    fin5_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/5/input", 1);

    control_loop_run = true;
    static std::thread run_thread([&] { control_loop(); });

    ros::spin();

    cout << endl
         << "exiting..." << endl;
    control_loop_run = false;
    run_thread.join();

    return 0;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->orientation, quat);

    lock_guard<std::mutex> guard(imu_mutex);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double rollspeed, pitchspeed, yawspeed;
    rollspeed = msg->angular_velocity.x;
    pitchspeed = msg->angular_velocity.y;
    yawspeed = msg->angular_velocity.z;

    ROS_INFO("roll: %f, pitch: %f, yaw: %f.",
             roll * rad2degree,
             pitch * rad2degree,
             yaw * rad2degree);
}

void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    depth = (double)((msg->fluid_pressure - 101) / 10.1) - 0.25;

    // ROS_INFO("depth: %f m", depth);
}

void joint_states_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    // ROS_INFO("joint_states.");
}

void control_loop()
{
    uint32_t seq;

    std_msgs::Header header;

// #define xy_z_test
//     bool turnLeft = true;
//     const double xy_z_test_value = 20 * degree2rad;

#define yz_z_test
    bool dive = true;
const double yz_z_test_value = 10 * degree2rad;

    while (control_loop_run)
    {
        usleep(200 * ms);
        ROS_INFO("control_loop");

        header.stamp.setNow(ros::Time::now());
        header.frame_id = "base_link";
        header.seq = seq++;

        thrusters_msg.header = header;

        // forward
        thrusters_msg.data = 200.0;
        thruster0_pub.publish(thrusters_msg);

        fins_msg.header = header;

#ifdef xy_z_test

        if (turnLeft)
        {
            if (getYaw() >= xy_z_test_value)
            {
                turnLeft = false;
                fins_msg.data = -xy_z_test_value;
                fin3_pub.publish(fins_msg);
                fins_msg.data = xy_z_test_value;
                fin5_pub.publish(fins_msg);
            }
            else
            {
                fins_msg.data = xy_z_test_value;
                fin3_pub.publish(fins_msg);
                fins_msg.data = -xy_z_test_value;
                fin5_pub.publish(fins_msg);
            }
        }
        else
        {
            if (getYaw() <= -xy_z_test_value)
            {

                turnLeft = true;
                fins_msg.data = xy_z_test_value;
                fin3_pub.publish(fins_msg);
                fins_msg.data = -xy_z_test_value;
                fin5_pub.publish(fins_msg);
            }
            else
            {
                fins_msg.data = -xy_z_test_value;
                fin3_pub.publish(fins_msg);
                fins_msg.data = xy_z_test_value;
                fin5_pub.publish(fins_msg);
            }
        }
#endif

#ifdef yz_z_test

        if (dive)
        {
            if (getPitch() >= yz_z_test_value)
            {
                dive = false;
                fins_msg.data = -yz_z_test_value;
                fin2_pub.publish(fins_msg);
                fins_msg.data = yz_z_test_value;
                fin4_pub.publish(fins_msg);
            }
            else
            {
                fins_msg.data = yz_z_test_value;
                fin2_pub.publish(fins_msg);
                fins_msg.data = -yz_z_test_value;
                fin4_pub.publish(fins_msg);
            }
        }
        else
        {
            if (getPitch() <= -yz_z_test_value)
            {

                dive = true;
                fins_msg.data = yz_z_test_value;
                fin2_pub.publish(fins_msg);
                fins_msg.data = -yz_z_test_value;
                fin4_pub.publish(fins_msg);
            }
            else
            {
                fins_msg.data = -yz_z_test_value;
                fin2_pub.publish(fins_msg);
                fins_msg.data = yz_z_test_value;
                fin4_pub.publish(fins_msg);
            }
        }
#endif
    }
}
