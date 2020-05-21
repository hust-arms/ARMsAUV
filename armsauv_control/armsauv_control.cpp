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
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>

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

/* sensors cb */
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg);
void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr &msg);
void posegt_cb(const nav_msgs::Odometry::ConstPtr &msg);
void dvl_cb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr &msg);

/* input cb */
void depth_cb(const std_msgs::Float64::ConstPtr &msg);
void pitch_cb(const std_msgs::Float64::ConstPtr &msg);
void yaw_cb(const std_msgs::Float64::ConstPtr &msg);

bool control_loop_run;
void control_loop();

typedef struct _sensors_data
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double x_speed;
    double y_speed;
    double z_speed;
    double roll_speed;
    double pitch_speed;
    double yaw_speed;
} sensors_data;

typedef struct _controler_input
{
    double depth;
    double pitch;
    double yaw;
} controler_input;

typedef struct _controler_output
{
    double rouder;
    double fwd_fin;
    double aft_fin;
} controler_output;

void controler_run(sensors_data *sensors, controler_input *input, controler_output *output);
void applyActuatorInput(double rouder, double fwd_fin, double aft_fin, double rpm);

const uint32_t ms = 1000;
const double rad2degree = 180 / 3.1415926;
const double degree2rad = 1 / rad2degree;

/* imu info */
double roll, pitch, yaw;                // rad
double rollspeed, pitchspeed, yawspeed; // rad/s
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
double getRollSpeed()
{
    lock_guard<std::mutex> guard(imu_mutex);
    return rollspeed;
}
double getPitchSpeed()
{
    lock_guard<std::mutex> guard(imu_mutex);
    return pitchspeed;
}
double getYawSpeed()
{
    lock_guard<std::mutex> guard(imu_mutex);
    return yawspeed;
}

/* pressure info */
double depth; // m
mutex pressure_mutex;
double getDepth()
{
    lock_guard<std::mutex> guard(pressure_mutex);
    return depth;
}

/* posegt info */
double x, y, z;
mutex posegt_mutex;
double getX()
{
    lock_guard<std::mutex> guard(posegt_mutex);
    return x;
}
double getY()
{
    lock_guard<std::mutex> guard(posegt_mutex);
    return y;
}
double getZ()
{
    lock_guard<std::mutex> guard(posegt_mutex);
    return z;
}

/* dvl info */
double x_speed, y_speed, z_speed;
mutex dvl_mutex;
double getXspeed()
{
    lock_guard<std::mutex> guard(dvl_mutex);
    return x_speed;
}
double getYspeed()
{
    lock_guard<std::mutex> guard(dvl_mutex);
    return y_speed;
}
double getZspeed()
{
    lock_guard<std::mutex> guard(dvl_mutex);
    return z_speed;
}

/* input info */
double depth_input, pitch_input, yaw_input;
mutex depth_input_mutex, pitch_input_mutex, yaw_input_mutex;
double getDepthInput()
{
    lock_guard<std::mutex> guard(depth_input_mutex);
    return depth_input;
}
double getPitchInput()
{
    lock_guard<std::mutex> guard(pitch_input_mutex);
    return pitch_input;
}
double getYawInput()
{
    lock_guard<std::mutex> guard(yaw_input_mutex);
    return yaw_input;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "armsauv_maneuverability");

    node = new ros::NodeHandle;

    /* sensors */
    ros::Subscriber sub_imu = node->subscribe("/armsauv/imu", 1, imu_cb);
    ros::Subscriber sub_pressure = node->subscribe("/armsauv/pressure", 1, pressure_cb);
    ros::Subscriber sub_posegt = node->subscribe("/armsauv/pose_gt", 1, posegt_cb);
    ros::Subscriber sub_dvl = node->subscribe("/armsauv/dvl", 1, dvl_cb);

    /* input */
    ros::Subscriber sub_depth = node->subscribe("/armsauv/control_input/depth", 1, depth_cb);
    ros::Subscriber sub_pitch = node->subscribe("/armsauv/control_input/pitch", 1, pitch_cb);
    ros::Subscriber sub_yaw = node->subscribe("/armsauv/control_input/yaw", 1, yaw_cb);

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

    rollspeed = msg->angular_velocity.x;
    pitchspeed = msg->angular_velocity.y;
    yawspeed = msg->angular_velocity.z;

    // ROS_INFO("roll: %f, pitch: %f, yaw: %f.",
    //          roll,
    //          pitch,
    //          yaw);
    // ROS_INFO("rollspeed: %f, pitchspeed: %f, yawspeed: %f.",
    //          rollspeed,
    //          pitchspeed,
    //          yawspeed);

    /* for convenience */
    ROS_INFO("roll: %f, pitch: %f, yaw: %f.",
             roll * rad2degree,
             pitch * rad2degree,
             yaw * rad2degree);
}

void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(pressure_mutex);

    depth = (double)((msg->fluid_pressure - 101) / 10.1) - 0.25;

    ROS_INFO("depth: %f m", depth);
}

void posegt_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(posegt_mutex);

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    ROS_INFO("x: %f, y: %f, z: %f.",
             x,
             y,
             z);
}

void dvl_cb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(dvl_mutex);

    x_speed = msg->velocity.x;
    y_speed = msg->velocity.y;
    z_speed = msg->velocity.z;

    ROS_INFO("x_speed: %f, y_speed: %f, z_speed: %f.",
             x_speed,
             y_speed,
             z_speed);
}

void depth_cb(const std_msgs::Float64::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(depth_input_mutex);

    depth_input = msg->data;

    ROS_INFO("depth_input: %f", depth_input);
}

void pitch_cb(const std_msgs::Float64::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(pitch_input_mutex);

    pitch_input = msg->data;

    ROS_INFO("pitch_input: %f", pitch_input);
}

void yaw_cb(const std_msgs::Float64::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(yaw_input_mutex);

    yaw_input = msg->data;

    ROS_INFO("yaw_input: %f", yaw_input);
}

void control_loop()
{
    sensors_data *sensors = new sensors_data;
    controler_input *input = new controler_input;
    controler_output *output = new controler_output;

    // output->rouder = 0.5;
    // output->fwd_fin = 0.5;
    // output->aft_fin = 0.5;

    while (control_loop_run)
    {
        usleep(200 * ms);
        ROS_INFO("control_loop");

        sensors->x = getX();
        sensors->y = getY();
        sensors->z = getZ();
        sensors->roll = getRoll();
        sensors->pitch = getPitch();
        sensors->yaw = getYaw();
        sensors->x_speed = getXspeed();
        sensors->y_speed = getYspeed();
        sensors->z_speed = getZspeed();
        sensors->roll_speed = getRollSpeed();
        sensors->pitch_speed = getPitchSpeed();
        sensors->yaw_speed = getYawSpeed();

        input->depth = getDepthInput();
        input->pitch = getPitchInput();
        input->yaw = getYawInput();

        controler_run(sensors, input, output);

        applyActuatorInput(output->rouder, output->fwd_fin, output->aft_fin, 0);
    }

    delete sensors;
    delete input;
    delete output;
}

void controler_run(sensors_data *sensors, controler_input *input, controler_output *output)
{
}

void applyActuatorInput(double rouder, double fwd_fin, double aft_fin, double rpm)
{
    static uint32_t seq;

    std_msgs::Header header;
    header.stamp.setNow(ros::Time::now());
    header.frame_id = "base_link";
    header.seq = seq++;

    thrusters_msg.header = header;
    fins_msg.header = header;

    thrusters_msg.data = rpm;
    // thrusters_msg.data = 200;
    // thruster0_pub.publish(thrusters_msg);

    // rouder
    fins_msg.data = rouder;
    fin3_pub.publish(fins_msg);
    fins_msg.data = -rouder;
    fin5_pub.publish(fins_msg);

    // fwd_fin
    fins_msg.data = fwd_fin;
    fin0_pub.publish(fins_msg);
    fins_msg.data = -fwd_fin;
    fin1_pub.publish(fins_msg);

    // aft_fin
    fins_msg.data = aft_fin;
    fin2_pub.publish(fins_msg);
    fins_msg.data = -aft_fin;
    fin4_pub.publish(fins_msg);
}
