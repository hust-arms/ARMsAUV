#include <cstddef>
#include <cmath>
#include <thread>
#include <mutex>

#include <math.h>

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

/* timer cb */
void timer_cb(const ros::TimerEvent &event);

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
    double x;           //艇体x坐标，单位m
    double y;           //艇体y坐标，单位m
    double z;           //艇体z坐标，单位m
    double roll;        //横滚角，单位rad
    double pitch;       //纵倾角，单位rad
    double yaw;         //方向角，单位rad
    double x_speed;     //艏向速度，单位m/s
    double y_speed;     //横向速度，单位m/s
    double z_speed;     //垂向速度，单位m/s
    double roll_speed;  //横滚角速度，单位rad/s
    double pitch_speed; //纵倾角速度，单位rad/s
    double yaw_speed;   //方向角速度，单位rad/s
} sensors_data;

typedef struct _controler_input
{
    double depth; //期望深度
    double pitch; //期望纵倾角
    double yaw;   //期望航路的方向角
    double x_d;   //期望航路上某一点的x坐标
    double y_d;   //期望航路上某一点的y坐标
} controler_input;

typedef struct _controler_output
{
    double rouder;  //方向舵
    double fwd_fin; //艏舵舵角
    double aft_fin; //艉舵舵角
} controler_output;

sensors_data *sensors;
controler_input *input;
controler_output *output;

void controler_run(sensors_data *sensors, controler_input *input, controler_output *output, double dt);
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

    // control_loop_run = true;
    // static std::thread run_thread([&] { control_loop(); });

    sensors = new sensors_data;
    input = new controler_input;
    output = new controler_output;

    ros::Timer timer = node->createTimer(ros::Duration(0.1), timer_cb);

    ros::spin();

    // cout << endl
    //      << "exiting..." << endl;
    // control_loop_run = false;
    // run_thread.join();

    delete sensors;
    delete input;
    delete output;

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

    // /* for convenience */
    // ROS_INFO("roll: %f, pitch: %f, yaw: %f.",
    //          roll * rad2degree,
    //          pitch * rad2degree,
    //          yaw * rad2degree);
}

void pressure_cb(const sensor_msgs::FluidPressure::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(pressure_mutex);

    depth = (double)((msg->fluid_pressure - 101) / 10.1) - 0.25;

    // ROS_INFO("depth: %f m", depth);
}

void posegt_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(posegt_mutex);

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;

    // ROS_INFO("x: %f, y: %f, z: %f.",
    //          x,
    //          y,
    //          z);
}

void dvl_cb(const uuv_sensor_ros_plugins_msgs::DVL::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(dvl_mutex);

    x_speed = msg->velocity.x;
    y_speed = msg->velocity.y;
    z_speed = msg->velocity.z;

    // ROS_INFO("x_speed: %f, y_speed: %f, z_speed: %f.",
    //          x_speed,
    //          y_speed,
    //          z_speed);
}

void depth_cb(const std_msgs::Float64::ConstPtr &msg)
{
   lock_guard<std::mutex> guard(depth_input_mutex);

    depth_input = msg->data;
   std::cout << "test";
   // ROS_INFO("depth_input: %f", depth_input);
}

void pitch_cb(const std_msgs::Float64::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(pitch_input_mutex);

    pitch_input = msg->data;

    // ROS_INFO("pitch_input: %f", pitch_input);
}

void yaw_cb(const std_msgs::Float64::ConstPtr &msg)
{
    lock_guard<std::mutex> guard(yaw_input_mutex);

    yaw_input = msg->data;

    // ROS_INFO("yaw_input: %f", yaw_input);
}

void control_loop()
{
    sleep(1);

    sensors_data *sensors = new sensors_data;
    controler_input *input = new controler_input;
    controler_output *output = new controler_output;

    // output->rouder = 0.5;
    // output->fwd_fin = 0.5;
    // output->aft_fin = 0.5;

    while (control_loop_run)
    {
        usleep(100 * ms);
        // ROS_INFO("control_loop");

        sensors->x = getX();
        sensors->y = -getY();
        sensors->z = -getZ();
        sensors->roll = getRoll();
        sensors->pitch = -getPitch();
        sensors->yaw = -getYaw();
        sensors->x_speed = getXspeed();
        sensors->y_speed = -getYspeed();
        sensors->z_speed = -getZspeed();
        sensors->roll_speed = getRollSpeed();
        sensors->pitch_speed = -getPitchSpeed();
        sensors->yaw_speed = -getYawSpeed();

        input->x_d = 0;
        input->y_d = 0;
        input->depth = 0.0;
        input->pitch = 0.0;
        input->yaw = 10 * degree2rad;

        controler_run(sensors, input, output, 0.1);

        // std::cout << output->rouder << " " << output->fwd_fin << " " << output->aft_fin << std::endl;

        std::cout << sensors->x << " "
                  << sensors->y << " "
                  << sensors->z << " "
                  //   << "| "
                  << sensors->roll * rad2degree << " "
                  << sensors->pitch * rad2degree << " "
                  << sensors->yaw * rad2degree << " "
                  //   << "| "
                  << sensors->x_speed << " "
                  << sensors->y_speed << " "
                  << sensors->z_speed << " "
                  //   << "| "
                  << sensors->roll_speed * rad2degree << " "
                  << sensors->pitch_speed * rad2degree << " "
                  << sensors->yaw_speed * rad2degree << " "
                  //   << "| "
                  << output->rouder * rad2degree << " "
                  << output->fwd_fin * rad2degree << " "
                  << output->aft_fin * rad2degree << " "
            //   << std::endl
            //   << std::endl;
            ;

        // std::cout << output->rouder * rad2degree << " "
        //           << output->fwd_fin * rad2degree << " "
        //           << output->aft_fin * rad2degree << " "
        //         //   << std::endl
        //           << std::endl;

        applyActuatorInput(
            output->rouder,
            // output->fwd_fin,
            // output->aft_fin,
            // 0 * degree2rad,
            0 * degree2rad,
            0 * degree2rad,
            1000);
    }

    delete sensors;
    delete input;
    delete output;
}

void timer_cb(const ros::TimerEvent &event)
{
    if (getX() == 0 || getXspeed() == 0)
    {
        return;
    }

    sensors->x = getX();
    sensors->y = -getY();
    sensors->z = -getZ();
    sensors->roll = getRoll();
    sensors->pitch = -getPitch();
    sensors->yaw = -getYaw();
    sensors->x_speed = getXspeed();
    sensors->y_speed = -getYspeed();
    sensors->z_speed = -getZspeed();
    sensors->roll_speed = getRollSpeed();
    sensors->pitch_speed = -getPitchSpeed();
    sensors->yaw_speed = -getYawSpeed();

    input->x_d = 30;
    input->y_d = 0;
    input->depth = 2.0;
    input->pitch = 0.0;
    input->yaw = 30 * degree2rad;

    std::cout << sensors->x << " "
              << sensors->y << " "
              << sensors->z << " "
              //   << "| "
              << sensors->roll * rad2degree << " "
              << sensors->pitch * rad2degree << " "
              << sensors->yaw * rad2degree << " "
              //   << "| "
              << sensors->x_speed << " "
              << sensors->y_speed << " "
              << sensors->z_speed << " "
              //   << "| "
              << sensors->roll_speed * rad2degree << " "
              << sensors->pitch_speed * rad2degree << " "
              << sensors->yaw_speed * rad2degree << " ";

    controler_run(sensors, input, output, 0.1);

    static double rouder, fwd_fin, aft_fin;

    // rouder = output->rouder;
    // fwd_fin = output->fwd_fin;
    // aft_fin = output->aft_fin;

    static bool firstRun = true;
    static bool zflag1 = false;
    static bool zflag2 = false;


// #define xz_test
#ifdef xz_test

    if (firstRun && sensors->x_speed > 2.0)
    {
        rouder = 0;
        fwd_fin = -10 * degree2rad;
        aft_fin = -10 * degree2rad;
        firstRun = false;
    }

    if (sensors->pitch * rad2degree < -10)
    {
        rouder = 0;
        fwd_fin = 10 * degree2rad;
        aft_fin = 10 * degree2rad;
    }

    if (sensors->pitch * rad2degree > 10)
    {
        rouder = 0;
        fwd_fin = -10 * degree2rad;
        aft_fin = -10 * degree2rad;
    }

#endif

// #define xy_test
#ifdef xy_test

    if (sensors->yaw * rad2degree < -10 )
    {
	zflag1=true;
	zflag2=false;
    }

    if(zflag1)
    {
    	rouder = rouder - (0.5*degree2rad);
        if(rouder < -10*degree2rad)
	{	
	rouder = -10 * degree2rad;
	}

        fwd_fin = 0;
        aft_fin = 0;
    }

    if (sensors->yaw * rad2degree > 10)
    {
	    zflag1=false;
	    zflag2=true;
    }

    if(zflag2)
    {
	rouder = rouder + (0.5*degree2rad);
	if(rouder > 10*degree2rad)
	{
        rouder = 10 * degree2rad;
	}
        fwd_fin = 0;
        aft_fin = 0;
    }

        if (firstRun && sensors->x_speed > 2.0)
    {
        rouder = 10 * degree2rad;
        fwd_fin = 0;
        aft_fin = 0;
        firstRun = false;
    }


#endif

//#define rouder_test
#ifdef rouder_test
    static double start_time = 0;
    double now_time = 0;
    if (firstRun && sensors->x_speed > 2.0)
    {
        start_time = ros::Time::now().toSec();
        firstRun = false;
    }

    if (!firstRun)
    {
        now_time = ros::Time::now().toSec();

        if (now_time - start_time > 30)
        {
            rouder = 0;
            fwd_fin = 0;
            aft_fin = 0;
        }
        else
        {
            rouder = 20 * degree2rad;
            fwd_fin = 0;
            aft_fin = 0;
        }
    }
#endif


/*    if (firstRun && sensors->x_speed > 6)
    {
        firstRun = false;
    }
    else
    {
        rouder = 0 * degree2rad;
        fwd_fin = 0 * degree2rad;
        aft_fin = 0 * degree2rad;
    }

    if (!firstRun)
    {
        rouder = 20 * degree2rad;
        fwd_fin = -10 * degree2rad;
        aft_fin = -10 * degree2rad;
    }
*/
    applyActuatorInput(
        rouder,
        // 0,
        fwd_fin,
        aft_fin,
        // 20 * degree2rad,
        // -5 * degree2rad,
        // 5 * degree2rad,
        1250);

    std::cout << rouder * rad2degree << " "
              << fwd_fin * rad2degree << " "
              << aft_fin * rad2degree << " "
              << std::endl;
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
    thruster0_pub.publish(thrusters_msg);

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
    fins_msg.data = -aft_fin;
    fin2_pub.publish(fins_msg);
    fins_msg.data = aft_fin;
    fin4_pub.publish(fins_msg);
}

double sat(double input, double thick);
int sign(double input);

void controler_run(sensors_data *sensors, controler_input *input, controler_output *output, double dt)
{
    //艇体本体参数
    static double m = 84.71, L = 2.712, W = 831, B = 838;                    //质量/长度/重力和浮力,实际模型中B=838
    static double x_B = 0, y_B = 0, z_B = 0, x_G = 0, y_G = 0, z_G = 0.0086; //浮心和重心坐标
    static double I_xx = 0.82, I_yy = 30.14, I_zz = 30.14;                   //惯性矩

    //水动力参数
    static double X_dotu = -1.432, Y_dotv = -120.645, Y_dotr = -4.895, Z_dotw = -130.513, Z_dotq = 16.488;
    static double K_dotp = -0.386, M_dotw = 16.488, M_dotq = -78.266, N_dotv = -4.895, N_dotr = -67.489;
    static double X_uu = -3.9, Y_vv = -373.287, Y_rr = -4.204, Z_ww = -489.07, Z_qq = 23.016;
    static double K_pp = -0.1177, M_ww = 23.342, M_qq = -353.406, N_vv = 0.4193, N_rr = -227.024;
    static double Y_uv = -130.64, Y_ur = 40.25, Z_uw = -522.87, Z_uq = 4.27;
    static double M_uw = 140.68, M_uq = 73, N_uv = -57.47, N_ur = -50.3;

    //控制力参数
    static double Y_uudr = 38.279, Z_uuds = -38.279, Z_uudb = -44.981;
    static double M_uuds = 41.686, M_uudb = -44.531, N_uudr = -41.686;

    //控制参数
    static double c_z = 0.08, k_z = 0.1, alpha_z = 0.6;             //深度通道参数
    static double c_theta = 0.1, k_theta = 0.1, alpha_theta = 0.6; //纵倾通道参数
    static double c_psi = 0.8, k_psi = 0.8, alpha_psi = 0.8;       //航向通道参数
    static double boundary_thick = 0.1;                             //边界层厚度

    //艇体位姿和速度变量
    static double x, y, z, phi, theta, psi, u, v, w, p, q, r;

    //控制任务信息变量
    static double REFz, REFdot_z, REFdot2_z;             //期望深度及其一阶和二阶导数
    static double preREFz, preREFdot_z;                  //上一控制周期的期望深度及其一阶导
    static double REFtheta, REFdot_theta, REFdot2_theta; //期望纵倾及其一阶和二阶导数
    static double preREFtheta, preREFdot_theta;          //上一控制周期的期望纵倾及其一阶导
    static double REFpsi, REFdot_psi, REFdot2_psi;       //期望航向及其一阶和二阶导数
    static double preREFpsi, preREFdot_psi;              //上一控制周期的期望航行及其一阶导
    static double lateral_dis;                           //航行器距离目标航向的横向偏距

    //艇体位姿和速度信息提取
    x = sensors->x;
    y = sensors->y;
    z = sensors->z;
    phi = sensors->roll;
    theta = sensors->pitch;
    psi = sensors->yaw;
    u = sensors->x_speed;
    v = sensors->y_speed;
    w = sensors->z_speed;
    p = sensors->roll_speed;
    q = sensors->pitch_speed;
    r = sensors->yaw_speed;

    //控制任务信息提取和计算
    REFz = input->depth;    //深度信息
    REFdot_z = 0;           //深度一阶导
    REFdot2_z = 0;          //深度二阶导
    preREFz = REFz;         //更新上一周期的深度值
    preREFdot_z = REFdot_z; //更新上一周期深度一阶导

    REFtheta = input->pitch + 0.1 * atan((z - REFz) / (4 * L)); //期望的纵倾加上纵倾制导量
    REFdot_theta = (REFtheta - preREFtheta) / dt;
    REFdot2_theta = (REFdot_theta - preREFdot_theta) / dt;
    preREFtheta = REFtheta;
    preREFdot_theta = REFdot_theta;

    lateral_dis = (x - input->x_d) * sin(input->yaw) - (y - input->y_d) * cos(input->yaw); //计算横向偏距
    std::cout << lateral_dis << " ";
    REFpsi = input->yaw + atan(lateral_dis / 10);
    //REFpsi = input->yaw;
    REFdot_psi = (REFpsi - preREFpsi) / dt;
    REFdot2_psi = (REFdot_psi - preREFdot_psi) / dt;
    //REFdot_psi = 0;
    //REFdot2_psi = 0;
    preREFpsi = REFpsi;
    preREFdot_psi = REFdot_psi;

    //艇体深度面状态变量
    static double a_zw, a_zq, a_zs, a_zb, f_z;      //垂向运动方程中的变量替换
    static double a_tw, a_tq, a_ts, a_tb, f_t;      //纵倾运动方程变量替换
    static double b_z, b_zb, b_zs, b_t, b_tb, b_ts; //dot_w和dot_q表达式中的量
    static double g_z, g_zb, g_zs, g_t, g_tb, g_ts; //dot2_z和dot2_theta表达式中的量
    static double dot_z, dot_theta;                 //深度及纵倾的一阶导

    //垂向运动方程变量替换
    a_zw = m - Z_dotw;
    a_zq = -(m * x_G + Z_dotq);
    a_zs = Z_uuds * u * u;
    a_zb = Z_uudb * u * u;
    f_z = m * u * q + m * z_G * q * q - X_dotu * u * q + Z_ww * w * fabs(w) + Z_uw * u * w + Z_qq * q * fabs(q) + Z_uq * u * q + (W - B) * cos(theta);
    //纵倾运动方程变量替换(变量中的t为theta的缩写)
    a_tw = -(m * x_G + M_dotw);
    a_tq = I_yy - M_dotq;
    a_ts = M_uuds * u * u;
    a_tb = M_uudb * u * u;
    f_t = -m * z_G * w * q - m * x_G * u * q - (Z_dotw * w + Z_dotq * q) * u + X_dotu * u * w + M_ww * w * fabs(w) + M_uw * u * w + M_qq * q * fabs(q) + M_uq * u * q - (z_G * W - z_B * B) * sin(theta) - (x_G * W - x_B * B) * cos(theta);
    //dot_w和dot_q表达式中的量
    b_z = (a_tq * f_z - a_zq * f_t) / (a_zw * a_tq - a_zq * a_tw);
    b_zb = (a_tq * a_zb - a_zq * a_tb) / (a_zw * a_tq - a_zq * a_tw);
    b_zs = (a_tq * a_zs - a_zq * a_ts) / (a_zw * a_tq - a_zq * a_tw);
    b_t = (a_zw * f_t - a_tw * f_z) / (a_zw * a_tq - a_zq * a_tw);
    b_tb = (a_zw * a_tb - a_tw * a_zb) / (a_zw * a_tq - a_zq * a_tw);
    b_ts = (a_zw * a_ts - a_tw * a_zs) / (a_zw * a_tq - a_zq * a_tw);
    //dot2_z和dot2_theta表达式中的量
    g_z = b_z * cos(theta) - u * q * cos(theta) - w * q * sin(theta);
    g_zb = b_zb * cos(theta);
    g_zs = b_zs * cos(theta);
    g_t = b_t;
    g_tb = b_tb;
    g_ts = b_ts;
    //深度及纵倾的一阶导
    dot_z = -u * sin(theta) + w * cos(theta);
    dot_theta = q;

    //艇体水平面状态变量
    static double a_yv, a_yr, a_ydr, f_y; //横向运动方程变量替换
    static double a_pv, a_pr, a_pdr, f_p; //横摇运动方程变量替换
    static double b_y, b_ydr, b_p, b_pdr; //dot_v和dot_r表达式中的量
    static double dot_psi;                //航向psi的一阶导

    //横向运动方程变量替换(变量中的d为delta的缩写,p为psi的缩写)
    a_yv = m - Y_dotv;
    a_yr = m * x_G - Y_dotr;
    a_ydr = Y_uudr * u * u;
    f_y = m * y_G * r * r - m * u * r + X_dotu * u * r + Y_vv * v * fabs(v) + Y_uv * u * v + Y_rr * r * fabs(r) + Y_ur * u * r;
    //横摇运动方程变量替换
    a_pv = m * x_G - N_dotv;
    a_pr = I_zz - N_dotr;
    a_pdr = N_uudr * u * u;
    f_p = -m * x_G * u * r - m * y_G * v * r + (Y_dotv * v + Y_dotr * r) * u - X_dotu * u * v + N_vv * v * fabs(v) + N_uv * u * v + N_rr * r * fabs(r) + N_ur * u * r;
    //dot_v和dot_r表达式中的量
    b_y = (a_pr * f_y - a_yr * f_p) / (a_yv * a_pr - a_pv * a_yr);
    b_ydr = (a_pr * a_ydr - a_yr * a_pdr) / (a_yv * a_pr - a_pv * a_yr);
    b_p = (a_yv * f_p - a_pv * f_y) / (a_yv * a_pr - a_pv * a_yr);
    b_pdr = (a_yv * a_pdr - a_pv * a_ydr) / (a_yv * a_pr - a_pv * a_yr);
    //航向psi的一阶导
    dot_psi = r;

    //滑模控制
    static double e_z, dot_e_z, S_z;
    static double e_theta, dot_e_theta, S_theta;
    static double e_psi, dot_e_psi, S_psi;
    static double L_z, L_theta, L_psi;

    //深度
    e_z = z - REFz;
    dot_e_z = dot_z - REFdot_z;
    S_z = dot_e_z + c_z * e_z;

    //纵倾
    e_theta = theta - REFtheta;
    dot_e_theta = dot_theta - REFdot_theta;
    S_theta = dot_e_theta + c_theta * e_theta;

    //航向
    e_psi = psi - REFpsi;
    dot_e_psi = dot_psi - REFdot_psi;
    S_psi = dot_e_psi + c_psi * e_psi;

    std::cout << REFz << " "
              << REFtheta * 57.3 << " "
              << REFpsi * 57.3 << " "
              << e_z << " "
              << e_theta * 57.3 << " "
              << e_psi * 57.3 << " ";

    //指令计算公式中的中间量
    L_z = REFdot2_z - g_z - c_z * dot_e_z - k_z * pow(fabs(S_z), alpha_z) * sat(S_z, boundary_thick);
    L_theta = REFdot2_theta - g_t - c_theta * dot_e_theta - k_theta * pow(fabs(S_theta), alpha_theta) * sat(S_theta, boundary_thick);
    L_psi = REFdot2_psi - b_p - c_psi * dot_e_psi - k_psi * pow(fabs(S_psi), alpha_psi) * sat(S_psi, boundary_thick);

    //指令计算
    static double deltab, deltas, deltar;
    deltab = (L_z * g_ts - L_theta * g_zs) / (g_zb * g_ts - g_tb * g_zs);
    deltas = (L_theta * g_zb - L_z * g_tb) / (g_zb * g_ts - g_tb * g_zs);
    deltar = L_psi / b_pdr;

    if (fabs(deltab) > 30 / 57.3)
        deltab = (30 / 57.3) * sign(deltab);

    if (fabs(deltas) > 30 / 57.3)
        deltas = (30 / 57.3) * sign(deltas);

    if (fabs(deltar) > 30 / 57.3)
        deltar = (30 / 57.3) * sign(deltar);

    output->fwd_fin = deltab;
    output->aft_fin = deltas;
    output->rouder = deltar;
}

double sat(double input, double thick)
{
    if (fabs(input) >= thick)
        return sign(input);
    else
        return input / thick;
}

int sign(double input)
{
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else
        return 0;
}
