#include "smc.h"
#include <stdio.h>
#include <math.h>

#include <cstddef>
#include <thread>
#include <mutex>
#include <unistd.h>

#include <ros/ros.h>

// ros components
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/FluidPressure.h>
#include <nav_msgs/Odometry.h>
#include <uuv_sensor_ros_plugins_msgs/DVL.h>

#include <uuv_gazebo_ros_plugins_msgs/FloatStamped.h>

#include <tf/transform_datatypes.h>

#include <geometry_msgs/Pose.h>

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

// For UWSim
ros::Publisher pose_pub;

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
controler_input* ctrl_input;
controler_output* ctrl_output;

const uint32_t ms = 1000;

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

/**
 * @brief 控制参数初始化
 * @param _c_z
 * @param _k_z
 * @param _alpha_z
 * @param _c_theta
 * @param _k_theta
 * @param _alpha_theta
 * @param _c_psi
 * @param _k_psi
 * @param _alpha_psi
 * @param _boundary_thick
 */
void paramInit(double _c_z, double _k_z, double _alpha_z, double _c_theta, double _k_theta,
               double _alpha_theta, double _c_psi, double _k_psi, double _alpha_psi, double _boundary_thick)
{
    c_z = _c_z;
    k_z = _k_z;
    alpha_z = _alpha_z;
    c_theta = _c_theta;
    k_theta = _k_theta;
    alpha_theta = _alpha_theta;
    c_psi = _c_psi;
    k_psi = _k_psi;
    alpha_psi = _alpha_psi;
    boundary_thick = _boundary_thick;
}

/**
 * @brief courseControl 滑模航向控制，右倾为正，艉倾为正，艏向角0-360
 * @param ROB[4] 输入当前艇体状态，ROB[4] ={psi,u,v,r} 都是国际标准单位
 * @param REF[1] 输入期望状态，REF[1] = {refpsi} 都是国际标准单位
 * @param deltar 方向舵舵角，右转为正，单位：弧度
 */
void courseControl(const double ROB[4], const double REF[1], double *deltar)
{
    double psi = ROB[0];
    double u = ROB[1];
    double v = ROB[2];
    double r = ROB[3];

    double refpsi = REF[0];

    //水平面艇体状态计算
    //横向运动方程变量替换(变量中的d为delta的缩写,p为psi的缩写)
    double a_yv = m-Y_dotv;
    double a_yr = m*x_G-Y_dotr;
    double a_ydr = Y_uudr*u*u;
    double f_y = m*y_G*r*r - m*u*r + X_dotu*u*r + Y_vv*v*fabs(v) + Y_uv*u*v + Y_rr*r*fabs(r) + Y_ur*u*r;

    //横摇运动方程变量替换
    double a_pv = m*x_G-N_dotv;
    double a_pr = I_zz-N_dotr;
    double a_pdr = N_uudr*u*u;
    double f_p = -m*x_G*u*r - m*y_G*v*r + (Y_dotv*v+Y_dotr*r)*u - X_dotu*u*v + N_vv*v*fabs(v) + N_uv*u*v + N_rr*r*fabs(r) + N_ur*u*r;
    double b_p = (a_yv*f_p-a_pv*f_y)/(a_yv*a_pr-a_pv*a_yr);
    double b_pdr = (a_yv*a_pdr-a_pv*a_ydr)/(a_yv*a_pr-a_pv*a_yr);

    double e_psi = psi - refpsi;
    if(e_psi >= PI)
        e_psi -= PI2;
    if(e_psi <= -PI)
        e_psi += PI2;
    double dot_e_psi = r;
    double S_psi = dot_e_psi + c_psi*e_psi;
    double L_psi = -r - b_p -k_psi*S_psi ;

    *deltar = -L_psi/b_pdr;
    if (fabs(*deltar) > MAX_RUDDER_RAD)
        *deltar = MAX_RUDDER_RAD * sign(*deltar);
}

/**
 * @brief control 滑模控制，右倾为正，艉倾为正，艏向角0-360
 * @param ROB[12] 输入当前艇体状态，ROB[12] = {x,y,z,phi,theta,psi,u,v,w,p,q,r} 都是国际标准单位
 * @param REF[9] 输入期望状态，REF[9] = {dz,dot_dz,dot2_dz,dtheta,dot_dtheta,dot2_dtheta,dpsi,dot_dpsi,dot2_dpsi} 都是国际标准单位
 * @param *deltar 期望方向舵角，上浮舵为正，单位：弧度
 * @param *deltab 期望艏舵角，上浮舵为正，单位：弧度
 * @param *deltas 期望艉舵角，右转为正，单位：弧度
 */
void control(const double ROB[12], const double REF[9], double *deltar, double
             *deltab, double *deltas)
{
  double a_zw;
  double a_zq;
  double a_zs;
  double a_zb;
  double f_z;
  double a_tw;
  double a_tq;
  double a_ts;
  double a_tb;
  double f_t;
  double b_tb;
  double b_ts;
  double g_zb;
  double g_zs;
  double a_yv;
  double a_yr;
  double a_pv;
  double a_pr;
  double dot_e_z;
  double S_z;
  double dot_e_theta;
  double S_theta;
  double dot_e_psi;
  double S_psi;
  double output;
  double L_z;
  double L_theta;
  double b_deltab;
  double b_deltas;
  double b_deltar;

  /* %水动力参数 */
  /* %控制力参数 */
  /* 控制参数 */
  /* % 艇体状态信息提取 */
  /* % 期望信息提取 */
  /* % 深度面艇体状态计算 */
  /* 垂向运动方程变量替换 */
  a_zw = m - Z_dotw;
  a_zq = -(m * x_G + Z_dotq);
  a_zs = Z_uuds * ROB[6] * ROB[6];
  a_zb = Z_uudb * ROB[6] * ROB[6];
  f_z = ((((((m * ROB[6] * ROB[10] + m * z_G * ROB[10] * ROB[10]) - X_dotu * ROB[6] * ROB[10]) + Z_ww * ROB[8] * fabs(ROB[8])) + Z_uw * ROB[6] *ROB[8]) + Z_qq * ROB[10] * fabs(ROB[10])) + Z_uq * ROB[6] * ROB[10])+ (W - B) * cos(ROB[4]);

  /* 纵倾运动方程变量替换(变量中的t为theta的缩写) */
  a_tw = -(m * x_G + M_dotw);
  a_tq = I_yy - M_dotq;
  a_ts = M_uuds * ROB[6] * ROB[6];
  a_tb = M_uudb * ROB[6] * ROB[6];
  f_t = ((((((((-m * z_G * ROB[8] * ROB[10] - m * x_G * ROB[6] * ROB[10]) -
               (Z_dotw * ROB[8] + Z_dotq * ROB[10]) * ROB[6]) + X_dotu * ROB[6] *
              ROB[8]) + M_ww * ROB[8] * fabs(ROB[8])) + M_uw * ROB[6] * ROB[8])
           + M_qq * ROB[10] * fabs(ROB[10])) + M_uq * ROB[6] * ROB[10]) - (z_G *
          W - z_B * B) * sin(ROB[4])) - (x_G * W - x_B * B) * cos(ROB[4]);

  /* dot_w和dot_q表达式中的量 */
  b_tb = (a_zw * a_tb - a_tw * a_zb) / (a_zw * a_tq - a_zq * a_tw);
  b_ts = (a_zw * a_ts - a_tw * a_zs) / (a_zw * a_tq - a_zq * a_tw);

  /* dot2_z和dot2_theta表达式中的量 */
  g_zb = (a_tq * a_zb - a_zq * a_tb) / (a_zw * a_tq - a_zq * a_tw) * cos(ROB[4]);
  g_zs = (a_tq * a_zs - a_zq * a_ts) / (a_zw * a_tq - a_zq * a_tw) * cos(ROB[4]);

  /* 深度及纵倾的一阶导 */
  /* % 水平面艇体状态计算 */
  /* 横向运动方程变量替换(变量中的d为delta的缩写,p为psi的缩写) */
  a_yv = m - Y_dotv;
  a_yr = m * x_G - Y_dotr;
  double f_y = ((((((m *
                     y_G * ROB[11] * ROB[11] - m * ROB[6] * ROB[11]) + X_dotu * ROB[6] * ROB[11])
                     + Y_vv * ROB[7] * fabs(ROB[7])) + Y_uv * ROB[6] * ROB[7]) + Y_rr * ROB[11] *
                     fabs(ROB[11])) + Y_ur * ROB[6] * ROB[11]);

  /* 横摇运动方程变量替换 */
  a_pv = m * x_G - N_dotv;
  a_pr = I_zz - N_dotr;
  double f_p = (((((((-m * x_G * ROB[6] * ROB[11] - m * y_G *
                    ROB[7] * ROB[11]) + (Y_dotv * ROB[7] + Y_dotr * ROB[11]) * ROB[6]) - X_dotu *
                    ROB[6] * ROB[7]) + N_vv * ROB[7] * fabs(ROB[7])) + N_uv * ROB[6] * ROB[7]) +
                    N_rr * ROB[11] * fabs(ROB[11])) + N_ur * ROB[6] * ROB[11]);

  /* dot_v和dot_r表达式中的量 */

  double b_p = (a_yv * f_p - a_pv * f_y) / (a_yv * a_pr - a_pv * a_yr);

  /* 航向psi的一阶导 */
  /* % 滑模控制 */
  /* 深度 */
  dot_e_z = (-ROB[6] * sin(ROB[4]) + ROB[8] * cos(ROB[4])) - REF[1];
  S_z = dot_e_z + c_z * (ROB[2] - REF[0]);

  /* 纵倾 */
  dot_e_theta = ROB[10] - REF[4];
  S_theta = dot_e_theta + c_theta * (ROB[4] - REF[3]);

  /* 航向 */
  double e_psi = ROB[5] - REF[6];
  if(e_psi >= PI)
      e_psi -= PI2;
  if(e_psi <= -PI)
      e_psi += PI2;
  dot_e_psi = ROB[11] - REF[7];
  S_psi = dot_e_psi + c_psi * (e_psi);

  /*    此处显示详细说明 */
  if (fabs(S_z) >= boundary_thick) {
    if (S_z < 0.0) {
      output = -1.0;
    } else if (S_z > 0.0) {
      output = 1.0;
    } else if (S_z == 0.0) {
      output = 0.0;
    } else {
      output = S_z;
    }
  } else {
    output = S_z / boundary_thick;
  }

  L_z = ((REF[2] - (((a_tq * f_z - a_zq * f_t) / (a_zw * a_tq - a_zq * a_tw) *
                     cos(ROB[4]) - ROB[6] * ROB[10] * cos(ROB[4])) - ROB[8] *
                    ROB[10] * sin(ROB[4]))) - c_z * dot_e_z) - k_z * pow
    (fabs(S_z), alpha_z) * output;

  /*    此处显示详细说明 */
  if (fabs(S_theta) >= boundary_thick) {
    if (S_theta < 0.0) {
      output = -1.0;
    } else if (S_theta > 0.0) {
      output = 1.0;
    } else if (S_theta == 0.0) {
      output = 0.0;
    } else {
      output = S_theta;
    }
  } else {
    output = S_theta / boundary_thick;
  }

  L_theta = ((REF[5] - (a_zw * f_t - a_tw * f_z) / (a_zw * a_tq - a_zq * a_tw))
             - c_theta * dot_e_theta) - k_theta * pow(fabs(S_theta),
    alpha_theta) * output;

  /*    此处显示详细说明 */
  if (fabs(S_psi) >= boundary_thick) {
    if (S_psi < 0.0) {
      output = -1.0;
    } else if (S_psi > 0.0) {
      output = 1.0;
    } else if (S_psi == 0.0) {
      output = 0.0;
    } else {
      output = S_psi;
    }
  } else {
    output = S_psi / boundary_thick;
  }

  /* 指令计算 */
  *deltab = -(L_z * b_ts - L_theta * g_zs) / (g_zb * b_ts - b_tb * g_zs);
  *deltas = -(L_theta * g_zb - L_z * b_tb) / (g_zb * b_ts - b_tb * g_zs);
  double b_pdr = ((a_yv * (N_uudr * ROB[6] * ROB[6]) - a_pv * (Y_uudr *
                                                           ROB[6] * ROB[6])) / (a_yv * a_pr - a_pv * a_yr));

  double L_psi = ((REF[8] - b_p - c_psi * dot_e_psi) - k_psi * pow(fabs(S_psi), alpha_psi) *output);
  //printf("L_psi=%f b_p=%f\n",L_psi,b_p);
  *deltar = -L_psi / b_pdr;

  if (fabs(*deltab) > MAX_RUDDER_RAD) {
    if (*deltab < 0.0) {
      b_deltab = -1.0;
    } else if (*deltab > 0.0) {
      b_deltab = 1.0;
    } else if (*deltab == 0.0) {
      b_deltab = 0.0;
    } else {
      b_deltab = *deltab;
    }

    *deltab = MAX_RUDDER_RAD * b_deltab;
  }

  if (fabs(*deltas) > MAX_RUDDER_RAD) {
    if (*deltas < 0.0) {
      b_deltas = -1.0;
    } else if (*deltas > 0.0) {
      b_deltas = 1.0;
    } else if (*deltas == 0.0) {
      b_deltas = 0.0;
    } else {
      b_deltas = *deltas;
    }

    *deltas = MAX_RUDDER_RAD * b_deltas;
  }

  if (fabs(*deltar) > MAX_RUDDER_RAD) {
    if (*deltar < 0.0) {
      b_deltar = -1.0;
    } else if (*deltar > 0.0) {
      b_deltar = 1.0;
    } else if (*deltar == 0.0) {
      b_deltar = 0.0;
    } else {
      b_deltar = *deltar;
    }

    *deltar = MAX_RUDDER_RAD * b_deltar;
  }
}

/**
 * @brief controler_run 滑模控制，右倾为正，艉倾为正，艏向角0-360
 * @param ROB[12] 输入当前艇体状态，ROB[12] = {x,y,z,phi,theta,psi,u,v,w,p,q,r} 都是国际标准单位
 * @param REF[3] 输入期望状态，REF[3] = {dz,dtheta,dpsi} 都是国际标准单位
 * @param deltab 期望艏舵角，上浮舵为正，单位：弧度，
 * @param deltas 期望艉舵角，上浮舵为正，单位：弧度
 * @param deltar 期望方向舵舵角，右转为正，单位：弧度
 * @param dt 调用时间间隔，用于计算期望状态的导数和二阶导
 */
void controler_run(const double ROB[12],  const double REF[3], double* deltab, double* deltas, double* deltar, double dt)
{
    std::cout << "[armsauv_control]:Start control" << std::endl;

    //艇体位姿和速度变量
    static double x, y, z, phi, theta, psi, u, v, w, p, q, r;

    //控制任务信息变量
    static double REFz, REFdot_z, REFdot2_z;             //期望深度及其一阶和二阶导数
    static double preREFz, preREFdot_z;                  //上一控制周期的期望深度及其一阶导

    static double REFtheta, REFdot_theta, REFdot2_theta; //期望纵倾及其一阶和二阶导数
    static double preREFtheta, preREFdot_theta;          //上一控制周期的期望纵倾及其一阶导

    static double REFpsi, REFdot_psi, REFdot2_psi;       //期望航向及其一阶和二阶导数
    static double preREFpsi=3.14/3, preREFdot_psi;              //上一控制周期的期望航行及其一阶导

    std::cout << "[armsauv_control]:ROB:" 
          << "{x:" << ROB[0] << " y:" << ROB[1] << " z:" << ROB[2] 
          << " phi:" << ROB[3] << " theta:" << ROB[4] << " psi:" << ROB[5]
          << " u:" << ROB[6] << " v:" << ROB[7] << " w:" << ROB[8] 
          << " p:" << ROB[9] << " q:" << ROB[10] << " r:" << ROB[11] << "}" << std::endl;
  
    //艇体位姿和速度信息提取
    x = ROB[0];
    y = ROB[1];
    z = ROB[2];
    phi = ROB[3];
    theta = ROB[4];
    psi = ROB[5];
    u = ROB[6];
    v = ROB[7];
    w = ROB[8];
    p = ROB[9];
    q = ROB[10];
    r = ROB[11];


    //控制任务信息提取和计算
    REFz = REF[0];    //深度信息
    REFdot_z = 0;           //深度一阶导
    REFdot2_z = 0;          //深度二阶导
    preREFz = REFz;         //更新上一周期的深度值
    preREFdot_z = REFdot_z; //更新上一周期深度一阶导

    REFtheta = REF[1]; //期望的纵倾加上纵倾制导量
    REFdot_theta = (REFtheta - preREFtheta) / dt;
    REFdot2_theta = (REFdot_theta - preREFdot_theta) / dt;
    preREFtheta = REFtheta;
    preREFdot_theta = REFdot_theta;

    REFpsi = REF[2];
    REFdot_psi = (REFpsi - preREFpsi) / dt;
    REFdot2_psi = (REFdot_psi - preREFdot_psi) / dt;
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
    if(e_psi >= PI)
        e_psi -= PI2;
    if(e_psi <= -PI)
        e_psi += PI2;
    dot_e_psi = dot_psi - REFdot_psi;
    S_psi = dot_e_psi + c_psi * e_psi;
    //printf("%f\n",dot_e_psi);

    //指令计算公式中的中间量
    L_z = REFdot2_z - g_z - c_z * dot_e_z - k_z * pow(fabs(S_z), alpha_z) * sat(S_z, boundary_thick);
    L_theta = REFdot2_theta - g_t - c_theta * dot_e_theta - k_theta * pow(fabs(S_theta), alpha_theta) * sat(S_theta, boundary_thick);
    L_psi = REFdot2_psi - b_p - c_psi * dot_e_psi - k_psi * pow(fabs(S_psi), alpha_psi) * sat(S_psi, boundary_thick);
    //指令计算
    //printf("L_psi=%f b_p=%f\n",L_psi,b_p);
    *deltab = -(L_z * g_ts - L_theta * g_zs) / (g_zb * g_ts - g_tb * g_zs);
    *deltas = -(L_theta * g_zb - L_z * g_tb) / (g_zb * g_ts - g_tb * g_zs);
    *deltar = -L_psi / b_pdr;

    // *deltab = -(L_z * g_ts - L_theta * g_zs) / (g_zb * g_ts - g_tb * g_zs);
    // *deltas = -(L_theta * g_zb - L_z * g_tb) / (g_zb * g_ts - g_tb * g_zs);
    // *deltar =  L_psi / b_pdr;
    
    if (fabs(*deltab) > MAX_RUDDER_RAD)
        *deltab = MAX_RUDDER_RAD * sign(*deltab);

    if (fabs(*deltas) > MAX_RUDDER_RAD)
        *deltas = MAX_RUDDER_RAD * sign(*deltas);

    if (fabs(*deltar) > MAX_RUDDER_RAD)
        *deltar = MAX_RUDDER_RAD * sign(*deltar);
}

double sat(double input, double thick)
{
    if (fabs(input) >= thick)
        return sign(input);
    else
        return input / thick;
}

/**
 * @brief sign 符号函数，判断输入参数的符号
 * @param input 输入参数
 * @return -1,0,1 三者之一
 */
int sign(double input)
{
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else
        return 0;
}

/**
 * @brief Apply the controller output
 * @rouder Rouder value
 * @fwd_fin heading fin value
 * @aft_fin stern fin value
 * @rpm Rotate velocity of propeller 
 */
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


/**
 * @brief Timer callback function, execute the controller
 * @event ROS timer event variable
 */
void timer_cb(const ros::TimerEvent& event){
    if(getX() == 0 || getXspeed() == 0){
        return;
    }

    std::cout << "[armsauv_control]:Update sensor messages" << std::endl;
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

    /* reserved for input */
    double auv[12];
    auv[0] = sensors->x;
    auv[1] = sensors->y;
    auv[2] = sensors->z;
    auv[3] = sensors->roll;
    auv[4] = sensors->pitch;
    auv[5] = sensors->yaw;
    auv[6] = sensors->x_speed;
    auv[7] = sensors->y_speed;
    auv[8] = sensors->z_speed;
    auv[9] = sensors->roll_speed;
    auv[10] = sensors->pitch_speed;
    auv[11] = sensors->yaw_speed;

    std::cout << "[aramsauv_control]:auv: "
	      << auv[0] << " " << auv[1] << " " << auv[2] 
	      << " " << auv[3] << " " << auv[4] << " " << auv[5] 
	      << " " << auv[6] << " " << auv[7] << " " << auv[8]
	      << " " << auv[9] << " " << auv[10] << " " << auv[11] << std::endl;   

    /*
    std::cout << "[armsauv_control]:Set mission messages" << std::endl;
    std::cout << "x" << std::endl;
    ctrl_input->x_d = 0.0;
    std::cout << "y" << std::endl;
    ctrl_input->y_d = 0.0;
    std::cout << "depth" << std::endl;
    ctrl_input->depth = 10.0;
    std::cout << "pitch" << std::endl;
    ctrl_input->pitch = 0.0;
    std::cout << "yaw" << std::endl;
    ctrl_input->yaw = 0.0;
    */

    std::cout << "[armsauv_control]:Set reference" << std::endl;
    double ref_params[3];
    ref_params[0] = 0.0; // dz
    ref_params[1] = 30.0; // dtheta
    ref_params[2] = 0.0; // dpsi
    std::cout << "[armsauv_control]:ref: "
	      << ref_params[0] << " " << ref_params[1] << " " << ref_params[2] << std::endl;

    std::cout << "[armsauv_control]:Run controller" << std::endl;
    

    double deltar, deltab, deltas;
    // Controller 
    controler_run(auv, ref_params, &deltab, &deltas, &deltar, 0.1);
    std::cout << "[armsauv_control]:Apply the controller output" << std::endl;

    // ctrl_output->rouder = deltar;
    // ctrl_output->fwd_fin = deltab;
    // ctrl_output->aft_fin = deltas;

    std::cout << "[armsuav_control]:Controller output:"
	      << "{fwd fin:" << deltab
	      << " aft fin:" << deltas
	      << " rouder:" << deltar << "}" << std::endl;

    // Publish thruster and fin message    
    applyActuatorInput(deltar, deltab, deltas, ms);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "armsuav_smc");

    node = new ros::NodeHandle;

    /* sensors */
    ros::Subscriber sub_imu = node->subscribe("/armsauv/imu", 1, imu_cb);
    ros::Subscriber sub_pressure = node->subscribe("/armsauv/pressure", 1, pressure_cb);
    ros::Subscriber sub_posegt = node->subscribe("/armsauv/pose_gt", 1, posegt_cb);
    ros::Subscriber sub_dvl = node->subscribe("/armsauv/dvl", 1, dvl_cb);
    
    /* actuators */
    thruster0_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/thrusters/0/input", 1);
    fin0_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/0/input", 1);
    fin1_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/1/input", 1);
    fin2_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/2/input", 1);
    fin3_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/3/input", 1);
    fin4_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/4/input", 1);
    fin5_pub = node->advertise<uuv_gazebo_ros_plugins_msgs::FloatStamped>("/armsauv/fins/5/input", 1);
    pose_pub = node->advertise<geometry_msgs::Pose>("/armsauv/pose", 1);  
    sensors = new sensors_data;
    
    ros::Timer timer = node->createTimer(ros::Duration(0.1), timer_cb);

    ros::spin(); 

    delete sensors;
    delete ctrl_input;
    delete ctrl_output;

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

    // Publish pose
    geometry_msgs::Pose armsauv_pose;
    armsauv_pose.position.x = msg->pose.pose.position.x;
    armsauv_pose.position.y = -msg->pose.pose.position.y;
    armsauv_pose.position.z = -msg->pose.pose.position.z;

    double roll, pitch, yaw;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);

    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    pitch = -pitch;
    yaw = -yaw;

    armsauv_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    pose_pub.publish(armsauv_pose);

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

