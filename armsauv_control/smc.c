﻿#include "../incl/smc.h"
#include <stdio.h>
#include <math.h>

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
    
    printf("c = %f, k = %f \n",c_psi, k_psi);
	
    boundary_thick = _boundary_thick;
}

/**
 * @brief courseControl 滑模航向控制，右倾为正，艉倾为正，艏向角0-2PI
 * @param ROB[4] 输入当前艇体状态，ROB[4] ={psi,u,v,r} 都是国际标准单位
 * @param REF[1] 输入期望状态，REF[2] = {refpsi,dot_refpsi} 都是国际标准单位
 * @param deltar 方向舵舵角，右转为正，单位：弧度
 */
void courseSMCControl(const double ROB[4], const double REF[2], float *deltar)
{
    double psi = ROB[0];
    double u = ROB[1];
    double v = ROB[2];
    double r = ROB[3];

    double refpsi = REF[0];
	double dot_refpsi = REF[1];

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
    if(e_psi >= Pi)
        e_psi -= PI2;
    if(e_psi <= -Pi)
        e_psi += PI2;
    double dot_e_psi = r - dot_refpsi;
    double S_psi = dot_e_psi + c_psi*e_psi;
    double L_psi = -c_psi*r - b_p -k_psi*S_psi ;

    *deltar = -L_psi/b_pdr;
    printf("c = %f, k = %f \n",c_psi, k_psi);
    printf("e_psi = %f, dot_e_psi = %f, S_psi = %f, L_psi = %f, b_pdr = %f, deltar = %f \n",e_psi,dot_e_psi,S_psi,L_psi,b_pdr,(*deltar)*57.3);
    if (fabs(*deltar) > MAX_RUDDER_RAD)
        *deltar = MAX_RUDDER_RAD * sign_smc(*deltar);
        
}

/**
 * @brief control 滑模控制，右倾为正，艉倾为正，艏向角0-360
 * @param ROB[12] 输入当前艇体状态，ROB[12] = {x,y,z,phi,theta,psi,u,v,w,p,q,r} 都是国际标准单位
 * @param REF[9] 输入期望状态，REF[9] = {dz,dot_dz,dot2_dz,dtheta,dot_dtheta,dot2_dtheta,dpsi,dot_dpsi,dot2_dpsi} 都是国际标准单位
 * @param *deltar 期望方向舵角，上浮舵为正，单位：弧度
 * @param *deltab 期望艏舵角，上浮舵为正，单位：弧度
 * @param *deltas 期望艉舵角，右转为正，单位：弧度
 */
void control(const double ROB[12], const double REF[9], float *deltar, float
             *deltab, float *deltas)
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
  f_z = ((((((m * ROB[6] * ROB[10] + m * z_G * ROB[10] * ROB[10]) - X_dotu * ROB[6] * ROB[10]) + Z_ww * ROB[8] * fabs(ROB[8])) + Z_uw * ROB[6] *ROB[8]) + Z_qq * ROB[10] * fabs(ROB[10])) + Z_uq * ROB[6] * ROB[10])+ (WW - BB) * cos(ROB[4]);

  /* 纵倾运动方程变量替换(变量中的t为theta的缩写) */
  a_tw = -(m * x_G + M_dotw);
  a_tq = I_yy - M_dotq;
  a_ts = M_uuds * ROB[6] * ROB[6];
  a_tb = M_uudb * ROB[6] * ROB[6];
  f_t = ((((((((-m * z_G * ROB[8] * ROB[10] - m * x_G * ROB[6] * ROB[10]) -
               (Z_dotw * ROB[8] + Z_dotq * ROB[10]) * ROB[6]) + X_dotu * ROB[6] *
              ROB[8]) + M_ww * ROB[8] * fabs(ROB[8])) + M_uw * ROB[6] * ROB[8])
           + M_qq * ROB[10] * fabs(ROB[10])) + M_uq * ROB[6] * ROB[10]) - (z_G *
          WW - z_B * BB) * sin(ROB[4])) - (x_G * WW - x_B * BB) * cos(ROB[4]);

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
  if(e_psi >= Pi)
      e_psi -= PI2;
  if(e_psi <= -Pi)
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
void controler_run(const double ROB[12],  const double REF[3], float* deltab, float* deltas, float* deltar, double dt)
{
    //艇体位姿和速度变量
    static double x, y, z, phi, theta, psi, u, v, w, p, q, r;

    //控制任务信息变量
    static double REFz, REFdot_z, REFdot2_z;             //期望深度及其一阶和二阶导数
    static double preREFz, preREFdot_z;                  //上一控制周期的期望深度及其一阶导

    static double REFtheta, REFdot_theta, REFdot2_theta; //期望纵倾及其一阶和二阶导数
    static double preREFtheta, preREFdot_theta;          //上一控制周期的期望纵倾及其一阶导

    static double REFpsi, REFdot_psi, REFdot2_psi;       //期望航向及其一阶和二阶导数
    static double preREFpsi=3.14/3, preREFdot_psi;              //上一控制周期的期望航行及其一阶导

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
    f_z = m * u * q + m * z_G * q * q - X_dotu * u * q + Z_ww * w * fabs(w) + Z_uw * u * w + Z_qq * q * fabs(q) + Z_uq * u * q + (WW - BB) * cos(theta);
    //纵倾运动方程变量替换(变量中的t为theta的缩写)
    a_tw = -(m * x_G + M_dotw);
    a_tq = I_yy - M_dotq;
    a_ts = M_uuds * u * u;
    a_tb = M_uudb * u * u;
    f_t = -m * z_G * w * q - m * x_G * u * q - (Z_dotw * w + Z_dotq * q) * u + X_dotu * u * w + M_ww * w * fabs(w) + M_uw * u * w + M_qq * q * fabs(q) + M_uq * u * q - (z_G * WW - z_B * BB) * sin(theta) - (x_G * WW - x_B * BB) * cos(theta);
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
    if(e_psi >= Pi)
        e_psi -= PI2;
    if(e_psi <= -Pi)
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

    if (fabs(*deltab) > MAX_RUDDER_RAD)
        *deltab = MAX_RUDDER_RAD * sign_smc(*deltab);

    if (fabs(*deltas) > MAX_RUDDER_RAD)
        *deltas = MAX_RUDDER_RAD * sign_smc(*deltas);

    if (fabs(*deltar) > MAX_RUDDER_RAD)
        *deltar = MAX_RUDDER_RAD * sign_smc(*deltar);
}

double sat(double input, double thick)
{
    if (fabs(input) >= thick)
        return sign_smc(input)*thick;
    else
        return input / thick;
}

/**
 * @brief sign_smc 符号函数，判断输入参数的符号
 * @param input 输入参数
 * @return -1,0,1 三者之一
 */
int sign_smc(double input)
{
    if (input > 0)
        return 1;
    else if (input < 0)
        return -1;
    else
        return 0;
}
