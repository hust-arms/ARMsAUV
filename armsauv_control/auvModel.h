#ifndef AUVMODEL_H
#define AUVMODEL_H


const double PI = 3.1415926;
const double PI2 = 6.2831852;

//艇体参数
const double m = 84.71;
const double L = 2.712;
const double W = 831;
const double B = 838; //重力和浮力
const double x_B = 0;
const double y_B = 0;
const double z_B = 0; // 浮心坐标
const double x_G = 0;
const double y_G = 0;
const double z_G = 0.0086; //重心坐标
const double I_xx = 0.82;
const double I_yy = 30.14;
const double I_zz = 30.14;  //惯性矩
const double MAX_RUDDER_RAD = 0.436;  //舵角限制，弧度

//水动力参数
const double X_dotu = -1.432;
const double Y_dotv = -120.645;
const double Y_dotr = -4.895;
const double Z_dotw = -130.513;
const double Z_dotq = 16.488;
const double K_dotp = -0.386;
const double M_dotw = 16.488;
const double M_dotq = -78.266;
const double N_dotv = -4.895;
const double N_dotr = -67.489;
const double X_uu = -3.9;
const double Y_vv = -373.287;
const double Y_rr = -4.204;
const double Z_ww = -489.07;
const double Z_qq = 23.016;
const double K_pp = -0.1177;
const double M_ww = 23.342;
const double M_qq = -353.406;
const double N_vv = 0.4193;
const double N_rr = -227.024;
const double Y_uv = -130.64;
const double Y_ur = 40.25;
const double Z_uw = -522.87;
const double Z_uq = 4.27;
const double M_uw = 140.68;
const double M_uq = 73;
const double N_uv = -57.47;
const double N_ur = -50.3;

//控制力参数
const double Y_uudr = 38.279;
const double Z_uuds = -38.279;
const double Z_uudb = -44.981;
const double M_uuds = 41.686;
const double M_uudb = -44.531;
const double N_uudr = -41.686;

#endif
