#ifndef AUVMODEL_H
#define AUVMODEL_H

#include <stdio.h>
#define Pi 3.1415926
#define PI2 6.2831852

//艇体参数
#define m  84.71
#define LL  2.712
#define WW  831
#define BB  838 //重力和浮力,实际模型中B=838
#define x_B  0
#define y_B  0
#define z_B  0 // 浮心坐标
#define x_G  0
#define y_G  0
#define z_G  0.0086 //重心坐标
#define I_xx  0.82
#define I_yy  30.14
#define I_zz  30.14  //惯性矩
#define MAX_RUDDER_RAD 0.524  //舵角限制，弧度

//水动力参数
#define X_dotu  -1.432
#define Y_dotv  -120.645
#define Y_dotr  -4.895
#define Z_dotw  -130.513
#define Z_dotq  16.488
#define K_dotp  -0.386
#define M_dotw  16.488
#define M_dotq  -78.266
#define N_dotv  -4.895
#define N_dotr  -67.489
#define X_uu  -3.9
#define Y_vv  -373.287
#define Y_rr  -4.204
#define Z_ww  -489.07
#define Z_qq  23.016
#define K_pp  -0.1177
#define M_ww  23.342
#define M_qq  -353.406
#define N_vv  0.4193
#define N_rr  -227.024
#define Y_uv  -130.64
#define Y_ur  40.25
#define Z_uw  -522.87
#define Z_uq  4.27
#define M_uw  140.68
#define M_uq  73
#define N_uv -57.47
#define N_ur  -50.3

//控制力参数
#define Y_uudr  38.279
#define Z_uuds  -38.279
#define Z_uudb  -44.981
#define M_uuds  41.686
#define M_uudb  -44.531
#define N_uudr  -41.686

#endif
