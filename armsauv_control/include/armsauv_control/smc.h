#ifndef SMC_H
#define SMC_H

#include "auvModel.h"

static double c_z,k_z,alpha_z,c_theta,k_theta,alpha_theta,c_psi,k_psi,alpha_psi,boundary_thick;

void paramInit(double _c_z, double _k_z, double _alpha_z, double _c_theta, double _k_theta, double _alpha_theta, double _c_psi, double _k_psi, double _alpha_psi, double _boundary_thick);
void courseSMCControl(const double ROB[], const double REF[], float *deltar);
void control(const double ROB[12], const double REF[9], float *deltar, float *deltab, float *deltas);
void controler_run(const double ROB[12],  const double REF[3], float* deltab, float* deltas, float* deltar, double dt);
double sat(double input, double thick);
int sign_smc(double input);

#endif
