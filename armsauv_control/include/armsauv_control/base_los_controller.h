/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 13:08:08
 * @LastEditTime: 2020-05-23 21:20:04
 * @LastEditors: Please set LastEditors
 * @Description: Definition of BaseLosController class, the fundamental class of all los controller
 * @FilePath: /armsauv_control/include/armsauv_control/base_los_controller.h
 */
#ifndef BASE_LOS_CONTROLLER_H_
#define BASE_LOS_CONTROLLER_H_

#include <utility>
#include <armsauv_control/common.h>

struct LosCtrlParam{
    LosCtrlParam(double kp = 0.0, double ki = 0.0, double kd = 0.0,
        double dx_err = 0.0, double dy_err = 0.0):kp_(kp), ki_(ki), kd_(kd), dx_err_(dx_err), dy_err_(dy_err){}
    LosCtrlParam(const LosCtrlParam& pid_param){
        kp_ = pid_param.kp_;
        ki_ = pid_param.ki_;
        kd_ = pid_param.kd_;
        dx_err_ = pid_param.dx_err_;
        dy_err_ = pid_param.dy_err_;
    }
    double kp_, ki_, kd_;
    double dx_err_, dy_err_;
};

/**
 * @brief LOS implementation
 */
class BaseLosController{
public:
    BaseLosController(LosCtrlParam los_ctrl_param, double stop_tolerance) : 
        los_ctrl_param_(los_ctrl_param), stop_tolerance_(stop_tolerance) {}
    virtual ~BaseLosController(){};
  
    /**
     * @brief Compute control quantity for control mission
     * @param double x Coordination x of robot
     * @param double y Coordination y of robot
     * @param double tx Coordination x of target point
     * @param double ty Coordination y of target point
     * @param double yaw The euler angle yaw of robot
     * @return Compute result
     */ 
    virtual std::pair<double, int> computeCtrlQuantity(double x, double y, double tx, double ty, double yaw) = 0;

protected:    
    LosCtrlParam los_ctrl_param_;
    double ctrl_quat_;
    double stop_tolerance_;
}; // end of class

#endif
