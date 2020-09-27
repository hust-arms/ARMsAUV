/*
 * @Author: Zhao Wang
 * @Date: 2020-05-13 11:13:33
 * @LastEditTime: 2020-06-04 15:55:12
 * @LastEditors: Please set LastEditors
 * @Description: Definition of CLFLosController class
 * @FilePath: /armsauv_control/include/armsauv_control/slf_los_controller.h
 */
#ifndef CLF_LOS_CONTROLLER_H_
#define CLF_LOS_CONTROLLER_H_

#include <armsauv_control/base_los_controller.h>
#include <iostream>

/**
 * @brief Common line follow LOS implementation
 */
class CLFLosController : public BaseLosController{
public:
    /**
     * @brief Constructor
     * @param LosCtrlParam pid_param PID parameters for LOS controller
     * @param Point Coordinate of target point
     */ 
    CLFLosController(LosCtrlParam pid_param, const CLine& line, double los_factor = 20.0, double stop_tolerance = 15.0) : 
        BaseLosController(pid_param, stop_tolerance), line_(line), factor_(los_factor){ std::cout << "In CLine Controller: " << factor_ << std::endl; det_phi_ = 0.0; det_pre_ = 0.0, det_last_ = 0.0;}

    /**
     * @brief Compute control quantity for point follow
     * @param double x Coordination x of robot
     * @param double y Coordination y of robot
     * @param double tx Coordination x of target point
     * @param double ty Coordination y of target point
     * @param double yaw The euler angle yaw of robot
     * @return Compute result
     */ 
    std::pair<double, int> computeCtrlQuantity(double x, double y, double tx, double ty, double yaw)override;

    /**
     * @brief Return Ye
     * @return Ye
     */ 
    double getYe()const{ return ye_; }

    /**
     * @brief Reset parameters of desired line
     * @param line Info of desired line
     */ 
    void resetLineInfo(const CLine& line);

private:
    CLine line_;
    double factor_;
    double ye_;
    double det_phi_;
    double det_sum_;
    // double det_phi_diff_, det_phi_int_;
    double det_pre_, det_last_; 
    double output_last_;
}; // end of class

#endif
