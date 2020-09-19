/*
 * @Author: Zhao Wang
 * @Date: 2020-05-13 13:01:19
 * @LastEditTime: 2020-06-03 16:51:22
 * @LastEditors: Please set LastEditors
 * @Description: Implementation of interface of CLFLosController class
 * @FilePath: /armsauv_control/src/clf_los_controller.cpp
 */
#include <armsauv_control/clf_los_controller.h>
#include <iostream>

    std::pair<double, int> CLFLosController::computeCtrlQuantity(double x, double y, double tx, double ty, double yaw){
        std::cout << "[pid controller]: " << "x: " << x 
		                          << " y: " << y 
					  << " yaw: " << yaw 
					  << " target x: " << tx
					  << " target y: " << ty << std::endl;

        // det_pre_ = det_last_;
        // det_last_ = det_phi_; 
        
        double dist = distance(x, y, tx, ty);
        
        if(dist <= this->stop_tolerance_){
            return std::make_pair<double, int>(0.0, 1);
        }

        // Check if line is reverse
        if(line_.end_x_ < line_.start_x_)
        {
            line_.is_reverse_ = true;
        }
        else
        {
            line_.is_reverse_ = false;
        }

        std::cout << "[pid_controller]: " << "k: " << line_.k_ << " b: " << line_.b_ << " is_rev: " << line_.is_reverse_ << std::endl;
        // std::cout << "dy err: " << los_ctrl_param_.dy_err_ << std::endl;

         
        double target_y = line_.k_ * x + line_.b_;
        double line_dir = std::atan2(line_.end_y_ - line_.start_y_, line_.end_x_ - line_.start_x_);
 
        double ye = (y - target_y) * std::cos(line_dir);
        double ref_phi =  -std::atan(ye / factor_);

        if(line_.is_reverse_)
        {
            if(ref_phi < 0)
                ref_phi = -PI - ref_phi;
            else
                ref_phi = PI - ref_phi;
        }

        // det_phi_ = yaw - (line_dir - ref_phi);

        det_phi_ = ref_phi - (yaw - line_dir);

        if(det_phi_ > PI){
            det_phi_ = det_phi_ - 2 * PI;
        }
        if(det_phi_ < -PI)
        {
            det_phi_ = det_phi_ + 2 * PI;
        }
        
        std::cout << "ye: " << ye << " line_dir: " << line_dir << " ref_phi: " << ref_phi << " yaw: " << yaw << " det_phi: " << det_phi_ << std::endl;
        
        double r = this->los_ctrl_param_.kp_ * (det_phi_ - det_last_) + this->los_ctrl_param_.ki_ * det_phi_ +
            this->los_ctrl_param_.kd_ * (det_phi_ - 2 * det_last_ + det_pre_);
        if(fabs(r - output_last_) > 1.0){
	    r = output_last_;
	};

	det_pre_ = det_last_;
	det_last_ = det_phi_;

        return std::make_pair(r, 0);
    }

    void CLFLosController::resetLineInfo(const CLine& line){
	line_.start_x_ = line.start_x_;
	line_.start_y_ = line.start_y_;
	line_.end_x_ = line.end_x_;
	line_.end_y_ = line.end_y_;
	line_.k_ = line.k_;
	line_.b_ = line.b_;
	line_.is_reverse_ = line.is_reverse_;
    }
