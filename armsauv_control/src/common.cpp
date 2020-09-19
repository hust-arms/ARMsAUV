/*
 * @Author: Zhao Wang
 * @Date: 2020-05-07 
 * @LastEditTime: 2020-05-10 
 * @LastEditors: Zhao Wang
 * @Description: Implementation of common methods for los nav objects
 * @FilePath: /armsauv_control/src/pf_los_controller.cpp
 */
#include <armsauv_control/common.h>

    double distance(double x1, double y1, double x2, double y2){
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    };

    double radToAngle(double rad){
        return rad / PI * 180;  
    };

    double slopeToAngle(double slope){
        return std::atan(slope) / PI * 180;
    };

    double slopeToAngle(double det_y, double det_x){
        return std::atan2(det_y, det_x) / PI * 180;
    };
