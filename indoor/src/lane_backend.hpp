// 室内唯一巡线入口：沿用原HSV白线/霍夫拟合，其他模块与室外一致。
#pragma once
#include "lane_v2.hpp"
using LaneBackendConfig=LaneConfig;
class LaneBackend {
    LaneDetectorV2 detector;
public:
    static const char* name(){return "indoor";}
    explicit LaneBackend(const LaneBackendConfig& c):detector(c){}
    void detect(cv::Mat& frame,LaneV2& geometry,cv::Mat& mask,double& error,int& left,int& right){
        geometry=detector.detect(frame);error=geometry.error;mask=detector.mask;
        left=geometry.leftValid;right=geometry.rightValid;
    }
};
