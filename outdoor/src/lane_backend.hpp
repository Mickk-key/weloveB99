// 室外唯一巡线入口：原版灰度边缘算法及其实际拟合边界，计算保持原样。
#pragma once
#include "lane_original.hpp"
#include "lane_geometry.hpp"
struct LaneBackendConfig {void load(cv::FileStorage&) {}};
// Extract actual original fitted boundaries for obstacle/parking geometry. Lane driving
// continues to use the original TUxiang_Init3 error, including original edge fallback.
// 把原版 320×96 拟合掩码还原到 640×480 图像坐标，供绕桶和停车判断边界位置。
// 这份几何不替代原版巡线误差；原版的实际转向误差仍来自 TUxiang_Init3。
inline LaneV2 originalGeometry(){
    LaneV2 l;l.top=210;l.bottom=402;std::vector<cv::Point2f> left,right;
    const auto& mask=original_lane::lane_mask;
    int split=std::max(1,std::min(318,original_lane::mid_line));
    for(int y=0;y<mask.rows;y+=2){
        if(original_lane::lane_left_count)for(int x=split;x>=0;--x)if(mask.at<uchar>(y,x)){left.emplace_back(x*2,y*2+210);break;}
        if(original_lane::lane_right_count)for(int x=split;x<mask.cols;++x)if(mask.at<uchar>(y,x)){right.emplace_back(x*2,y*2+210);break;}
    }
    auto fit=[](const std::vector<cv::Point2f>& pts,double& a,double& b){
        if(pts.size()<8)return false;cv::Vec4f v;cv::fitLine(pts,v,cv::DIST_HUBER,0,.01,.01);
        if(std::abs(v[1])<.1)return false;a=v[0]/v[1];b=v[2]-a*v[3];return std::isfinite(a)&&std::isfinite(b);
    };
    l.leftValid=fit(left,l.la,l.lb);l.rightValid=fit(right,l.ra,l.rb);return l;
}
class LaneBackend {
public:
    static const char* name(){return "outdoor";}
    explicit LaneBackend(const LaneBackendConfig&) {}
    void detect(cv::Mat& frame,LaneV2& geometry,cv::Mat& mask,double& error,int& left,int& right){
        error=original_lane::TUxiang_Init3(frame);geometry=originalGeometry();
        mask=original_lane::lane_mask;left=original_lane::lane_left_count;right=original_lane::lane_right_count;
    }
};
