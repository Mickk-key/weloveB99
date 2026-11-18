// 两环境统一：原室外形状候选 → 视觉临时边界 → 两桶通过并回中。
// 候选筛选、跟踪和误差增量沿用室外算法；车道边界由各自巡线入口提供。
#pragma once
#include "lane_geometry.hpp"
#include "cone_config.hpp"
#include "cone_shape.hpp"
#include "cone_lane_visual.hpp"
class ConeAvoidV2 {
    struct Candidate {cv::Rect box;int pixels;double bottom()const{return box.y+box.height;}};
    ConeConfig cfg;ConeLaneVisual visual;
    bool timeReady=false;double lastTime=0;
    std::vector<Candidate> candidates;
    static double bottom(const cv::Rect& b) {return b.y+b.height;}
    // 保守通道模式只认可两侧真实拟合；先要求 ROI 某处可见有序双线，再外推到锥桶底边。
    // 不把 LaneV2 的画面边缘补位当成已确认的道路边界，也不把出画面的近端线裁窄。
    bool bounds(const LaneV2& lane,double y,double& l,double& r) const {
        if(!lane.leftValid || !lane.rightValid) return false;
        // Both REAL fits must be visible at a reference row. Near the vehicle,
        // a genuine boundary may extend outside the image; never clip it into a
        // falsely narrower corridor. This does not certify image-edge fallbacks.
        // Different camera poses can place the horizon inside the original
        // outdoor ROI. Require a visible, ordered pair somewhere IN the ROI,
        // rather than assuming top+30 is always below the horizon.
        bool visiblePair=false;
        for(int row=lane.top;row<lane.bottom;row+=5){
            double a=lane.la*row+lane.lb,b=lane.ra*row+lane.rb;
            if(std::isfinite(a)&&std::isfinite(b)&&a>=0&&b<=639&&b-a>=30){visiblePair=true;break;}
        }
        l=lane.la*y+lane.lb;r=lane.ra*y+lane.rb;
        return visiblePair&&std::isfinite(l)&&std::isfinite(r)&&r-l>=30&&
               l>=-cfg.cone_lane_extrapolation_px&&r<=639+cfg.cone_lane_extrapolation_px;
    }
    void detect(const cv::Mat& frame,const LaneV2& lane) {
        candidates.clear(); mask=cv::Mat::zeros(frame.size(),CV_8UC1);
        { // 唯一形状检测路径，计算保持原样。
            ConeShapeDetector shape;
            for(const auto& b:shape.detect(frame,cfg,mask)){
                double l,r;if(bounds(lane,bottom(b),l,r)&&(b.br().x<l||b.x>r))continue;
                candidates.push_back({b,b.area()});
            }
            std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){
                return a.bottom()!=b.bottom()?a.bottom()>b.bottom():a.pixels>b.pixels;
            });
            return;
        }
    }
public:
    bool laneOverride()const {return visual.controlling();}
    double laneErrorDelta()const {return visual.errorDelta;}
    bool laneCompleted()const {return visual.completed();}
    void laneFeedback(bool moving){visual.feedback(moving);}
    const ConeLaneVisual& laneVisual()const {return visual;}

    bool near=false;
    cv::Mat mask;std::string phase="SEARCH";
    double offsetFraction=0;int passedCount=0;
    explicit ConeAvoidV2(ConeConfig c):cfg(c),visual(c){cfg.validate();}
    void update(const cv::Mat& frame,const LaneV2& lane,double timeSeconds,bool canStart=true){
        if(frame.empty()||frame.type()!=CV_8UC3||frame.cols!=640||frame.rows!=480||!std::isfinite(timeSeconds))throw std::invalid_argument("ConeAvoidV2 requires BGR 640x480 and finite time");
        if(timeReady&&timeSeconds<lastTime)throw std::invalid_argument("Cone time must be monotonic");
        timeReady=true;lastTime=timeSeconds;
        if(visual.searchFinished()){
            candidates.clear();mask=cv::Mat::zeros(frame.size(),CV_8UC1);
        }else detect(frame,lane);
        { // 唯一视觉绕桶路径。
            std::vector<cv::Rect> observations;for(const auto& c:candidates)observations.push_back(c.box);
            visual.update(observations,lane,timeSeconds,canStart);
            near=visual.near;
            offsetFraction=visual.offset;passedCount=visual.passed;phase=visual.phase;
            return;
        }

    }
    void draw(cv::Mat& image,const std::string& displayedPhase="")const{
        visual.draw(image);
        cv::putText(image,"CONE "+(displayedPhase.empty()?phase:displayedPhase)+" pass="+std::to_string(passedCount)+"/"+std::to_string(cfg.cone_lane_count)+" offset="+cv::format("%.3f",offsetFraction),{5,49},cv::FONT_HERSHEY_SIMPLEX,.43,{255,255,0},1,cv::LINE_AA);
    }
};
