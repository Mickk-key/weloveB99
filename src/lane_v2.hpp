// 公共车道几何与室内白线后端；室外 V2 复用 LaneV2 的误差计算和绘图。
// 这里的 LaneDetectorV2 仅在 lane_backend=indoor 时作为主巡线检测器运行。
#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

// 室内阈值以 HSV 低饱和度、高亮度筛白线；ROI、跨度及容差都使用原图像素。
struct LaneConfig {
    int lane_roi_top=300,lane_roi_bottom=470;
    int white_s_max=85,white_v_min=150;
    int lane_hough_votes=28,lane_min_span=55;
    double lane_fit_tolerance=18;int lane_fit_margin_px=160;
    void load(cv::FileStorage& f) {
#define LV(n) if(!f[#n].empty()) f[#n]>>n
        LV(lane_roi_top);LV(lane_roi_bottom);LV(white_s_max);LV(white_v_min);
        LV(lane_hough_votes);LV(lane_min_span);LV(lane_fit_tolerance);LV(lane_fit_margin_px);
#undef LV
        if(lane_fit_margin_px<0||lane_fit_margin_px>640||lane_roi_top<0||lane_roi_bottom>480||lane_roi_bottom-lane_roi_top<80||white_s_max<0||white_s_max>255||white_v_min<0||white_v_min>255||lane_hough_votes<5||lane_min_span<20||lane_min_span>lane_roi_bottom-lane_roi_top||!std::isfinite(lane_fit_tolerance)||lane_fit_tolerance<2||lane_fit_tolerance>50) throw std::runtime_error("Invalid white lane settings");
    }
};
// 左右边界均用 x=a*y+b 表示；Valid 表示本帧有真实拟合结果。
// 无效左侧返回 x=0，无效右侧返回 x=639；这些补位值不代表检测到真实白线。
struct LaneV2 {
    bool leftValid=false,rightValid=false;
    double la=0,lb=0,ra=0,rb=639,error=0;
    int top=300,bottom=470;
    double leftAt(double y) const {return leftValid?std::max(0.0,std::min(639.0,la*y+lb)):0.0;}
    double rightAt(double y) const {return rightValid?std::max(0.0,std::min(639.0,ra*y+rb)):639.0;}
    // Same original units: 320-pixel midpoint error divided by 4.
    // A missing side is ALWAYS the opposite image edge, not estimated lane width.
    // offset 是相对车道宽度的目标偏移：0 为中线，正值朝图像右侧，负值朝左侧。
    // 按多行加权平均：(目标横坐标-320)/8，换算为既有半分辨率巡线使用的误差尺度。
    double targetError(double offset=0) const {
        double sum=0,weights=0;
        for(int i=10;i<86;i+=2){double y=top+(bottom-top-1)*i/95.0;
            double w=std::min(8,1+std::min(i,95-i)/6);
            double l=leftAt(y),r=rightAt(y);
            sum+=((l+(r-l)*(0.5+offset)-320.0)/8.0)*w;weights+=w;
        }
        return sum/weights;
    }
    // 绿色表示真实边界，黄色分段线表示缺侧边缘补位，紫色点是当前偏移后的目标。
    // 绘图使用与 targetError 相同的边界查询，便于对照控制目标。
    void draw(cv::Mat& frame,double offset=0) const {
        for(int y=top;y<bottom-5;y+=5){
            if(leftValid||((y-top)/5)%3!=0) cv::line(frame,{int(leftAt(y)),y},{int(leftAt(y+5)),y+5},leftValid?cv::Scalar(0,255,0):cv::Scalar(0,220,255),2);
            if(rightValid||((y-top)/5)%3!=0) cv::line(frame,{int(rightAt(y)),y},{int(rightAt(y+5)),y+5},rightValid?cv::Scalar(0,255,0):cv::Scalar(0,220,255),2);
            double l=leftAt(y),r=rightAt(y);
            cv::circle(frame,{int((l+r)/2),y},1,{255,255,0},-1);
            cv::circle(frame,{int(l+(r-l)*(0.5+offset)),y},2,{255,0,255},-1);
        }
        cv::putText(frame,std::string("WHITE ")+(leftValid?(rightValid?"BOTH":"LEFT + RIGHT EDGE"):(rightValid?"RIGHT + LEFT EDGE":"EDGE FALLBACK")),{5,74},cv::FONT_HERSHEY_SIMPLEX,.48,{0,255,0},1);
    }
};

class LaneDetectorV2 {
    struct Segment {double a,b,len;cv::Point p,q;};
    struct Fit {bool valid=false;double a=0,b=0,score=0;};
    // 用上下两个参考行的横向差筛选同组线段；以总长度评分并做 Huber 直线拟合。
    // 最小竖直跨度过滤短小杂物，接近水平的拟合结果也会被排除。
    Fit fit(const std::vector<Segment>& lines) const {
        Fit best;
        for(const auto& seed:lines){
            std::vector<cv::Point2f> points;double score=0;int ymin=480,ymax=0;
            for(const auto& s:lines){
                double d1=std::abs((s.a-seed.a)*(c.lane_roi_top+30)+s.b-seed.b);
                double d2=std::abs((s.a-seed.a)*(c.lane_roi_bottom-20)+s.b-seed.b);
                if(std::max(d1,d2)>c.lane_fit_tolerance)continue;
                points.emplace_back(s.p);points.emplace_back(s.q);score+=s.len;
                ymin=std::min(ymin,std::min(s.p.y,s.q.y));ymax=std::max(ymax,std::max(s.p.y,s.q.y));
            }
            if(ymax-ymin<c.lane_min_span||score<=best.score||points.size()<2)continue;
            cv::Vec4f v;cv::fitLine(points,v,cv::DIST_HUBER,0,.01,.01);
            if(std::abs(v[1])<.1)continue;
            best.valid=true;best.a=v[0]/v[1];best.b=v[2]-best.a*v[3];best.score=score;
        }
        return best;
    }
public:
    LaneConfig c;cv::Mat mask;
    explicit LaneDetectorV2(LaneConfig config={}):c(config){}
    // 输入必须是 640×480 BGR；HSV 掩码只保留 ROI 内白色，再经闭运算和霍夫提线。
    LaneV2 detect(const cv::Mat& frame){
        CV_Assert(frame.cols==640&&frame.rows==480&&frame.type()==CV_8UC3);
        cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(0,0,c.white_v_min),cv::Scalar(179,c.white_s_max,255),mask);
        mask.rowRange(0,c.lane_roi_top).setTo(0);
        if(c.lane_roi_bottom<480)mask.rowRange(c.lane_roi_bottom,480).setTo(0);
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
        std::vector<cv::Vec4i> hough;cv::HoughLinesP(mask,hough,1,CV_PI/180,c.lane_hough_votes,35,12);
        std::vector<Segment> left,right;
        // 用近端位置与斜率分侧；允许弯道中接近竖直的边界，限制过度外推及横向反光。
        for(auto s:hough){
            double dy=s[3]-s[1],dx=s[2]-s[0];if(std::abs(dy)<24)continue;
            double a=dx/dy,b=s[0]-a*s[1],near=a*(c.lane_roi_bottom-25)+b;
            // Reject horizontal joins/reflections; accept near-vertical boundaries in bends.
            if(std::abs(a)>4||near< -c.lane_fit_margin_px||near>640+c.lane_fit_margin_px)continue;
            Segment seg{a,b,std::hypot(dx,dy),{s[0],s[1]},{s[2],s[3]}};
            if(near<302&&a<.45)left.push_back(seg);
            else if(near>338&&a>-.45)right.push_back(seg);
        }
        Fit l=fit(left),r=fit(right);
        // 两侧宽度关系异常时舍弃评分较低的一侧，之后按公共边缘补位规则计算误差。
        if(l.valid&&r.valid){
            double w0=(r.a-l.a)*(c.lane_roi_top+30)+r.b-l.b;
            double w1=(r.a-l.a)*(c.lane_roi_bottom-20)+r.b-l.b;
            if(w0<25||w1<100||w1<w0*.65){if(l.score<r.score)l.valid=false;else r.valid=false;}
        }
        LaneV2 result;result.leftValid=l.valid;result.rightValid=r.valid;
        result.la=l.a;result.lb=l.b;result.ra=r.a;result.rb=r.b;
        result.top=c.lane_roi_top;result.bottom=c.lane_roi_bottom;
        result.error=result.targetError();return result;
    }
};
