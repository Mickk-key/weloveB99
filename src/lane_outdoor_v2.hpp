// 新版室外巡线：白色候选提取 → 线段分组拟合 → 双侧几何检查 → 已观测边界平滑。
// 由 Suite 在 lane_backend=outdoor_v2 时调用；所有像素阈值基于 640×480 图像。
#pragma once
#include "lane_v2.hpp"

// ROI 为图像行号；sMax/vMin/brightV 是 HSV 阈值；contrast 是灰度局部亮度差。
// fitTolerance 为拟合分组的像素容差；smoothAlpha 是当前帧在平滑结果中的权重。
struct OutdoorLaneConfig {
    int roiTop=275,roiBottom=475,sMax=130,vMin=50,brightV=150;
    int votes=18,minLength=32,maxGap=18,minSpan=50,margin=320;
    double contrast=3,fitTolerance=16,smoothAlpha=.65;
    // 仅覆盖 YAML 中实际存在的字段，再校验 ROI、霍夫参数、拟合容差和滤波范围。
    void load(const cv::FileStorage& f){
#define R(key,value) if(!f[key].empty())f[key]>>value
        R("outdoor_roi_top",roiTop);R("outdoor_roi_bottom",roiBottom);
        R("outdoor_white_s_max",sMax);R("outdoor_white_v_min",vMin);R("outdoor_bright_v",brightV);
        R("outdoor_local_contrast",contrast);R("outdoor_hough_votes",votes);R("outdoor_min_length",minLength);
        R("outdoor_max_gap",maxGap);R("outdoor_min_span",minSpan);R("outdoor_fit_margin",margin);
        R("outdoor_fit_tolerance",fitTolerance);R("outdoor_smooth_alpha",smoothAlpha);
#undef R
        if(roiTop<200||roiBottom>480||roiBottom-roiTop<80||sMax<0||sMax>255||vMin<0||vMin>255||brightV<vMin||brightV>255||votes<5||minLength<15||maxGap<0||maxGap>60||minSpan<25||minSpan>roiBottom-roiTop||margin<0||margin>640||!std::isfinite(contrast)||contrast<0||contrast>60||!std::isfinite(fitTolerance)||fitTolerance<2||fitTolerance>50||!std::isfinite(smoothAlpha)||smoothAlpha<=0||smoothAlpha>1)throw std::runtime_error("Invalid outdoor_v2 lane parameters");
    }
};

class OutdoorLaneDetector {
    // 用 x=a*y+b 表示边界，接近竖直的车道线也能稳定表示；length 用作候选组评分。
    // Fit.firstY 保存该组实际出现的最上方行，用于避开无真实线段支持的远端几何检查。
    struct Segment {double a,b,length;cv::Point2f p,q;};
    struct Fit {bool valid=false;double a=0,b=0,score=0;int span=0,firstY=480;};
    OutdoorLaneConfig c;LaneV2 previous;double last=-1;
    // 把每段依次作为种子，在 ROI 上下两个参考行比较横坐标差，聚集位置一致的线段。
    // 总长度更大且竖直跨度足够的组，使用 Huber 距离拟合得到最终边界。
    Fit fit(const std::vector<Segment>& segments)const{
        Fit best;
        for(const auto& seed:segments){
            std::vector<cv::Point2f> points;double score=0;int low=480,high=0;
            for(const auto& s:segments){
                double d0=std::abs((s.a-seed.a)*(c.roiTop+25)+s.b-seed.b);
                double d1=std::abs((s.a-seed.a)*(c.roiBottom-25)+s.b-seed.b);
                if(std::max(d0,d1)>c.fitTolerance)continue;
                points.push_back(s.p);points.push_back(s.q);score+=s.length;
                low=std::min(low,int(std::min(s.p.y,s.q.y)));high=std::max(high,int(std::max(s.p.y,s.q.y)));
            }
            if(high-low<c.minSpan||score<=best.score||points.size()<2)continue;
            cv::Vec4f v;cv::fitLine(points,v,cv::DIST_HUBER,0,.01,.01);
            if(std::abs(v[1])<.1)continue;
            double a=v[0]/v[1],b=v[2]-a*v[3];
            best={true,a,b,score,high-low,low};
        }
        return best;
    }
public:
    cv::Mat mask;double leftScore=0,rightScore=0;
    explicit OutdoorLaneDetector(OutdoorLaneConfig config={}):c(config){}
    LaneV2 detect(const cv::Mat& frame,double now){
        CV_Assert(frame.size()==cv::Size(640,480)&&frame.type()==CV_8UC3);
        cv::Mat hsv,gray,blur,fgray,fblur,contrast,color,bright,neutral,support,clean;
        // 3×3 中值滤波抑制孤立噪声；灰度减去大尺度高斯背景，提取比周围更亮的区域。
        cv::medianBlur(frame,clean,3);
        cv::cvtColor(clean,hsv,cv::COLOR_BGR2HSV);cv::cvtColor(clean,gray,cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray,blur,{0,0},9);
        gray.convertTo(fgray,CV_32F);blur.convertTo(fblur,CV_32F);
        cv::compare(fgray-fblur,c.contrast,contrast,cv::CMP_GT);
        // 白线候选须满足低饱和度和最低亮度，并满足“局部更亮”或“足够亮且更低饱和度”。
        // 先组合颜色和亮度支持，再限制地面 ROI；掩码是给霍夫变换使用的二值图。
        cv::inRange(hsv,cv::Scalar(0,0,c.vMin),cv::Scalar(179,c.sMax,255),color);
        cv::inRange(hsv,cv::Scalar(0,0,c.brightV),cv::Scalar(179,65,255),bright);
        cv::bitwise_or(contrast,bright,support);cv::bitwise_and(color,support,mask);
        mask.rowRange(0,c.roiTop).setTo(0);mask.rowRange(c.roiBottom,480).setTo(0);
        // 3×5 闭运算连接小断点；概率霍夫变换输出候选线段，并非直接作为最终左右边界。
        cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,5}));
        std::vector<cv::Vec4i> lines;cv::HoughLinesP(mask,lines,1,CV_PI/180,c.votes,c.minLength,c.maxGap);
        std::vector<Segment> left,right;
        // 过滤竖直跨度过短、方向不合理或近端外推过远的线段。
        // 按近端横坐标及斜率划分左/右候选，中间 310..330 的模糊区域不强行分侧。
        for(auto s:lines){double dx=s[2]-s[0],dy=s[3]-s[1];if(std::abs(dy)<24)continue;
            double a=dx/dy,b=s[0]-a*s[1],near=a*(c.roiBottom-20)+b;
            if(std::abs(a)>4||near< -c.margin||near>640+c.margin)continue;
            Segment seg{a,b,std::hypot(dx,dy),{float(s[0]),float(s[1])},{float(s[2]),float(s[3])}};
            if(near<310&&a<.55)left.push_back(seg);else if(near>330&&a>-.55)right.push_back(seg);
        }
        auto l=fit(left),r=fit(right);
        // 两侧均有效时，在实际线段起始高度以下检查车道宽度和远近关系。
        // 出现交叉、过窄或近端异常收缩时，仅保留累计线段长度评分较高的一侧。
        if(l.valid&&r.valid){double anchor=std::max(c.roiTop+25,std::max(l.firstY,r.firstY)+10);double w0=(r.a-l.a)*anchor+r.b-l.b,w1=(r.a-l.a)*(c.roiBottom-20)+r.b-l.b;
            if(w0<20||w1<100||w1<w0*.65){if(l.score<r.score)l.valid=false;else r.valid=false;}}
        LaneV2 out;out.top=c.roiTop;out.bottom=c.roiBottom;out.leftValid=l.valid;out.rightValid=r.valid;
        out.la=l.a;out.lb=l.b;out.ra=r.a;out.rb=r.b;leftScore=l.score;rightScore=r.score;
        // Smooth only currently observed real lines. Never invent a missing line.
        // 只在帧间隔小于 0.25 秒且同侧本帧和上帧都真实检测到时平滑 a/b。
        // 本帧缺线立即交给 LaneV2 的画面边缘补位，不用上一帧边界冒充当前观测。
        if(last>=0&&now>last&&now-last<.25){double a=c.smoothAlpha;
            if(l.valid&&previous.leftValid){out.la=a*out.la+(1-a)*previous.la;out.lb=a*out.lb+(1-a)*previous.lb;}
            if(r.valid&&previous.rightValid){out.ra=a*out.ra+(1-a)*previous.ra;out.rb=a*out.rb+(1-a)*previous.rb;}}
        // 按公共 LaneV2 的误差单位计算转向输入，再保存本帧用于下一次平滑。
        out.error=out.targetError();previous=out;last=now;return out;
    }
};
