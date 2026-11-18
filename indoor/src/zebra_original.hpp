// 斑马线视觉模块：从白色掩码寻找多条间隔规则且在多行中连续出现的条纹。
// 本文件只返回识别结果；连续帧确认、底边阈值、制动和等待由 Suite::ZebraGate 处理。
#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
using std::string;using std::vector;
namespace zebra {
// ROI、条纹宽度、跨度和支持高度是相对整幅图像的比例；HSV 阈值使用 OpenCV 范围。
// zebra_confirm_frames 属于上层跨帧门控，不是本文件内的支持行数。
struct Config {
    int zebra_white_s_max=75,zebra_white_v_min=175,zebra_min_stripes=4,zebra_confirm_frames=3;
    double zebra_roi_left=.12,zebra_roi_right=.88,zebra_roi_top=.45,zebra_roi_bottom=.94;
    double zebra_min_stripe_width=.008,zebra_max_stripe_width=.14,zebra_min_span=.12,zebra_min_support_height=.008;
    double zebra_max_width_ratio=2.2,zebra_max_pitch_ratio=1.8,zebra_min_gap_ratio=.15,zebra_max_gap_ratio=2.5;
};
// found 是本帧模式是否成立；bands 为条纹数，supporting_rows 为同一图像内的支持行数。
// box/stripes 使用整图坐标，reason 说明未确认或已确认的原因。
struct ZebraResult {
    bool found=false;int bands=0,supporting_rows=0,scan_y=-1;
    cv::Rect roi,box;vector<cv::Rect> stripes;string reason="no_pattern";
};
}
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace zebra {
using namespace cv;
using namespace std;

// 校验比例、条纹数、颜色阈值以及宽度/间距比，避免无效配置进入扫描逻辑。
void validate_zebra_config(const Config& c) {
    auto unit=[](double n){return isfinite(n)&&n>=0&&n<=1;};
    if(!unit(c.zebra_roi_left)||!unit(c.zebra_roi_right)||c.zebra_roi_left>=c.zebra_roi_right ||
       !unit(c.zebra_roi_top)||!unit(c.zebra_roi_bottom)||c.zebra_roi_top>=c.zebra_roi_bottom ||
       !unit(c.zebra_min_stripe_width)||!unit(c.zebra_max_stripe_width)||c.zebra_min_stripe_width<=0||c.zebra_min_stripe_width>=c.zebra_max_stripe_width ||
       !unit(c.zebra_min_span)||c.zebra_min_span<=0 ||
       !unit(c.zebra_min_support_height)||c.zebra_min_support_height<=0 ||
       c.zebra_min_stripes<3||c.zebra_min_stripes>24||c.zebra_confirm_frames<1||c.zebra_confirm_frames>120 ||
       c.zebra_white_s_max<0||c.zebra_white_s_max>255||c.zebra_white_v_min<0||c.zebra_white_v_min>255 ||
       !isfinite(c.zebra_max_width_ratio)||c.zebra_max_width_ratio<1 ||
       !isfinite(c.zebra_max_pitch_ratio)||c.zebra_max_pitch_ratio<1 ||
       !isfinite(c.zebra_min_gap_ratio)||!isfinite(c.zebra_max_gap_ratio)||c.zebra_min_gap_ratio<=0||c.zebra_min_gap_ratio>=c.zebra_max_gap_ratio)
        throw invalid_argument("Invalid zebra_* configuration (ROI, thresholds or stripe geometry)");
}

namespace {
struct RowPattern { int y; vector<Rect> runs; };
struct Track {
    RowPattern last,best;
    int first_y,rows=1;
    Rect box;
};
int span(const vector<Rect>& r) {return r.back().br().x-r.front().x;}
// 跨相邻图像行匹配条纹：数量必须一致，每条左右端点差不能超过按条纹宽度给出的容差。
bool aligned(const RowPattern& a,const RowPattern& b) {
    if(a.runs.size()!=b.runs.size())return false;
    for(size_t i=0;i<a.runs.size();++i){
        double tolerance=max(3.0,a.runs[i].width*0.55);
        if(abs(a.runs[i].x-b.runs[i].x)>tolerance ||
           abs(a.runs[i].br().x-b.runs[i].br().x)>tolerance)return false;
    }
    return true;
}
// 检查同一行条纹组的总跨度、最大/最小条宽、空隙与中位条宽之比，以及中心间距比。
// 这些约束用来排除不规则白块，不只是数够几条白色就判定。
bool regular(const vector<Rect>& runs,const Config& c,int min_span) {
    if(span(runs)<min_span)return false;
    vector<double> widths,pitches;
    for(const auto& r:runs)widths.push_back(r.width);
    auto wm=minmax_element(widths.begin(),widths.end());
    if(*wm.second / *wm.first > c.zebra_max_width_ratio)return false;
    sort(widths.begin(),widths.end());double median=widths[widths.size()/2];
    for(size_t i=1;i<runs.size();++i){
        int gap=runs[i].x-runs[i-1].br().x;
        if(gap<max(2.0,median*c.zebra_min_gap_ratio)||gap>median*c.zebra_max_gap_ratio)return false;
        pitches.push_back(runs[i].x+runs[i].width*.5-runs[i-1].x-runs[i-1].width*.5);
    }
    auto pm=minmax_element(pitches.begin(),pitches.end());
    return *pm.second / *pm.first <= c.zebra_max_pitch_ratio;
}
}

// 接受至少 20×20 的单通道白色二值掩码；从比例配置换算本帧像素阈值。
// Track 仅跟踪同一帧内相邻扫描行，不会保存到下一帧。
ZebraResult detect_zebra(const Mat& white,const Config& c) {
    validate_zebra_config(c);
    if(white.empty()||white.type()!=CV_8UC1||white.cols<20||white.rows<20)
        throw invalid_argument("Zebra detector expects a CV_8UC1 mask at least 20x20");
    const int w=white.cols,h=white.rows;
    int x0=int(w*c.zebra_roi_left),x1=int(w*c.zebra_roi_right);
    int y0=int(h*c.zebra_roi_top),y1=int(h*c.zebra_roi_bottom);
    ZebraResult result;result.roi=Rect(x0,y0,max(1,x1-x0),max(1,y1-y0));
    // Horizontal opening removes tiny white speckles without joining adjacent stripes.
    Mat clean;
    int kernel=max(3,int(round(w*0.004))|1);
    morphologyEx(white,clean,MORPH_OPEN,getStructuringElement(MORPH_RECT,Size(kernel,1)));
    const int min_width=max(3,int(ceil(w*c.zebra_min_stripe_width)));
    const int max_width=max(min_width,int(w*c.zebra_max_stripe_width));
    const int min_span=max(1,int(ceil(w*c.zebra_min_span)));
    const int min_rows=max(3,int(ceil(h*c.zebra_min_support_height)));
    vector<Track> tracks;
    // 优先选择支持行数已达标的轨迹，再比较支持行数和条纹数；合并其覆盖框作为结果。
    auto consider=[&](const Track& t){
        bool found=t.rows>=min_rows;
        if((found&&!result.found)||(found==result.found &&
           (t.rows>result.supporting_rows||(t.rows==result.supporting_rows&&int(t.best.runs.size())>result.bands)))){
            result.found=found;result.bands=int(t.best.runs.size());result.supporting_rows=t.rows;
            result.box=t.box;result.scan_y=t.best.y;result.stripes=t.best.runs;
            for(auto& r:result.stripes){r.y=t.first_y;r.height=t.last.y-t.first_y+1;}
            result.reason=found?"horizontal_stripes":"too_few_support_rows";
        }
    };
    // 逐行提取连续白色区间，排除被 ROI 边缘截断的区间和小白点。
    for(int y=y0;y<y1;++y){
        vector<Rect> runs;
        const auto* pixels=clean.ptr<uchar>(y);
        for(int x=x0;x<x1;){
            if(!pixels[x]){++x;continue;}
            int start=x;while(x<x1&&pixels[x])++x;
            // Ignore runs clipped by the ROI and isolated white dots.
            if(start>x0&&x<x1&&x-start>=min_width)runs.emplace_back(start,y,x-start,1);
        }
        vector<RowPattern> patterns;
        // 从每个起点扩展条纹组，保留满足 regular 的最长组；单组最多检查 24 条。
        for(size_t i=0;i<runs.size();++i){
            vector<Rect> group;RowPattern best{y,{}};
            for(size_t j=i;j<runs.size()&&j<i+24;++j){
                if(runs[j].width>max_width)break;
                group.push_back(runs[j]);
                if(int(group.size())>=c.zebra_min_stripes&&regular(group,c,min_span))best.runs=group;
            }
            if(!best.runs.empty()){
                patterns.push_back(best);
                // The maximal group already covers these start positions.
                i+=best.runs.size()-1;
            }
        }
        // 把本行条纹组接到最多相隔两行且左右端点对齐的轨迹；未匹配时创建新轨迹。
        // 轨迹不再延续时从活动列表移除，但已经评估出的最佳结果保留。
        vector<bool> used(tracks.size(),false);
        for(const auto& p:patterns){
            int match=-1;
            for(size_t i=0;i<tracks.size();++i)
                if(!used[i]&&y-tracks[i].last.y<=2&&aligned(tracks[i].last,p)){match=int(i);break;}
            Rect box(p.runs.front().x,y,span(p.runs),1);
            if(match>=0){
                auto& t=tracks[match];t.last=p;t.rows++;t.box|=box;
                if(span(p.runs)>=span(t.best.runs))t.best=p;
                used[match]=true;
            } else { tracks.push_back({p,p,y,1,box});used.push_back(true); }
        }
        for(const auto& t:tracks)consider(t);
        tracks.erase(remove_if(tracks.begin(),tracks.end(),[&](const Track& t){return y-t.last.y>=2;}),tracks.end());
    }
    return result;
}
}


namespace zebra {
// BGR 转 HSV 后按低饱和度和最低亮度取白色，交给几何条纹检测；不在这里发停车指令。
ZebraResult detect_frame(const Mat& bgr,const Config& c){
    Mat hsv,mask;cvtColor(bgr,hsv,COLOR_BGR2HSV);
    inRange(hsv,Scalar(0,0,c.zebra_white_v_min),Scalar(179,c.zebra_white_s_max,255),mask);
    return detect_zebra(mask,c);
}
}
