#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
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
