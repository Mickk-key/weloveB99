// 蓝挡板识别：颜色候选、完整矩形形状及跨帧稳定确认。
// stopped 表示蓝挡板已确认的锁存信号；在集成二维码停车中，它用于选空闲半区，
// 并不单独代表二维码最终停车完成，实际电机命令由 Suite 仲裁。
#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>

// ROI 门槛和最小尺寸是 640×480 原图像素；fill 为轮廓占旋转外接矩形的面积比。
// 确认同时要求帧数和观察秒数，参数从当前环境 YAML 读取。
struct BoardConfig {
    int board_h_min=100,board_h_max=130,board_s_min=40,board_v_min=55;
    int board_roi_top=100,board_roi_left=64,board_roi_right=576;
    int board_min_area=1500,board_min_width=40,board_min_height=25,board_confirm_frames=5;
    double board_min_fill=.82,board_min_aspect=.8,board_max_aspect=5,board_confirm_seconds=.15;
    void load(const cv::FileStorage& f){
#define B(n) if(!f[#n].empty()) f[#n]>>n
        B(board_h_min);B(board_h_max);B(board_s_min);B(board_v_min);B(board_roi_top);B(board_roi_left);B(board_roi_right);
        B(board_min_area);B(board_min_width);B(board_min_height);B(board_confirm_frames);B(board_min_fill);B(board_min_aspect);B(board_max_aspect);B(board_confirm_seconds);
#undef B
        if(board_h_min<0||board_h_max>179||board_h_min>board_h_max||board_s_min<0||board_s_min>255||board_v_min<0||board_v_min>255||board_roi_top<0||board_roi_top>400||board_roi_left<0||board_roi_right>640||board_roi_right-board_roi_left<80||board_min_area<100||board_min_area>300000||board_min_width<5||board_min_height<5||board_confirm_frames<2||board_confirm_frames>60||!std::isfinite(board_min_fill)||board_min_fill<.6||board_min_fill>1||!std::isfinite(board_min_aspect)||!std::isfinite(board_max_aspect)||board_min_aspect<=0||board_max_aspect<board_min_aspect||!std::isfinite(board_confirm_seconds)||board_confirm_seconds<.05||board_confirm_seconds>3)throw std::runtime_error("Invalid blue board configuration");
    }
};
class BlueBoardStop {
    BoardConfig c;cv::Rect previous;double since=0,last=-1;
public:
    bool stopped=false,candidate=false;int hits=0;cv::Rect box;cv::Mat mask;
    explicit BlueBoardStop(BoardConfig cfg={}):c(cfg){}
    // 先全图提取蓝色外轮廓，再按候选中心是否位于 ROI 筛选，避免裁切改变物体形状。
    void update(const cv::Mat& frame,double now){
        if(frame.size()!=cv::Size(640,480)||frame.type()!=CV_8UC3||!std::isfinite(now)||now<last)throw std::runtime_error("Board input/time invalid");
        double gap=last<0?0:now-last;last=now;
        cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(c.board_h_min,c.board_s_min,c.board_v_min),cv::Scalar(c.board_h_max,255,255),mask);
        // Detect full silhouettes before gating their centers, so ROI clipping cannot turn a cone into a rectangle.
        std::vector<std::vector<cv::Point>> contours;cv::findContours(mask.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        candidate=false;box={};double best=0;
        for(const auto& contour:contours){
            auto r=cv::boundingRect(contour);double area=cv::contourArea(contour),aspect=double(r.width)/r.height;
            if(area<c.board_min_area||r.width<c.board_min_width||r.height<c.board_min_height||aspect<c.board_min_aspect||aspect>c.board_max_aspect)continue;
            if(r.x<=0||r.y<=0||r.br().x>=640||r.br().y>=480)continue; // incomplete shape cannot certify a board
            if(r.x+r.width/2<c.board_roi_left||r.x+r.width/2>c.board_roi_right||r.y+r.height/2<c.board_roi_top)continue;
            // 用矩形填充率、凸四边形和近直角约束区分挡板与锥桶；多候选时保留面积最大的。
            cv::RotatedRect rr=cv::minAreaRect(contour);double rectArea=rr.size.area();
            if(rectArea<1||area/rectArea<c.board_min_fill)continue;
            std::vector<cv::Point> poly;cv::approxPolyDP(contour,poly,.025*cv::arcLength(contour,true),true);
            if(poly.size()!=4||!cv::isContourConvex(poly))continue;
            double maxCos=0;
            for(int i=0;i<4;++i){cv::Point2d u=poly[(i+3)%4]-poly[i],v=poly[(i+1)%4]-poly[i];maxCos=std::max(maxCos,std::abs(u.dot(v))/(cv::norm(u)*cv::norm(v)+1e-9));}
            if(maxCos>.45)continue;
            if(area>best){best=area;box=r;candidate=true;}
        }
        // 一旦确认就保持锁存，挡板移走也不自动解除；上方仍更新当前 candidate/box 供选侧使用。
        if(stopped)return; // Latch survives loss/removal of the board. Restart is explicit.
        if(!candidate){hits=0;previous={};return;}
        // 相邻框重叠不足或帧间隔超过 0.25 秒时重新计数；帧数和时长都达标才置 stopped。
        // 此处重叠率的分母是两框的外接矩形面积，按现有表达式使用。
        double overlap=previous.area()>0?double((box&previous).area())/std::max(1,(box|previous).area()):0;
        if(hits==0||gap>.25||overlap<.3){hits=1;since=now;}else ++hits;
        previous=box;
        if(hits>=c.board_confirm_frames&&now-since>=c.board_confirm_seconds)stopped=true;
    }
    // 辅助诊断显示：区分搜索、连续确认和已锁存；不参与识别或硬件输出。
    void draw(cv::Mat& frame)const{
        if(candidate)cv::rectangle(frame,box,stopped?cv::Scalar(0,0,255):cv::Scalar(255,255,0),2);
        cv::putText(frame,std::string("BLUE BOARD ")+(stopped?"STOP LATCHED":candidate?"CONFIRM "+std::to_string(hits):"SEARCH"),{5,98},cv::FONT_HERSHEY_SIMPLEX,.55,stopped?cv::Scalar(0,0,255):cv::Scalar(255,255,0),2);
    }
};
