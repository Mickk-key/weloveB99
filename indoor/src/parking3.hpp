#pragma once
#include "blue_board.hpp"
#include "lane_geometry.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// 当前唯一停车实现：蓝板/黄横线选侧 → 所选侧白边 → 白矩形锁停。坐标640×480。
// 蓝板使用board_*；白边直接使用巡线几何。这里只加载实际使用的黄线/跟边/白矩形参数。
struct Parking3Config {
    int hmin=18,hmax=40,smin=60,vmin=100,roiTop=180,roiBottom=470;
    int minLength=65,houghVotes=25,maxGap=18,sideFrames=2,sideGap=25;
    double maxSlope=.25,kp=.4,kd=.5,steerLimit=15.,steerRate=40.;
    int forwardPwm=10600,waitPwm=10500,boardFullWidth=1;
    double distanceNear=100,distanceFar=40,errorBias=0,holdSeconds=.3;
    int stopWhiteV=230,stopWhiteS=120,stopMinArea=2500,stopMinBottom=200,stopFrames=3;
    double stopMinFill=.65,stopSeconds=.2,stopCenterTolerance=.3;
    int stopDimV=90,stopDimS=90,stopDarkV=100;
    double stopDarkFraction=.6;
    void load(const cv::FileStorage& f){
#define P2(k,v) if(!f[k].empty())f[k]>>v
        P2("parking3_yellow_h_min",hmin);P2("parking3_yellow_h_max",hmax);
        P2("parking3_yellow_s_min",smin);P2("parking3_yellow_v_min",vmin);
        P2("parking3_roi_top",roiTop);P2("parking3_roi_bottom",roiBottom);
        P2("parking3_min_length",minLength);P2("parking3_hough_votes",houghVotes);
        P2("parking3_max_gap",maxGap);P2("parking3_max_slope",maxSlope);
        P2("parking3_side_frames",sideFrames);P2("parking3_side_gap_px",sideGap);
        P2("parking3_follow_pwm",forwardPwm);P2("parking3_kp",kp);P2("parking3_kd",kd);
        P2("parking3_steer_limit",steerLimit);P2("parking3_steer_rate",steerRate);
        P2("parking3_stop_white_v_min",stopWhiteV);P2("parking3_stop_white_s_max",stopWhiteS);
        P2("parking3_stop_min_area",stopMinArea);P2("parking3_stop_min_bottom",stopMinBottom);
        P2("parking3_stop_min_fill",stopMinFill);P2("parking3_stop_confirm_frames",stopFrames);
        P2("parking3_stop_confirm_seconds",stopSeconds);
        P2("parking3_stop_center_tolerance",stopCenterTolerance);
        P2("parking3_stop_dim_v_min",stopDimV);P2("parking3_stop_dim_s_max",stopDimS);
        P2("parking3_stop_dark_v_max",stopDarkV);P2("parking3_stop_dark_ring_fraction",stopDarkFraction);
        P2("parking3_board_full_width",boardFullWidth);
        if(boardFullWidth!=0&&boardFullWidth!=1)throw std::runtime_error("parking3_board_full_width must be 0 or 1");
        P2("parking3_distance_near_px",distanceNear);P2("parking3_distance_far_px",distanceFar);
        P2("parking3_error_bias",errorBias);P2("parking3_line_hold_seconds",holdSeconds);P2("parking3_wait_side_pwm",waitPwm);
        if(!std::isfinite(distanceNear)||distanceNear<20||distanceNear>250||!std::isfinite(distanceFar)||distanceFar<5||distanceFar>distanceNear||!std::isfinite(errorBias)||std::abs(errorBias)>20||!std::isfinite(holdSeconds)||holdSeconds<0||holdSeconds>1||waitPwm<10000||waitPwm>12000)throw std::runtime_error("Invalid parking3 edge offset config");
#undef P2
        if(hmin<0||hmax>179||hmin>hmax||smin<0||smin>255||vmin<0||vmin>255||
           roiTop<0||roiBottom>480||roiBottom-roiTop<80||minLength<20||minLength>640||
           houghVotes<2||houghVotes>300||maxGap<0||maxGap>100||sideFrames<1||sideFrames>30||
           sideGap<0||sideGap>200||!std::isfinite(maxSlope)||maxSlope<0||maxSlope>.8||
           forwardPwm<=10000||forwardPwm>12000||!std::isfinite(kp)||kp<0||kp>2||
           !std::isfinite(kd)||kd<0||kd>2||!std::isfinite(steerLimit)||steerLimit<=0||steerLimit>15||
           !std::isfinite(steerRate)||steerRate<=0||steerRate>100||
           stopWhiteV<1||stopWhiteV>255||
           stopWhiteS<0||stopWhiteS>255||stopMinArea<500||stopMinArea>50000||
           stopMinBottom<100||stopMinBottom>479||!std::isfinite(stopMinFill)||
           stopMinFill<.3||stopMinFill>.95||stopFrames<2||stopFrames>30||
           !std::isfinite(stopSeconds)||stopSeconds<.05||stopSeconds>3||
           !std::isfinite(stopCenterTolerance)||stopCenterTolerance<.05||stopCenterTolerance>.5||
           stopDimV<1||stopDimV>255||stopDimS<0||stopDimS>255||
           stopDarkV<0||stopDarkV>255||!std::isfinite(stopDarkFraction)||
           stopDarkFraction<0||stopDarkFraction>1)
            throw std::runtime_error("Invalid parking3 configuration");
    }
};

class Parking3 {
    Parking3Config c;
    BlueBoardStop blueGate;
    static BoardConfig gateConfig(BoardConfig b,int full){if(full){b.board_roi_left=0;b.board_roi_right=640;}return b;}
    int proposal=0,hits=0,stopHits=0;
    double previousError=0,boardCenter=320.,stopSince=-1,stopLast=-1;
    bool previousValid=false,stopFound=false;
    cv::Rect stopBox,stopPrevious;
    double whiteSeen=-1,targetNear=320,targetFar=320;
    bool detectStopRectangle(const cv::Mat& hsv,double now,bool enabled){
        stopFound=false;stopBox={};double best=0;
        cv::Mat value,dark;cv::extractChannel(hsv,value,2);
        cv::threshold(value,dark,c.stopDarkV,255,cv::THRESH_BINARY_INV);
        for(int mode=0;mode<2;++mode){
            cv::Mat white;cv::inRange(hsv,
                cv::Scalar(0,0,mode?c.stopDimV:c.stopWhiteV),
                cv::Scalar(179,mode?c.stopDimS:c.stopWhiteS,255),white);
            white.rowRange(0,100).setTo(0);
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(white,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
            for(const auto& q:contours){auto r=cv::boundingRect(q);double area=cv::contourArea(q);
                // 沿用jgy/race最新逻辑：近处白板允许超过旧250×300上限，其余门槛保留。
                if(r.width<35||r.height<40||
                   r.br().y<c.stopMinBottom||r.area()<c.stopMinArea||
                   area/r.area()<c.stopMinFill||double(r.width)/r.height<.35||
                   double(r.width)/r.height>1.8||
                   std::abs(r.x+r.width*.5-hsv.cols*.5)>hsv.cols*c.stopCenterTolerance)continue;
                if(mode){cv::Rect outer(r.x-10,r.y-10,r.width+20,r.height+20);
                    outer&=cv::Rect(0,0,hsv.cols,hsv.rows);
                    int ring=outer.area()-r.area();
                    if(ring<=0||double(cv::countNonZero(dark(outer))-cv::countNonZero(dark(r)))/ring<c.stopDarkFraction)continue;}
                if(area>best){best=area;stopFound=true;stopBox=r;}
            }
        }
        if(!enabled||!stopFound){stopHits=0;stopLast=now;return false;}
        double overlap=stopPrevious.empty()?0:double((stopBox&stopPrevious).area())/(stopBox|stopPrevious).area();
        if(!stopHits||now-stopLast>.3||overlap<.4){stopHits=1;stopSince=now;}else ++stopHits;
        stopPrevious=stopBox;stopLast=now;
        return stopHits>=c.stopFrames&&now-stopSince+1e-9>=c.stopSeconds;
    }
public:
    int side=0;bool line=false,stopped=false,useLane=false;double midX=320,error=0,correction=0;
    double whiteNear=0,whiteFar=0,whiteA=0,whiteB=0;
    cv::Vec4i segment{0,0,0,0};cv::Mat mask;
    std::string state="PARK3_WAIT_BOARD",path="NONE";
    explicit Parking3(Parking3Config cfg={},BoardConfig boardCfg={}):c(cfg),blueGate(gateConfig(boardCfg,cfg.boardFullWidth)){}
    double rate()const{return c.steerRate;}
    int motor()const{return side?c.forwardPwm:c.waitPwm;}
    bool engaged()const{return side!=0||state=="PARK3_SEARCH_LINE"||stopped;}
    bool panelFound()const{return stopFound;}
    int panelHits()const{return stopHits;}
    void update(const cv::Mat& frame,double now,bool canStart,const LaneV2& lane){
        if(stopped){state="PARK3_PANEL_STOP";path="STOP";return;}
        cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        // Shared board_* thresholds; full-width observation changes ROI only, never area/time gates.
        blueGate.update(frame,now);
        const bool blueConfirmed=blueGate.stopped;
        if(blueGate.candidate)boardCenter=blueGate.box.x+blueGate.box.width*.5;
        cv::inRange(hsv,cv::Scalar(c.hmin,c.smin,c.vmin),cv::Scalar(c.hmax,255,255),mask);
        mask.rowRange(0,c.roiTop).setTo(0);mask.rowRange(c.roiBottom,frame.rows).setTo(0);
        cv::morphologyEx(mask,mask,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,{5,3}));
        std::vector<cv::Vec4i> lines;cv::HoughLinesP(mask,lines,1,CV_PI/180,c.houghVotes,c.minLength,c.maxGap);
        double best=-1;line=false;
        for(auto s:lines){double dx=s[2]-s[0],dy=s[3]-s[1],len=std::hypot(dx,dy);
            if(std::abs(dx)<c.minLength||std::abs(dy/dx)>c.maxSlope)continue;
            double x=(s[0]+s[2])*.5;
            if(side && (side>0?x<boardCenter+c.sideGap:x>boardCenter-c.sideGap))continue;
            if(len>best){best=len;segment=s;midX=x;line=true;}
        }
        if(!side&&!blueConfirmed){state="PARK3_WAIT_BOARD";path="NONE";return;}
        if(!side){
            int candidate=(line&&blueGate.candidate)?(midX>boardCenter+c.sideGap?1:midX<boardCenter-c.sideGap?-1:0):0;
            if(candidate && candidate==proposal)++hits;else{proposal=candidate;hits=candidate?1:0;}
            if(hits>=c.sideFrames)side=candidate;
            if(!side){useLane=true;state="PARK3_SEARCH_LINE";path="SEARCH_WITH_LANE";
                error=correction=0;previousValid=false;return;}
        }
        if(detectStopRectangle(hsv,now,canStart)){stopped=true;state="PARK3_PANEL_STOP";path="STOP";error=correction=0;return;}
        useLane=false;error=correction=0;
        const int nearY=410,farY=230;
        // Reuse the exact geometry already produced by the active lane backend.
        // No second HSV/Hough pass and no parking-specific boundary reselection.
        bool found=side>0?lane.rightValid:lane.leftValid;
        if(found){
            whiteNear=side>0?lane.rightAt(nearY):lane.leftAt(nearY);
            whiteFar=side>0?lane.rightAt(farY):lane.leftAt(farY);
            whiteA=(whiteNear-whiteFar)/double(nearY-farY);
            whiteB=whiteNear-whiteA*nearY;
        }
        if(found){whiteSeen=now;path="WHITE_MEASURED";}
        else if(whiteSeen>=0&&now-whiteSeen<=c.holdSeconds)path="WHITE_HELD";
        else{whiteA=0;whiteB=side>0?639:0;path="FRAME_EDGE";}
        whiteNear=whiteA*nearY+whiteB;whiteFar=whiteA*farY+whiteB;
        targetNear=whiteNear-side*c.distanceNear+8*c.errorBias;
        targetFar=whiteFar-side*c.distanceFar+8*c.errorBias;
        error=(.75*targetNear+.25*targetFar-frame.cols*.5)/8.;
        correction=std::max(-c.steerLimit,std::min(c.steerLimit,c.kp*error+(previousValid?c.kd*(error-previousError):0.)));
        previousError=error;previousValid=true;state="PARK3_FOLLOW_WHITE";
    }
    void draw(cv::Mat& view)const{
        if(side&&state=="PARK3_FOLLOW_WHITE"){
            cv::line(view,{int(whiteFar),230},{int(whiteNear),410},path=="WHITE_MEASURED"?cv::Scalar(255,255,255):cv::Scalar(0,180,255),2);
            for(int y=230;y<=410;y+=8){double t=(y-230)/180.;cv::circle(view,{int(targetFar+(targetNear-targetFar)*t),y},3,{255,0,255},-1);}}
        if(stopFound)cv::rectangle(view,stopBox,{0,0,255},2);
        cv::putText(view,state+" "+(side>0?"RIGHT":side<0?"LEFT":"UNSELECTED")+" "+path,{5,122},cv::FONT_HERSHEY_SIMPLEX,.43,{0,255,255},1);
    }
};
