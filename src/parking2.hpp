#pragma once
#include "blue_board.hpp"
#include "parking_panel.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// 独立于原 parking 的黄横线停车路径。图像坐标均以 640x480 为准。
struct Parking2Config {
    int hmin=18,hmax=40,smin=60,vmin=100,roiTop=180,roiBottom=470;
    int minLength=65,houghVotes=25,maxGap=18,sideFrames=2,sideGap=25;
    double maxSlope=.25,kp=.4,kd=.5,steerLimit=15.,steerRate=40.;
    int forwardPwm=10600;
    int blueHMin=100,blueHMax=130,blueSMin=60,blueVMin=55;
    int blueMinPixels=2500,blueMinHeight=100,blueConfirmFrames=2;
    int blueLeaveFrames=2;
    int whiteSMax=70,whiteVMin=150,whiteMinLength=65,whiteMinGap=25;
    double whiteMaxSlope=1.6;
    int laneYellowHMin=10,laneYellowHMax=34,laneYellowSMin=35,laneYellowVMin=125;
    int laneYellowMinLength=65,laneMinGap=60,laneMaxGap=550;
    double laneYellowMaxSlope=1.6;
    int stopWhiteV=230,stopWhiteS=120,stopMinArea=2500,stopMinBottom=200,stopFrames=3;
    double stopMinFill=.65,stopSeconds=.2,stopCenterTolerance=.3;
    int stopDimV=90,stopDimS=90,stopDarkV=100;
    double stopDarkFraction=.6;
    ParkingPanel panel;
    void load(const cv::FileStorage& f){
        panel.load(f); // 白板使用现有停车白板检测器及 parking_panel_* 标定。
#define P2(k,v) if(!f[k].empty())f[k]>>v
        P2("parking2_yellow_h_min",hmin);P2("parking2_yellow_h_max",hmax);
        P2("parking2_yellow_s_min",smin);P2("parking2_yellow_v_min",vmin);
        P2("parking2_roi_top",roiTop);P2("parking2_roi_bottom",roiBottom);
        P2("parking2_min_length",minLength);P2("parking2_hough_votes",houghVotes);
        P2("parking2_max_gap",maxGap);P2("parking2_max_slope",maxSlope);
        P2("parking2_side_frames",sideFrames);P2("parking2_side_gap_px",sideGap);
        P2("parking2_follow_pwm",forwardPwm);P2("parking2_kp",kp);P2("parking2_kd",kd);
        P2("parking2_steer_limit",steerLimit);P2("parking2_steer_rate",steerRate);
        P2("parking2_blue_h_min",blueHMin);P2("parking2_blue_h_max",blueHMax);
        P2("parking2_blue_s_min",blueSMin);P2("parking2_blue_v_min",blueVMin);
        P2("parking2_blue_min_pixels",blueMinPixels);P2("parking2_blue_min_height",blueMinHeight);
        P2("parking2_blue_confirm_frames",blueConfirmFrames);
        P2("parking2_blue_leave_frames",blueLeaveFrames);
        P2("parking2_white_s_max",whiteSMax);P2("parking2_white_v_min",whiteVMin);
        P2("parking2_white_min_length",whiteMinLength);P2("parking2_white_min_gap",whiteMinGap);
        P2("parking2_white_max_slope",whiteMaxSlope);
        P2("parking2_lane_yellow_h_min",laneYellowHMin);P2("parking2_lane_yellow_h_max",laneYellowHMax);
        P2("parking2_lane_yellow_s_min",laneYellowSMin);P2("parking2_lane_yellow_v_min",laneYellowVMin);
        P2("parking2_lane_yellow_min_length",laneYellowMinLength);
        P2("parking2_lane_yellow_max_slope",laneYellowMaxSlope);
        P2("parking2_lane_min_gap",laneMinGap);P2("parking2_lane_max_gap",laneMaxGap);
        P2("parking2_stop_white_v_min",stopWhiteV);P2("parking2_stop_white_s_max",stopWhiteS);
        P2("parking2_stop_min_area",stopMinArea);P2("parking2_stop_min_bottom",stopMinBottom);
        P2("parking2_stop_min_fill",stopMinFill);P2("parking2_stop_confirm_frames",stopFrames);
        P2("parking2_stop_confirm_seconds",stopSeconds);
        P2("parking2_stop_center_tolerance",stopCenterTolerance);
        P2("parking2_stop_dim_v_min",stopDimV);P2("parking2_stop_dim_s_max",stopDimS);
        P2("parking2_stop_dark_v_max",stopDarkV);P2("parking2_stop_dark_ring_fraction",stopDarkFraction);
#undef P2
        if(hmin<0||hmax>179||hmin>hmax||smin<0||smin>255||vmin<0||vmin>255||
           roiTop<0||roiBottom>480||roiBottom-roiTop<80||minLength<20||minLength>640||
           houghVotes<2||houghVotes>300||maxGap<0||maxGap>100||sideFrames<1||sideFrames>30||
           sideGap<0||sideGap>200||!std::isfinite(maxSlope)||maxSlope<0||maxSlope>.8||
           forwardPwm<=10000||forwardPwm>12000||!std::isfinite(kp)||kp<0||kp>2||
           !std::isfinite(kd)||kd<0||kd>2||!std::isfinite(steerLimit)||steerLimit<=0||steerLimit>15||
           !std::isfinite(steerRate)||steerRate<=0||steerRate>100||
           blueHMin<0||blueHMax>179||blueHMin>blueHMax||blueSMin<0||blueSMin>255||
           blueVMin<0||blueVMin>255||blueMinPixels<100||blueMinHeight<20||
           blueConfirmFrames<1||blueConfirmFrames>30||blueLeaveFrames<1||blueLeaveFrames>30||
           whiteSMax<0||whiteSMax>255||
           whiteVMin<0||whiteVMin>255||whiteMinLength<20||whiteMinLength>400||
           whiteMinGap<10||whiteMinGap>200||!std::isfinite(whiteMaxSlope)||
           whiteMaxSlope<0||whiteMaxSlope>3||laneYellowHMin<0||laneYellowHMax>179||
           laneYellowHMin>laneYellowHMax||laneYellowSMin<0||laneYellowSMin>255||
           laneYellowVMin<0||laneYellowVMin>255||laneYellowMinLength<20||
           laneYellowMinLength>400||!std::isfinite(laneYellowMaxSlope)||
           laneYellowMaxSlope<0||laneYellowMaxSlope>3||laneMinGap<20||
           laneMaxGap>640||laneMaxGap<=laneMinGap||stopWhiteV<1||stopWhiteV>255||
           stopWhiteS<0||stopWhiteS>255||stopMinArea<500||stopMinArea>50000||
           stopMinBottom<100||stopMinBottom>479||!std::isfinite(stopMinFill)||
           stopMinFill<.3||stopMinFill>.95||stopFrames<2||stopFrames>30||
           !std::isfinite(stopSeconds)||stopSeconds<.05||stopSeconds>3||
           !std::isfinite(stopCenterTolerance)||stopCenterTolerance<.05||stopCenterTolerance>.5||
           stopDimV<1||stopDimV>255||stopDimS<0||stopDimS>255||
           stopDarkV<0||stopDarkV>255||!std::isfinite(stopDarkFraction)||
           stopDarkFraction<0||stopDarkFraction>1)
            throw std::runtime_error("Invalid parking2 configuration");
    }
};

class Parking2 {
    Parking2Config c;
    int proposal=0,hits=0,blueHits=0,blueMissing=0,stopHits=0;
    double previousError=0,boardCenter=320.,stopSince=-1,stopLast=-1;
    bool previousValid=false,leftBluePhase=false,stopFound=false;
    cv::Rect stopBox,stopPrevious;
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
                if(r.width<35||r.width>250||r.height<40||r.height>300||
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
    double blueEdge=0,whiteNear=0,whiteFar=0,whiteA=0,whiteB=0;
    double yellowNear=0,yellowFar=0,yellowA=0,yellowB=0;
    cv::Vec4i segment{0,0,0,0};cv::Mat mask;
    std::string state="PARK2_WAIT_BOARD",path="NONE";
    explicit Parking2(Parking2Config cfg={}):c(cfg){}
    void beginYellowWhite(int selectedSide){side=selectedSide;leftBluePhase=true;}
    double rate()const{return c.steerRate;}
    int motor()const{return c.forwardPwm;}
    bool engaged()const{return side!=0||state=="PARK2_SEARCH_LINE"||stopped;}
    bool panelFound()const{return stopFound;}
    int panelHits()const{return stopHits;}
    void update(const cv::Mat& frame,const BlueBoardStop& board,double now,bool canStart){
        if(stopped){state="PARK2_PANEL_STOP";path="STOP";return;}
        cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        // 旧检测器的 ROI 会漏掉贴着画面边缘的蓝挡板；只在本模块内补检大面积竖直蓝块。
        cv::Mat blue;cv::inRange(hsv,cv::Scalar(c.blueHMin,c.blueSMin,c.blueVMin),
                                 cv::Scalar(c.blueHMax,255,255),blue);
        cv::Mat labels,stats,centers;int n=cv::connectedComponentsWithStats(blue,labels,stats,centers);
        int largest=0,blueX=-1,blueLeft=-1,blueRight=-1;
        for(int i=1;i<n;++i){int area=stats.at<int>(i,cv::CC_STAT_AREA);
            int height=stats.at<int>(i,cv::CC_STAT_HEIGHT);
            if(area>=c.blueMinPixels&&height>=c.blueMinHeight&&area>largest){
                largest=area;blueLeft=stats.at<int>(i,cv::CC_STAT_LEFT);
                blueRight=blueLeft+stats.at<int>(i,cv::CC_STAT_WIDTH)-1;
                blueX=(blueLeft+blueRight)/2;}}
        blueHits=blueX>=0?std::min(blueHits+1,c.blueConfirmFrames):0;
        bool blueConfirmed=blueHits>=c.blueConfirmFrames;
        if(board.candidate)boardCenter=board.box.x+board.box.width*.5;
        else if(blueConfirmed)boardCenter=blueX;
        bool blueVisible=board.candidate||blueConfirmed;
        if(blueVisible)blueEdge=board.candidate?
            (side<0?board.box.x:board.box.x+board.box.width-1):
            (side<0?blueLeft:blueRight);
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
        if(!side&&!board.stopped&&!blueConfirmed){state="PARK2_WAIT_BOARD";path="NONE";return;}
        if(!side){
            int candidate=line?(midX>boardCenter+c.sideGap?1:midX<boardCenter-c.sideGap?-1:0):0;
            if(candidate && candidate==proposal)++hits;else{proposal=candidate;hits=candidate?1:0;}
            if(hits>=c.sideFrames)side=candidate;
            if(!side){useLane=true;state="PARK2_SEARCH_LINE";path="SEARCH_WITH_LANE";
                error=correction=0;previousValid=false;return;}
        }
        if(detectStopRectangle(hsv,now,canStart)){stopped=true;state="PARK2_PANEL_STOP";path="STOP";error=correction=0;return;}
        useLane=true;error=correction=0;
        if(!leftBluePhase){
            blueMissing=blueVisible?0:std::min(blueMissing+1,c.blueLeaveFrames);
            if(blueMissing>=c.blueLeaveFrames)leftBluePhase=true;
        }
        const bool yellowWhite=leftBluePhase;
        if(!yellowWhite&&!blueVisible){previousValid=false;state="PARK2_FOLLOW_LANE";
            path="LANE_WAIT_BLUE_EXIT";return;}
        const int nearY=std::min(c.roiBottom-20,410),farY=std::max(c.roiTop+20,230);
        double boundaryNear=0,boundaryFar=0;
        if(yellowWhite){
            // 蓝板离开视野后，选中侧的黄纵线成为内侧边界；黄线消失时由普通巡线接管。
            cv::Mat yellow;cv::inRange(hsv,cv::Scalar(c.laneYellowHMin,c.laneYellowSMin,c.laneYellowVMin),
                cv::Scalar(c.laneYellowHMax,255,255),yellow);
            yellow.rowRange(0,c.roiTop).setTo(0);yellow.rowRange(c.roiBottom,frame.rows).setTo(0);
            cv::morphologyEx(yellow,yellow,cv::MORPH_OPEN,
                cv::getStructuringElement(cv::MORPH_RECT,{3,5}));
            std::vector<cv::Vec4i> ys;cv::HoughLinesP(yellow,ys,1,CV_PI/180,20,c.laneYellowMinLength,18);
            bool foundYellow=false;double bestYellow=side>0?1e9:-1e9;
            for(auto s:ys){double dy=s[3]-s[1],dx=s[2]-s[0];
                if(std::abs(dy)<c.laneYellowMinLength||std::abs(dx/dy)>c.laneYellowMaxSlope)continue;
                double a=dx/dy,b=s[0]-a*s[1],xn=a*nearY+b,xf=a*farY+b;
                if(xn<0||xn>=frame.cols||xf<0||xf>=frame.cols)continue;
                if((side>0&&xn<bestYellow)||(side<0&&xn>bestYellow)){
                    bestYellow=xn;yellowNear=xn;yellowFar=xf;yellowA=a;yellowB=b;foundYellow=true;}}
            if(!foundYellow){previousValid=false;state="PARK2_FOLLOW_LANE";path="LANE_NO_YELLOW";return;}
            boundaryNear=yellowNear;boundaryFar=yellowFar;
        }else{
            // 挡板仍在画面时，沿蓝板外缘和同侧白线行驶。
            blueEdge=board.candidate?(side>0?board.box.x+board.box.width-1:board.box.x):
                                      (side>0?blueRight:blueLeft);
            boundaryNear=boundaryFar=blueEdge;
        }
        cv::Mat white;cv::inRange(hsv,cv::Scalar(0,0,c.whiteVMin),cv::Scalar(179,c.whiteSMax,255),white);
        white.rowRange(0,c.roiTop).setTo(0);white.rowRange(c.roiBottom,frame.rows).setTo(0);
        std::vector<cv::Vec4i> whites;cv::HoughLinesP(white,whites,1,CV_PI/180,25,c.whiteMinLength,18);
        bool found=false;double bestWhite=side>0?-1e9:1e9;
        for(auto s:whites){double dx=s[2]-s[0],dy=s[3]-s[1];
            if(std::abs(dy)<c.whiteMinLength||std::abs(dx/dy)>c.whiteMaxSlope)continue;
            double a=dx/dy,b=s[0]-a*s[1],xn=a*nearY+b,xf=a*farY+b;
            double gapNear=side*(xn-boundaryNear),gapFar=side*(xf-boundaryFar);
            if(xn<0||xn>=frame.cols||xf<0||xf>=frame.cols||
               gapNear<(yellowWhite?c.laneMinGap:c.whiteMinGap)||
               gapNear>c.laneMaxGap||gapFar<(yellowWhite?-c.laneMinGap*.5:c.whiteMinGap))continue;
            if((side>0&&xn>bestWhite)||(side<0&&xn<bestWhite)){
                bestWhite=xn;whiteNear=xn;whiteFar=xf;whiteA=a;whiteB=b;found=true;}}
        if(!found){previousValid=false;state="PARK2_FOLLOW_LANE";path="LANE_NO_WHITE";return;}
        double targetNear=(boundaryNear+whiteNear)*.5,targetFar=(boundaryFar+whiteFar)*.5;
        error=(targetNear-frame.cols*.5)/8.;
        // 与普通巡线一致：P×当前误差 + D×相邻帧误差差值，再限幅。
        correction=std::max(-c.steerLimit,std::min(c.steerLimit,
            c.kp*error+(previousValid?c.kd*(error-previousError):0.)));
        previousError=error;previousValid=true;
        useLane=false;state=yellowWhite?"PARK2_FOLLOW_YELLOW_WHITE":"PARK2_FOLLOW_BLUE_WHITE";
        path=yellowWhite?"YELLOW_WHITE":"BLUE_EDGE_WHITE";
    }
    void draw(cv::Mat& view)const{
        if(line){cv::line(view,{segment[0],segment[1]},{segment[2],segment[3]},{0,255,255},2);
            if(!side)cv::line(view,{int(std::lround(midX)),0},{int(std::lround(midX)),view.rows-1},{255,0,255},2);}
        if(side&&(state=="PARK2_FOLLOW_BLUE_WHITE"||state=="PARK2_FOLLOW_YELLOW_WHITE")){
            int farY=std::max(c.roiTop+20,230),nearY=std::min(c.roiBottom-20,410);
            if(state=="PARK2_FOLLOW_BLUE_WHITE")
                cv::line(view,{int(blueEdge),farY},{int(blueEdge),nearY},{255,0,0},2);
            else cv::line(view,{int(yellowA*farY+yellowB),farY},
                          {int(yellowA*nearY+yellowB),nearY},{0,255,255},2);
            cv::line(view,{int(whiteA*farY+whiteB),farY},{int(whiteA*nearY+whiteB),nearY},{255,255,255},2);
            // 紫色点线就是本帧控制使用的两侧边界中线。
            for(int y=farY;y<=nearY;y+=8){double left=state=="PARK2_FOLLOW_BLUE_WHITE"?blueEdge:yellowA*y+yellowB;
                int x=int(std::lround((left+whiteA*y+whiteB)*.5));
                cv::circle(view,{x,y},3,{255,0,255},-1);}}
        if(stopFound)cv::rectangle(view,stopBox,{0,0,255},2);
        cv::putText(view,state+" side="+std::to_string(side),{5,122},cv::FONT_HERSHEY_SIMPLEX,.48,{0,255,255},1);
    }
};
