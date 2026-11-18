#pragma once
#include "lane_v2.hpp"
#include "parking_panel.hpp"
#include "parking_side_bar.hpp"
#include <stdexcept>
struct YellowParkingConfig {
 YellowSideBar sideBar;ParkingPanel panel;
 int manualEnabled=0,manualSide=1;
 double entrySeconds=0,entrySteer=5;int entryPwm=10500;
 int hmin=15,hmax=40,smin=35,vmin=80,top=190,bottom=470,minSpan=55,motor=10700;
 double kp=.2,kd=.08,limit=6,rate=12;
 void load(const cv::FileStorage& f){sideBar.load(f);panel.load(f);
#define Y(k,v) if(!f[k].empty())f[k]>>v
 Y("parking_manual_side_enabled",manualEnabled);Y("parking_manual_side",manualSide);
 Y("parking_yellow_h_min",hmin);Y("parking_yellow_h_max",hmax);Y("parking_yellow_s_min",smin);Y("parking_yellow_v_min",vmin);
 Y("parking_yellow_roi_top",top);Y("parking_yellow_roi_bottom",bottom);Y("parking_yellow_min_span",minSpan);
 Y("parking_yellow_pwm",motor);Y("parking_yellow_kp",kp);Y("parking_yellow_kd",kd);Y("parking_yellow_steer_limit",limit);Y("parking_yellow_steer_rate",rate);
 Y("parking_yellow_entry_seconds",entrySeconds);Y("parking_yellow_entry_steer",entrySteer);Y("parking_yellow_entry_pwm",entryPwm);
#undef Y
 if((manualEnabled!=0&&manualEnabled!=1)||(manualSide!=1&&manualSide!=-1))throw std::runtime_error("Invalid manual side: enabled 0/1, direction -1/1");
 if(!std::isfinite(entrySeconds)||entrySeconds<0||entrySeconds>3||!std::isfinite(entrySteer)||entrySteer<0||entrySteer>15||entryPwm<=10000||entryPwm>12000)throw std::runtime_error("Invalid parking entry turn parameters");
 if(hmin<0||hmax>179||hmin>hmax||smin<0||smin>255||vmin<0||vmin>255||top<0||bottom>480||bottom-top<100||minSpan<20||minSpan>bottom-top||motor<=10000||motor>12000||!std::isfinite(kp)||kp<0||kp>2||!std::isfinite(kd)||kd<0||kd>2||!std::isfinite(limit)||limit<=0||limit>15||!std::isfinite(rate)||rate<=0||rate>100)throw std::runtime_error("Invalid parking path parameters");
 }
};
class YellowParking {
 YellowParkingConfig c;double previousError=0,last=-1,brakeUntil=-1;bool pdReady=false;
 double entryElapsed=0;bool entryApplied=false;int drawSide=0;
 bool detectLine(const cv::Mat& frame,const LaneV2& white,int side){
 cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);cv::inRange(hsv,cv::Scalar(c.hmin,c.smin,c.vmin),cv::Scalar(c.hmax,255,255),mask);
 mask.rowRange(0,c.top).setTo(0);mask.rowRange(c.bottom,480).setTo(0);
  std::vector<cv::Vec4i> segments;cv::HoughLinesP(mask,segments,1,CV_PI/180,20,c.minSpan,12);
  double best=0;line=false;boundaries=false;
  for(auto l:segments){double dy=l[3]-l[1],dx=l[2]-l[0];if(std::abs(dy)<c.minSpan||std::abs(dx/dy)>.8)continue;double aa=dx/dy,bb=l[0]-aa*l[1];double x=aa*(c.top+c.bottom)*.5+bb;if(x<40||x>600)continue;
   if(side&&(side>0?white.rightValid:white.leftValid)){
    bool inside=true;for(double part:{.45,.6,.75}){double yy=c.top+(c.bottom-c.top)*part;double outer=side>0?white.rightAt(yy):white.leftAt(yy);double width=side*(outer-(aa*yy+bb));if(width<60||width>600)inside=false;}if(!inside)continue;
   }
   if(std::abs(dy)>best){best=std::abs(dy);a=aa;b=bb;line=true;}}
  if(!line){line=false;return false;}
  // Refit stripe CENTERS rather than either paint edge; omit wide transverse bars.
  std::vector<cv::Point2f> centers;
  for(int y=c.top;y<c.bottom;y+=3){int x=std::lround(a*y+b);
   bool crossbar=false;for(auto h:segments){double dx=h[2]-h[0];if(std::abs(dx)<60||std::abs(h[3]-h[1])>.2*std::abs(dx))continue;if(x<std::min(h[0],h[2])-4||x>std::max(h[0],h[2])+4)continue;double yy=h[1]+(x-h[0])*(h[3]-h[1])/dx;if(std::abs(y-yy)<=5){crossbar=true;break;}}if(crossbar)continue;
if(x<2||x>637)continue;int seed=-1;for(int d=0;d<=25&&seed<0;++d)for(int sign:{-1,1}){int xx=x+sign*d;if(xx>0&&xx<639&&mask.at<uchar>(y,xx)){seed=xx;break;}}if(seed<0)continue;int l=seed,r=seed;while(l>0&&mask.at<uchar>(y,l-1))--l;while(r<639&&mask.at<uchar>(y,r+1))++r;if(r-l>=3&&r-l<=70)centers.push_back({float((l+r)*.5),float(y)});}
  if(centers.size()<8){line=false;return false;}
  cv::Vec4f fit;cv::fitLine(centers,fit,cv::DIST_HUBER,0,.01,.01);if(std::abs(fit[1])<.1){line=false;return false;}a=fit[0]/fit[1];b=fit[2]-a*fit[3];

 return true;
 }
public:
 cv::Mat mask;bool line=false,boundaries=false,stopped=false;double a=0,b=320,error=0,correction=0;
 std::string state="PARK_FOLLOW",pathSource="NONE";
 explicit YellowParking(YellowParkingConfig config={}):c(config){}
 void entryFeedback(bool applied){entryApplied=applied;}
 double entryRemaining()const{return std::max(0.,c.entrySeconds-entryElapsed);}
 int entryMotor()const{return c.entryPwm;}
 int selectBarSide(const cv::Mat& frame,const cv::Rect& board,bool candidate,bool confirmed,double now){return c.sideBar.select(frame,board,candidate,confirmed,now,c.hmin,c.hmax,c.smin,c.vmin);}
 void drawSideBar(cv::Mat& im)const{c.sideBar.draw(im);}
 bool panelActive()const{return brakeUntil>=0||stopped;}
 bool panelFound()const{return c.panel.found;}int panelHits()const{return c.panel.hits;}
 int motor()const{return c.motor;}double steerRate()const{return c.rate;}double steerLimit()const{return c.limit;}
 double remaining(double now)const{return std::max(0.,brakeUntil-now);}
 void update(const cv::Mat& frame,const LaneV2& white,int side,double now,bool canStart,bool wasMoving,double brakeSeconds){
  double gap=last<0?0:now-last;last=now;drawSide=side;
  if(stopped){state="PARK_PANEL_STOP";return;}
  if(brakeUntil>=0){if(now>=brakeUntil){stopped=true;state="PARK_PANEL_STOP";}else state="PARK_PANEL_BRAKE";return;}
  if(c.panel.update(frame,now,canStart)){brakeUntil=now+(wasMoving?brakeSeconds:0);if(brakeUntil<=now){stopped=true;state="PARK_PANEL_STOP";}else state="PARK_PANEL_BRAKE";return;}
  if(!side){state="PARK_SEARCH_PANEL";pathSource="NONE";return;}
  if(side&&entryElapsed<c.entrySeconds){
   if(canStart&&entryApplied&&gap>0&&gap<=.25)entryElapsed=std::min(c.entrySeconds,entryElapsed+gap);
   if(entryElapsed+1e-9<c.entrySeconds){correction=side*c.entrySteer;pdReady=false;state="PARK_ENTRY_TURN";pathSource="ENTRY_TURN";return;}
  }
  // Steering in PARK_FOLLOW is owned by the existing lane backend and its PD.
  line=false;boundaries=false;error=correction=0;pathSource="WHITE_LANE";state="PARK_FOLLOW";

 }
 void draw(cv::Mat& im)const{c.panel.draw(im);if(drawSide&&state!="PARK_FOLLOW"){cv::line(im,{int(a*c.top+b),c.top},{int(a*c.bottom+b),c.bottom},line?cv::Scalar(0,255,255):cv::Scalar(255,0,255),2);cv::putText(im,pathSource,{5,130},cv::FONT_HERSHEY_SIMPLEX,.5,{255,255,0},1);}}
};
