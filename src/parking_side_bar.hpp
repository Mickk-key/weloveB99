#pragma once
#include <opencv2/opencv.hpp>
#include <stdexcept>
struct YellowSideBar {
 int enabled=0,minWidth=80,nearY=45,frames=2;double maxSlope=.18,deadband=.04;
 int hits=0,proposal=0;double last=-1;cv::Vec4i bar={0,0,0,0};bool found=false;
 void load(const cv::FileStorage& f){
#define B(k,v) if(!f[k].empty())f[k]>>v
 B("parking_side_yellow_bar",enabled);B("parking_side_bar_min_width",minWidth);B("parking_side_bar_near_y",nearY);B("parking_side_bar_frames",frames);B("parking_side_bar_max_slope",maxSlope);B("parking_side_bar_deadband",deadband);
#undef B
 if((enabled!=0&&enabled!=1)||minWidth<40||minWidth>600||nearY<10||nearY>150||frames<1||frames>10||!std::isfinite(maxSlope)||maxSlope<.03||maxSlope>.4||!std::isfinite(deadband)||deadband<0||deadband>.2)throw std::runtime_error("Invalid yellow bar side selection parameters");
 }
 int select(const cv::Mat& frame,const cv::Rect& board,bool candidate,bool confirmed,double now,int hmin,int hmax,int smin,int vmin){
  double gap=last<0?0:now-last;last=now;found=false;
  if(!candidate){hits=0;proposal=0;return 0;}
  cv::Mat hsv,mask;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);cv::inRange(hsv,cv::Scalar(hmin,smin,vmin),cv::Scalar(hmax,255,255),mask);
  int top=std::max(0,board.br().y-nearY),bottom=std::min(480,board.br().y+nearY+1);mask.rowRange(0,top).setTo(0);mask.rowRange(bottom,480).setTo(0);
  std::vector<cv::Vec4i> ls;cv::HoughLinesP(mask,ls,1,CV_PI/180,25,minWidth,12);double best=-1;double bx=board.x+board.width*.5;
  for(auto l:ls){if(l[0]>l[2]){std::swap(l[0],l[2]);std::swap(l[1],l[3]);}double width=l[2]-l[0];if(width<minWidth||std::abs(l[3]-l[1])>width*maxSlope||bx<l[0]||bx>l[2])continue;
   double atBoard=l[1]+(bx-l[0])*(l[3]-l[1])/width,dist=std::abs(atBoard-board.br().y);if(dist>nearY)continue;
   double score=width-2*dist;if(score>best){best=score;bar=l;found=true;}
  }
  if(!found){hits=0;proposal=0;return 0;}
  double mid=(bar[0]+bar[2])*.5,margin=std::max(5.,(bar[2]-bar[0])*deadband);int p=bx<mid-margin?1:bx>mid+margin?-1:0;
  hits=p&&p==proposal&&gap<=.25?hits+1:1;proposal=p;
  return p&&confirmed&&hits>=frames?p:0;
 }
 void draw(cv::Mat& im)const{if(found){cv::line(im,{bar[0],bar[1]},{bar[2],bar[3]},{255,0,255},3);cv::circle(im,{(bar[0]+bar[2])/2,(bar[1]+bar[3])/2},5,{0,0,255},-1);}}
};
