#pragma once
#include <opencv2/opencv.hpp>
#include <stdexcept>
struct ParkingPanel {
 int whiteV=190,whiteS=85,darkV=160,minArea=2000,minBottom=250,frames=3;
 int blobSize=12,blurKernel=7;double centerTolerance=.25;
 double seconds=.2;bool found=false;cv::Rect box,block,previous;int hits=0;double since=-1,last=-1;
 void load(const cv::FileStorage& f){
#define P(k,v) if(!f[k].empty())f[k]>>v
 P("parking_panel_white_v_min",whiteV);P("parking_panel_white_s_max",whiteS);P("parking_panel_dark_v_max",darkV);P("parking_panel_min_area",minArea);P("parking_panel_min_bottom",minBottom);P("parking_panel_confirm_frames",frames);P("parking_panel_confirm_seconds",seconds);
P("parking_panel_blob_min_size",blobSize);P("parking_panel_blur_kernel",blurKernel);P("parking_panel_center_tolerance",centerTolerance);
#undef P
 if(blobSize<6||blobSize>150||blurKernel<3||blurKernel>31||blurKernel%2==0||!std::isfinite(centerTolerance)||centerTolerance<.05||centerTolerance>.5||whiteV<1||whiteV>255||whiteS<0||whiteS>255||darkV<0||darkV>=whiteV||minArea<500||minArea>100000||minBottom<100||minBottom>479||frames<2||frames>30||!std::isfinite(seconds)||seconds<.05||seconds>3)throw std::runtime_error("Invalid parking panel parameters");
 }
 bool update(const cv::Mat& frame,double now,bool enabled){
  cv::Mat hsv,white;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);cv::inRange(hsv,cv::Scalar(0,0,whiteV),cv::Scalar(179,whiteS,255),white);
  cv::morphologyEx(white,white,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
  std::vector<std::vector<cv::Point>> contours;cv::findContours(white.clone(),contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);found=false;box={};block={};double best=0;
  for(auto& q:contours){auto r=cv::boundingRect(q);double ratio=double(r.width)/r.height;
   if(r.y<100||r.br().y<minBottom||r.width<35||r.height<40||r.width>250||r.height>300||ratio<.35||ratio>1.8||r.area()<minArea||cv::contourArea(q)<.5*r.area())continue;
   cv::Mat patch,blurred,dark;cv::extractChannel(hsv(r),patch,2);cv::GaussianBlur(patch,blurred,{blurKernel,blurKernel},0);cv::threshold(blurred,dark,darkV,255,cv::THRESH_BINARY_INV);
   dark.rowRange(0,5).setTo(0);dark.rowRange(r.height-5,r.height).setTo(0);dark.colRange(0,5).setTo(0);dark.colRange(r.width-5,r.width).setTo(0);
   // Blur and group dark pixels; no QR decoding, finder patterns, or transition-count requirement.
   cv::Mat grouped;cv::morphologyEx(dark,grouped,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{5,5}));
   std::vector<std::vector<cv::Point>> ds;cv::findContours(grouped,ds,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
   for(auto& d:ds){auto b=cv::boundingRect(d);double ar=double(b.width)/b.height;
    if(b.width<blobSize||b.height<blobSize||ar<.3||ar>3||b.area()<.01*r.area()||b.area()>.5*r.area())continue;
    double fill=double(cv::countNonZero(dark(b)))/b.area();if(fill<.25)continue;
    if(std::abs(r.x+b.x+b.width*.5-frame.cols*.5)>frame.cols*centerTolerance)continue;
    cv::Rect global(b.x+r.x,b.y+r.y,b.width,b.height),ring(global.x-4,global.y-4,global.width+8,global.height+8);
    double surround=double(cv::countNonZero(white(ring))-cv::countNonZero(white(global)))/(ring.area()-global.area());
    if(surround<.35)continue;if(r.area()>best){best=r.area();found=true;box=r;block=global;}
   }
  }
  if(!enabled||!found){hits=0;last=now;return false;}
  double overlap=previous.empty()?0:double((box&previous).area())/(box|previous).area();
  if(!hits||now-last>.3||overlap<.4){hits=1;since=now;}else ++hits;
  previous=box;last=now;return hits>=frames&&now-since+1e-9>=seconds;
 }
 void draw(cv::Mat& f)const{if(found){cv::rectangle(f,box,{255,255,255},2);cv::rectangle(f,block,{0,0,255},2);}}
};
