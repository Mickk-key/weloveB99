#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>
struct Parking4YellowConfig {
 int enabled=1,hmin=15,hmax=40,smin=60,vmin=100,top=180,bottom=470,hLength=80,vLength=60,gap=25,frames=2;
 int paleEnabled=1,paleV=220,paleS=50;
 double seconds=.1;
 void load(const cv::FileStorage& f){
#define Y(k,v) if(!f[k].empty())f[k]>>v
 Y("parking4_yellow_stop_enabled",enabled);Y("parking4_stop_yellow_h_min",hmin);Y("parking4_stop_yellow_h_max",hmax);Y("parking4_stop_yellow_s_min",smin);Y("parking4_stop_yellow_v_min",vmin);Y("parking4_yellow_stop_roi_top",top);Y("parking4_yellow_stop_roi_bottom",bottom);Y("parking4_yellow_stop_horizontal_px",hLength);Y("parking4_yellow_stop_vertical_px",vLength);Y("parking4_yellow_stop_join_px",gap);Y("parking4_yellow_stop_frames",frames);Y("parking4_yellow_stop_seconds",seconds);
Y("parking4_yellow_pale_bar_enabled",paleEnabled);Y("parking4_yellow_pale_bar_v_min",paleV);Y("parking4_yellow_pale_bar_s_max",paleS);
#undef Y
 if((paleEnabled!=0&&paleEnabled!=1)||paleV<0||paleV>255||paleS<0||paleS>255)throw std::runtime_error("Invalid pale yellow bar thresholds");
 if((enabled!=0&&enabled!=1)||hmin<0||hmax>179||hmin>hmax||smin<0||smin>255||vmin<0||vmin>255||top<0||bottom>480||bottom-top<80||hLength<30||hLength>640||vLength<30||vLength>480||gap<3||gap>80||frames<1||frames>30||!std::isfinite(seconds)||seconds<0||seconds>3)throw std::runtime_error("Invalid parking4 yellow stop config");
 }
};
class Parking4YellowStop {
 int hits=0;double since=-1,last=-1;cv::Point2d previous;
public:
 bool candidate=false;cv::Point2d join;
 bool update(const cv::Mat& hsv,double now,const Parking4YellowConfig& c){
 candidate=false;if(!c.enabled){hits=0;return false;}
 cv::Mat mask;cv::inRange(hsv,cv::Scalar(c.hmin,c.smin,c.vmin),cv::Scalar(c.hmax,255,255),mask);mask.rowRange(0,c.top).setTo(0);mask.rowRange(c.bottom,mask.rows).setTo(0);
 std::vector<cv::Vec4i> lines;cv::HoughLinesP(mask,lines,1,CV_PI/180,20,std::min(c.hLength,c.vLength),15);
 cv::Mat horizontalMask=mask.clone();if(c.paleEnabled){cv::Mat pale;cv::inRange(hsv,cv::Scalar(0,0,c.paleV),cv::Scalar(179,c.paleS,255),pale);pale.rowRange(0,c.top).setTo(0);pale.rowRange(c.bottom,pale.rows).setTo(0);horizontalMask|=pale;}
 std::vector<cv::Vec4i> bars;cv::HoughLinesP(horizontalMask,bars,1,CV_PI/180,20,c.hLength,15);
 double best=-1;
 for(auto h:bars){double hx=h[2]-h[0],hy=h[3]-h[1];if(std::abs(hx)<c.hLength||std::abs(hy/hx)>.3)continue;double ha=hy/hx,hb=h[1]-ha*h[0];
 for(auto v:lines){double vy=v[3]-v[1],vx=v[2]-v[0];if(std::abs(vy)<c.vLength||std::abs(vx/vy)>.8)continue;double va=vx/vy,vb=v[0]-va*v[1];double y=(ha*vb+hb)/(1-ha*va),x=va*y+vb;
 // Horizontal bar meets the FAR/top end of the longitudinal stem; reject near/bottom bar and crosses.
 if(std::abs(y-std::min(v[1],v[3]))>c.gap||std::max(v[1],v[3])-y<c.vLength*.7||x<std::min(h[0],h[2])+15||x>std::max(h[0],h[2])-15)continue;
 double score=std::abs(hx)+std::abs(vy);if(score>best){best=score;join={x,y};candidate=true;}}
 }
 if(!candidate){hits=0;last=now;return false;}
 if(!hits||now-last>.5||cv::norm(join-previous)>60){hits=1;since=now;}else ++hits;
 previous=join;last=now;return hits>=c.frames&&now-since+1e-9>=c.seconds;
 }
};
