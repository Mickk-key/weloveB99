#pragma once
#include "lane_v2.hpp"
struct ParkingWhiteConfig {
 double minSupport=120,minCoverage=.55,supportRatio=.35;
 double centerMaxOffset=140,trackTolerance=60,holdSeconds=.3;int minGap=60;
 void load(cv::FileStorage& f){
#define W(k,v) if(!f[k].empty())f[k]>>v
 W("parking_white_center_max_offset",centerMaxOffset);W("parking_white_track_tolerance",trackTolerance);W("parking_white_hold_seconds",holdSeconds);W("parking_white_min_gap",minGap);
W("parking_white_min_support",minSupport);W("parking_white_min_coverage",minCoverage);W("parking_white_support_ratio",supportRatio);
#undef W
 if(!std::isfinite(minSupport)||minSupport<50||minSupport>10000||!std::isfinite(minCoverage)||minCoverage<.2||minCoverage>.95||!std::isfinite(supportRatio)||supportRatio<.1||supportRatio>1)throw std::runtime_error("Invalid white support thresholds");
 if(!std::isfinite(centerMaxOffset)||centerMaxOffset<20||centerMaxOffset>300||!std::isfinite(trackTolerance)||trackTolerance<10||trackTolerance>150||!std::isfinite(holdSeconds)||holdSeconds<0||holdSeconds>1||minGap<30||minGap>200)throw std::runtime_error("Invalid parking white pairing parameters");
 }
};
class ParkingWhitePair {
 struct Track{double a=0,b=0,score=0;double at(double y)const{return a*y+b;}};
 LaneConfig lane;ParkingWhiteConfig cfg;Track center,outer;double centerSeen=-1,outerSeen=-1;int lockedSide=0;
public:
 bool usable=false;
 bool leftMeasured=false,rightMeasured=false;
 std::string source="NONE";cv::Mat mask;
 ParkingWhitePair(LaneConfig l={},ParkingWhiteConfig c={}):lane(l),cfg(c){}
 LaneV2 detect(const cv::Mat& frame,int side,double now){
  if(side!=lockedSide){centerSeen=outerSeen=-1;lockedSide=side;}
  cv::Mat hsv;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);cv::inRange(hsv,cv::Scalar(0,0,lane.white_v_min),cv::Scalar(179,lane.white_s_max,255),mask);
  mask.rowRange(0,lane.lane_roi_top).setTo(0);mask.rowRange(lane.lane_roi_bottom,480).setTo(0);
  cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{3,3}));
  std::vector<cv::Vec4i> ls;cv::HoughLinesP(mask,ls,1,CV_PI/180,lane.lane_hough_votes,35,12);
  struct Segment{Track t;cv::Point2f p,q;};std::vector<Segment> seg;
  double yn=lane.lane_roi_bottom-25,yf=lane.lane_roi_top+30;
  for(auto l:ls){double dy=l[3]-l[1],dx=l[2]-l[0];if(std::abs(dy)<24||std::abs(dx/dy)>2)continue;Track t{dx/dy,l[0]-dx/dy*l[1],std::hypot(dx,dy)};if(t.at(yn)<-lane.lane_fit_margin_px||t.at(yn)>640+lane.lane_fit_margin_px)continue;seg.push_back({t,{float(l[0]),float(l[1])},{float(l[2]),float(l[3])}});}
  std::vector<Track> candidates;
  for(auto seed:seg){std::vector<cv::Point2f> pts;double score=0,ylo=480,yhi=0;
   for(auto s:seg)if(std::max(std::abs(s.t.at(yn)-seed.t.at(yn)),std::abs(s.t.at(yf)-seed.t.at(yf)))<=lane.lane_fit_tolerance){pts.push_back(s.p);pts.push_back(s.q);score+=s.t.score;ylo=std::min(ylo,double(std::min(s.p.y,s.q.y)));yhi=std::max(yhi,double(std::max(s.p.y,s.q.y)));}
   if(pts.size()<2||yhi-ylo<lane.lane_min_span)continue;cv::Vec4f v;cv::fitLine(pts,v,cv::DIST_HUBER,0,.01,.01);if(std::abs(v[1])<.1)continue;Track t{v[0]/v[1],v[2]-v[0]/v[1]*v[3],score};
   int rows=0,supported=0;for(int y=int(ylo);y<=int(yhi);y+=3){int x=std::lround(t.at(y));if(x<4||x>=636||y<0||y>=480)continue;++rows;supported+=cv::countNonZero(mask(cv::Rect(x-4,y,9,1)))>=3;}
   if(t.score<cfg.minSupport||rows<10||double(supported)/rows<cfg.minCoverage)continue;
   bool duplicate=false;
   for(auto& old:candidates)if(std::abs(t.at(yn)-old.at(yn))<25&&std::abs(t.at(yf)-old.at(yf))<25){if(t.score>old.score)old=t;duplicate=true;break;}if(!duplicate)candidates.push_back(t);
  }
  bool centerReal=false,outerReal=false;double best=1e9;Track chosen;
  bool recentCenter=centerSeen>=0&&now-centerSeen<=cfg.holdSeconds;
  for(auto t:candidates){double distance=recentCenter?std::max(std::abs(t.at(yn)-center.at(yn)),std::abs(t.at(yf)-center.at(yf))):std::abs(t.at(yn)-320);
   if(distance>(recentCenter?cfg.trackTolerance:cfg.centerMaxOffset))continue;
   // An inner boundary must leave room toward the selected side.
   if(side*t.at(yn)> (side>0?640-cfg.minGap:-cfg.minGap))continue;
   if(distance<best){best=distance;chosen=t;centerReal=true;}}
  if(centerReal){center=chosen;centerSeen=now;}
  Track centerUsed=(centerReal||recentCenter)?center:Track{0,320,0};
  best=1e9;bool recentOuter=outerSeen>=0&&now-outerSeen<=cfg.holdSeconds;
  std::vector<Track> outerCandidates;double maxSupport=0;
  for(auto t:candidates){double nw=side*(t.at(yn)-centerUsed.at(yn)),fw=side*(t.at(yf)-centerUsed.at(yf));
   if(nw<cfg.minGap||fw<20||nw>650||nw<fw*.65)continue;outerCandidates.push_back(t);maxSupport=std::max(maxSupport,t.score);}
  for(auto t:outerCandidates){if(t.score<maxSupport*cfg.supportRatio)continue;
   // Only compare proximity AFTER rejecting weak/reflection candidates.
   double nw=side*(t.at(yn)-centerUsed.at(yn));double continuity=recentOuter?std::min(100.,std::abs(t.at(yn)-outer.at(yn))):0;
   double score=nw+.15*continuity;if(score<best){best=score;chosen=t;outerReal=true;}}
  if(outerReal){outer=chosen;outerSeen=now;}
  Track outerUsed=(outerReal||recentOuter)?outer:Track{0,side>0?639.:0.,0};
  if(side*(outerUsed.at(yn)-centerUsed.at(yn))<cfg.minGap||side*(outerUsed.at(yf)-centerUsed.at(yf))<20||side*(outerUsed.at(yn)-centerUsed.at(yn))<side*(outerUsed.at(yf)-centerUsed.at(yf))*.65){outerUsed={0,side>0?639.:0.,0};outerSeen=-1;outerReal=false;recentOuter=false;}
  usable=true;
  source=std::string(side>0?"RIGHT_":"LEFT_")+(centerReal?"CENTER_WHITE":recentCenter?"CENTER_HELD":"CENTER_ESTIMATE")+(outerReal?"_OUTER_WHITE":recentOuter?"_OUTER_HELD":"_OUTER_EDGE");
  leftMeasured=side>0?centerReal:outerReal;rightMeasured=side>0?outerReal:centerReal;
  LaneV2 result;result.top=lane.lane_roi_top;result.bottom=lane.lane_roi_bottom;
  // Geometry is usable even for an explicitly labelled held/estimated boundary.
  result.leftValid=result.rightValid=usable;auto l=side>0?centerUsed:outerUsed,r=side>0?outerUsed:centerUsed;result.la=l.a;result.lb=l.b;result.ra=r.a;result.rb=r.b;result.error=usable?result.targetError():0;return result;
 }
 void draw(cv::Mat& frame,const LaneV2& g)const{
  if(!usable){cv::putText(frame,source+" NO VALID PAIR",{5,74},cv::FONT_HERSHEY_SIMPLEX,.4,{0,0,255},1);return;}
  for(int y=g.top;y<g.bottom-5;y+=5){
   if(leftMeasured||((y-g.top)/5)%3!=0)cv::line(frame,{int(g.leftAt(y)),y},{int(g.leftAt(y+5)),y+5},leftMeasured?cv::Scalar(0,255,0):cv::Scalar(0,180,255),2);
   if(rightMeasured||((y-g.top)/5)%3!=0)cv::line(frame,{int(g.rightAt(y)),y},{int(g.rightAt(y+5)),y+5},rightMeasured?cv::Scalar(0,255,0):cv::Scalar(0,180,255),2);
   cv::circle(frame,{int((g.leftAt(y)+g.rightAt(y))*.5),y},2,{255,0,255},-1);
  }
  cv::putText(frame,source,{5,74},cv::FONT_HERSHEY_SIMPLEX,.4,{0,255,255},1);
 }

};
