#pragma once
#include "lane_geometry.hpp"
#include <cmath>
// parking4 changes side selection only; shared parking3 owns following/stopping.
class Parking4Selector {
 LaneV2 cached; double seen=-1; int proposal=0,hits=0;
public:
 int choose(const LaneV2& lane,const cv::Rect& board,bool candidate,double now,int frames,double gap,double hold){
  if(lane.leftValid&&lane.rightValid){cached=lane;seen=now;}
  if(!candidate||seen<0||now-seen>hold){proposal=hits=0;return 0;}
  double y=std::max(double(cached.top),std::min(double(cached.bottom-1),double(board.br().y)));
  double left=cached.leftAt(y),right=cached.rightAt(y),x=board.x+board.width*.5;
  if(right-left<40||x<left||x>right){proposal=hits=0;return 0;}
  double delta=std::abs(x-right)-std::abs(x-left);
  int pick=delta>gap?1:delta< -gap?-1:0;
  if(pick&&pick==proposal)++hits;else{proposal=pick;hits=pick?1:0;}
  return hits>=frames?pick:0;
 }
};
