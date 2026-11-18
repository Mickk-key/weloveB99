#pragma once
#include <opencv2/opencv.hpp>
// V2原颜色/亮芯算法：仅将类型重命名，保持检测实现和参数读取一致。
// x/y/w/h 是整图宽高百分比；rs/rv 和 gs/gv 是红/绿 HSV 最小饱和度与亮度。
// red/green 保存通过本算法筛选的像素数，用于状态日志和现场观察。
struct NightLightDetector {
 int x=8,y=25,w=84,h=40,rs=60,rv=230,gs=30,gv=200,rmin=100,gmin=12;
 int red=0,green=0;cv::Rect roi;
 void load(const std::string& path){cv::FileStorage f(path,cv::FileStorage::READ);if(!f.isOpened())return;
 #define READ(k,v) if(!f[k].empty())f[k]>>v
 READ("light_roi_x",x);READ("light_roi_y",y);READ("light_roi_w",w);READ("light_roi_h",h);
 READ("light_s_min",rs);READ("red_v_min",rv);READ("green_s_min",gs);READ("green_v_min",gv);
 READ("light_min_pixels",rmin);READ("green_min_pixels",gmin);
 #undef READ
 }
 // 先按百分比裁出灯区并与图像边界取交集，再在该区域中做颜色和高亮核心识别。
 int detect(const cv::Mat& image){using namespace cv;
  Mat hsv;cvtColor(image,hsv,COLOR_BGR2HSV);
  roi=Rect(image.cols*x/100,image.rows*y/100,image.cols*w/100,image.rows*h/100)&Rect(0,0,image.cols,image.rows);
  if(roi.empty())throw std::runtime_error("Invalid light ROI");
  Mat top=hsv(roi),r1,r2,g,core,labels,stats,centroids,accepted=Mat::zeros(top.size(),CV_8U);
  // 红色色相跨 HSV 首尾，分别取 0..12 与 165..179；绿色取 40..95。
  // core 提取低饱和度高亮区域，用于给绿色灯光寻找附近的发光中心。
  inRange(top,Scalar(0,rs,rv),Scalar(12,255,255),r1);inRange(top,Scalar(165,rs,rv),Scalar(179,255,255),r2);
  inRange(top,Scalar(40,gs,gv),Scalar(95,255,255),g);inRange(top,Scalar(0,0,240),Scalar(179,100,255),core);
  int n=connectedComponentsWithStats(core,labels,stats,centroids);
  // 只接受面积、尺寸和形状合理的高亮连通块，再累计其附近满足阈值的绿色像素。
  // 所以 green 不是整个 ROI 内全部绿色像素的数量。
  for(int i=1;i<n;++i){Rect b(stats.at<int>(i,CC_STAT_LEFT),stats.at<int>(i,CC_STAT_TOP),stats.at<int>(i,CC_STAT_WIDTH),stats.at<int>(i,CC_STAT_HEIGHT));
   if(stats.at<int>(i,CC_STAT_AREA)<5||b.width>image.cols/12||b.height>image.rows/9||std::max(b.width,b.height)>3*std::min(b.width,b.height))continue;
   Rect near=(Rect(b.x-6,b.y-6,b.width+12,b.height+12)&Rect(0,0,top.cols,top.rows));
   if(countNonZero(g(near))>=gmin)g(near).copyTo(accepted(near));
  }
  red=countNonZero(r1)+countNonZero(r2);green=countNonZero(accepted);
  // 红色超过阈值优先判红；否则绿色还需达到最小像素数并超过红色的 1.5 倍。
  // 两者都不满足时返回 UNKNOWN，由上层决定是否继续保持已锁存的停车。
  if(red>rmin)return -1; // Red wins if both colors occur.
  return green>=gmin&&green>red*1.5?1:0;
 }
};
