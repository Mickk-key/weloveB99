#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <cmath>
#include <stdexcept>
// Call only after Output::send succeeds. A final neutral stop must persist for the configured delay.
class ParkingPhoto {
 double delay=1.0,since=-1;bool attempted=false;
public:
 explicit ParkingPhoto(double seconds=1.0):delay(seconds){if(!std::isfinite(delay)||delay<0||delay>60)throw std::runtime_error("parking_photo_delay_seconds must be 0..60");}
 bool update(const cv::Mat& raw,double now,const std::string& state,int motor,int neutral,const std::string& path){
  if(attempted)return false;
  bool parked=(state=="PARK_PANEL_STOP"||state=="PARK3_PANEL_STOP"||state=="PARK3_PANEL_STOP"||state=="PARK_TIMED_STOP"||state=="QR_STOP_LATCHED"||state=="PARK_DONE")&&motor==neutral;
  if(!parked){since=-1;return false;}
  if(since<0)since=now;
  if(now-since<delay)return false;
  attempted=true;
  if(raw.empty()||!cv::imwrite(path,raw))throw std::runtime_error("Cannot save parking photo: "+path);
  return true;
 }
};
