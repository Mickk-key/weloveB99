#include "lane_outdoor_v2.hpp"
#include <chrono>
#include <iostream>
int main(int argc,char**argv){cv::setNumThreads(2);cv::Mat f=cv::imread(argv[1]);cv::resize(f,f,{640,480});OutdoorLaneDetector d;auto a=std::chrono::steady_clock::now();for(int i=0;i<30;++i)d.detect(f,i*.05);std::cout<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-a).count()/30<<" ms/frame\n";}
