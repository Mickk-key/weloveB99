#include "lane_outdoor_v2.hpp"
#include <fstream>
int main(int argc,char** argv){
    if(argc!=4)return 2;cv::setNumThreads(2);cv::Mat f=cv::imread(argv[1]);if(f.empty())return 3;cv::resize(f,f,{640,480});
    cv::FileStorage fs(argv[3],cv::FileStorage::READ);OutdoorLaneConfig c;c.load(fs);OutdoorLaneDetector d(c);auto lane=d.detect(f,0);lane.draw(f);
    cv::putText(f,cv::format("NEW L%d R%d e%.2f",lane.leftValid,lane.rightValid,lane.error),{5,25},cv::FONT_HERSHEY_SIMPLEX,.6,{0,255,255},1);
    cv::imwrite(std::string(argv[2])+".jpg",f);cv::imwrite(std::string(argv[2])+"-mask.png",d.mask);
    std::ofstream o(std::string(argv[2])+".json");o<<"{\"left\":"<<lane.leftValid<<",\"right\":"<<lane.rightValid<<",\"error\":"<<lane.error<<",\"la\":"<<lane.la<<",\"lb\":"<<lane.lb<<",\"ra\":"<<lane.ra<<",\"rb\":"<<lane.rb<<"}";
}
