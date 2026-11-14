#include "lane_original.hpp"
#include <fstream>
int main(int argc,char** argv){
    if(argc!=3)return 2;
    cv::setNumThreads(2);cv::Mat frame=cv::imread(argv[1]);if(frame.empty())return 3;cv::resize(frame,frame,{640,480});
    double e=original_lane::TUxiang_Init3(frame);
    cv::Mat color,mask;cv::resize(original_lane::lane_mask,mask,{640,192},0,0,cv::INTER_NEAREST);cv::cvtColor(mask,color,cv::COLOR_GRAY2BGR);
    color.setTo(cv::Scalar(0,255,0),mask);cv::Mat roi=frame(cv::Rect(0,210,640,192));cv::addWeighted(roi,.8,color,.8,0,roi);
    cv::rectangle(frame,{0,210,640,192},{0,220,220},1);
    cv::putText(frame,cv::format("L=%d R=%d error=%.2f",original_lane::lane_left_count,original_lane::lane_right_count,e),{5,25},cv::FONT_HERSHEY_SIMPLEX,.6,{0,255,255},1);
    cv::imwrite(std::string(argv[2])+".jpg",frame);
    std::ofstream out(std::string(argv[2])+".json");out<<"{\"left\":"<<original_lane::lane_left_count<<",\"right\":"<<original_lane::lane_right_count<<",\"error\":"<<e<<"}";
}
