#include "lane_outdoor_v2.hpp"
#include <iostream>
#include <fstream>
int main(){
    cv::setNumThreads(2);cv::FileStorage fs("config/outdoor_v2.yaml",cv::FileStorage::READ);OutdoorLaneConfig c;c.load(fs);
    int total=0,stable=0;double maxDrift=0;std::ofstream out("evidence/outdoor_v2/stress.csv");out<<"frame,condition,left,right,error,drift_pixels,stable\n";
    for(int i=0;i<8;++i){auto f=cv::imread("evidence/outdoor_photo/frame"+std::to_string(i)+".png");cv::resize(f,f,{640,480});OutdoorLaneDetector base(c);auto clean=base.detect(f,0);
        for(int k=0;k<7;++k){cv::Mat v;std::string name;
            if(k<3){double scale=k==0?.55:k==1?.75:1.3;f.convertTo(v,-1,scale);name="brightness"+std::to_string(scale);}
            if(k==3){cv::GaussianBlur(f,v,{5,5},1.2);name="blur";}
            if(k==4){v=f.clone();v(cv::Rect(0,325,640,65)).convertTo(v(cv::Rect(0,325,640,65)),-1,.55);name="shadow_band";}
            if(k==5){std::vector<uchar> bytes;cv::imencode(".jpg",f,bytes,{cv::IMWRITE_JPEG_QUALITY,35});v=cv::imdecode(bytes,cv::IMREAD_COLOR);name="jpeg35";}
            if(k==6){cv::Mat noise(f.size(),CV_16SC3),wide;cv::RNG rng(123+i);rng.fill(noise,cv::RNG::NORMAL,0,5);f.convertTo(wide,CV_16SC3);wide+=noise;wide.convertTo(v,CV_8UC3);name="noise5";}
            OutdoorLaneDetector detector(c);auto found=detector.detect(v,0);double drift=std::abs(clean.error-found.error)*8;bool ok=found.leftValid==clean.leftValid&&found.rightValid==clean.rightValid&&drift<20;
            ++total;stable+=ok;maxDrift=std::max(maxDrift,drift);out<<i<<','<<name<<','<<found.leftValid<<','<<found.rightValid<<','<<found.error<<','<<drift<<','<<ok<<'\n';
            if(!ok){found.draw(v);cv::imwrite("evidence/outdoor_v2/stress-"+std::to_string(i)+"-"+name+".jpg",v);}
        }
    }
    cv::Mat empty(480,640,CV_8UC3,cv::Scalar(35,30,100));OutdoorLaneDetector d(c);auto no=d.detect(empty,0);if(no.leftValid||no.rightValid)return 2;
    std::cout<<"Perturbation stability (same observed sides and target drift <20 px): "<<stable<<'/'<<total<<"; max drift="<<maxDrift<<" px; blank-red negative PASS\n";
}
