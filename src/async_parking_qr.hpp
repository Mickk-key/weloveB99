#pragma once
#include "parking_qr.hpp"
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

// 只有这个工作线程调用自己的QRCodeDetector。控制线程只提交快照/取结果，
// 最多一个待处理帧，新的覆盖旧的，不排队积累旧图。几何与图像来自同一帧。
class AsyncParkingQR {
    struct Job {
        cv::Mat frame;LaneV2 lane;int side=0,boardY=0;
        double widthA=0,widthB=0,now=0;bool widthReady=false;
    };
    Parking detector;
    std::mutex mutex;std::condition_variable wake;std::thread worker;
    bool ending=false,pending=false;Job latest;ParkingQrObservation result;
    void run(){
        for(;;){
            Job job;
            {std::unique_lock<std::mutex> lock(mutex);wake.wait(lock,[this]{return ending||pending;});
             if(ending)return;job=std::move(latest);pending=false;}
            detector.side=job.side;detector.boardGroundY=job.boardY;
            detector.edgeWidthA=job.widthA;detector.edgeWidthB=job.widthB;detector.edgeWidthReady=job.widthReady;
            ParkingQrObservation seen;
            try{seen=detector.observeQR(job.frame,job.lane,job.now);}
            catch(const cv::Exception&){seen.ready=true;seen.side=job.side;seen.capturedAt=job.now;}
            {std::lock_guard<std::mutex> lock(mutex);result=seen;}
        }
    }
public:
    explicit AsyncParkingQR(const std::string& path){
        cv::FileStorage fs(path,cv::FileStorage::READ);detector.load(fs);
        worker=std::thread(&AsyncParkingQR::run,this);
    }
    ~AsyncParkingQR(){
        {std::lock_guard<std::mutex> lock(mutex);ending=true;pending=false;}
        wake.notify_one();if(worker.joinable())worker.join();
    }
    void submit(const cv::Mat& frame,const LaneV2& lane,const Parking& p,double now){
        Job job;job.frame=frame.clone();job.lane=lane;job.side=p.side;job.boardY=p.boardGroundY;
        job.widthA=p.edgeWidthA;job.widthB=p.edgeWidthB;job.widthReady=p.edgeWidthReady;job.now=now;
        {std::lock_guard<std::mutex> lock(mutex);latest=std::move(job);pending=true;}
        wake.notify_one();
    }
    ParkingQrObservation poll(){
        std::lock_guard<std::mutex> lock(mutex);auto seen=result;result.ready=false;return seen;
    }
};
