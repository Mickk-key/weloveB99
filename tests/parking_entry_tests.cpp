#include "suite.hpp"
#include <cassert>
#include <iostream>
#include <chrono>
#include <thread>

cv::Mat road(int side=0){
    cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));
    cv::line(f,{80,479},{280,280},{240,240,240},8);cv::line(f,{560,479},{360,280},{240,240,240},8);
    if(side)cv::rectangle(f,{side>0?200:360,260,70,55},{255,0,0},-1);
    return f;
}
LaneV2 parallel(double fraction,double heading=0){
    // Width grows with image y. target error/width is constant when aligned;
    // heading>0 means far target right of near target (must steer right).
    LaneV2 l;l.leftValid=l.rightValid=true;
    double a=1.2,b=-160.;
    l.la=-fraction*a-heading;l.lb=320-fraction*b+heading*444.5;
    l.ra=l.la+a;l.rb=l.lb+b;return l;
}
int main(){
    cv::setNumThreads(2);SuiteConfig cfg;cfg.load("config/indoor.yaml");cv::FileStorage fs(cfg.path,cv::FileStorage::READ);
    for(int side:{1,-1}){
        Suite s(cfg,Modules("parking"));s.parking.deferQr=true;
        Command out;double t=1.1;bool sawOut=false,sawAlign=false,sawSettle=false,sawFollow=false;
        for(int i=0;i<60;++i,t+=.05){
            out=s.step(road(side),t);
            assert(out.state!="WAIT_BOARD_SIDE"&&out.motor>10000);
            if(out.state=="PARK_SHIFT_1_OUT"){assert(out.steer==80-side*4);sawOut=true;}
            if(out.state=="PARK_SHIFT_2_ALIGN"){assert(sawOut&&out.steer==80+side*4);sawAlign=true;}
            if(out.state=="PARK_SHIFT_3_SETTLE"){assert(sawAlign&&out.steer==80);sawSettle=true;}
            if(out.state=="PARK_FOLLOW_RIGHT_EDGE"||out.state=="PARK_FOLLOW_LEFT_EDGE"){
                assert(sawSettle&&std::abs(out.steer-80)<=3);sawFollow=true;
                assert(std::abs(out.offset-side*(.5-s.parking.parking_edge_margin_fraction))<1e-9);
            }
        }
        assert(sawOut&&sawAlign&&sawSettle&&sawFollow&&s.parking.side==side);
        auto oldRemaining=s.parking.entry.remaining();s.step(road(-side),t+.1);
        assert(s.parking.side==side&&s.parking.entry.done()&&oldRemaining==0);
        std::cout<<"PASS side="<<side<<" no redundant stop, fixed outward/countersteer/straight, permanent edge follow and side latch\n";
    }
    {Parking p;p.load(fs);p.side=1;p.deferQr=true;BlueBoardStop b;auto l=parallel(.5);auto f=road();
     p.update(b,l,f,1);p.entryFeedback(true);p.update(b,l,f,1.3);assert(std::abs(p.entry.remaining()-.3)<1e-9);
     p.entryFeedback(false);p.update(b,l,f,11.3,false);assert(std::abs(p.entry.remaining()-.3)<1e-9);
     p.entryFeedback(true);p.update(b,l,f,12);assert(p.entry.phase()=="2_ALIGN"); // no skipped reverse step
     p.entryFeedback(true);l.rightValid=false;p.update(b,l,f,12.1);assert(p.hold);p.entryFeedback(false);
     p.update(b,l,f,20);auto remaining=p.entry.remaining();l.rightValid=true;p.update(b,l,f,20.1);assert(!p.hold&&p.entry.remaining()==remaining);}
    std::cout<<"PASS forward-time entry pause on traffic/missing line, no skipped alignment after delayed frame\n";
    {Suite s(cfg,Modules("parking,light"));s.parking.deferQr=true;s.parking.side=1;auto f=road();
     auto out=s.step(f,1.1);assert(out.state=="PARK_SHIFT_1_OUT"&&out.steer==76);
     auto red=f.clone();cv::circle(red,{320,150},15,{0,0,255},-1);
     auto green=f.clone();cv::circle(green,{320,150},15,{0,255,0},-1);cv::circle(green,{320,150},4,{255,255,255},-1);
     out=s.step(red,1.2);assert(out.state=="LIGHT_BRAKE");double remaining=s.parking.entry.remaining();
     out=s.step(red,11.2);assert(out.motor==10000&&s.parking.entry.remaining()==remaining);
     s.step(green,11.3);out=s.step(green,11.9);assert(out.state=="PARK_SHIFT_1_OUT"&&s.parking.entry.remaining()==remaining);
     s.parking.stopped=true;out=s.step(f,12.0);assert(out.state=="QR_STOP_LATCHED"&&out.motor==10000);}
    std::cout<<"PASS Suite entry honors red-light pause/resume and final QR stop priority\n";
    for(int side:{1,-1}){
        Parking p;p.load(fs);p.side=side;double fraction=.5+side*(.5-p.parking_edge_margin_fraction);
        assert(p.updateEdge(parallel(fraction),.05));assert(std::abs(p.edgeError)<1e-8&&std::abs(p.edgeHeading)<1e-8&&std::abs(p.edgeCorrection())<1e-8);
        // Equal normalized lateral error at near/far is offset, not heading.
        p.edgeReady=false;p.updateEdge(parallel(fraction-.08),.05);assert(std::abs(p.edgeHeading)<1e-8&&p.edgeCorrection()>0);
        // Near target centered; far target LEFT after a right turn: countersteer LEFT.
        p.edgeReady=false;p.updateEdge(parallel(fraction,.10),.05);assert(std::abs(p.edgeError)<1e-8&&p.edgeHeading>0&&p.edgeCorrection()>0);
        p.edgeReady=false;p.updateEdge(parallel(fraction,-.10),.05);assert(std::abs(p.edgeError)<1e-8&&p.edgeHeading<0&&p.edgeCorrection()<0);
        if(side>0){auto one=parallel(fraction);one.leftValid=false;assert(p.updateEdge(one,.05));}
        else {auto one=parallel(fraction);one.rightValid=false;assert(p.updateEdge(one,.05));}
    }
    std::cout<<"PASS normalized heading separates lateral distance from yaw, countersteers before line crossing, handles chosen edge alone\n";
    {Parking p;p.load(fs);p.side=1;p.deferQr=true;ParkingQrObservation seen;seen.ready=seen.found=true;seen.side=1;seen.box={400,160,50,50};
     seen.capturedAt=1;p.applyQR(seen,3.1);assert(p.qrHits==0); // too old
     seen.capturedAt=4;seen.side=-1;p.applyQR(seen,4);assert(p.qrHits==0);
     p.qrValidSince=5;seen.side=1;seen.capturedAt=4.9;p.applyQR(seen,5);assert(p.qrHits==0);
     for(int i=0;i<p.qr_confirm_frames;++i){seen.capturedAt=6+i*.1;p.applyQR(seen,seen.capturedAt+.02);}
     assert(p.stopped&&p.state=="QR_STOP_LATCHED");}
    std::cout<<"PASS stale/wrong-side/pre-loss QR rejected; fresh independent confirmations latch final stop\n";
    // Real detector runs behind control. Measure the selected-side path with no camera/GPIO.
    {Suite s(cfg,Modules("parking"),true);s.parking.side=1;auto f=road();
     auto start=std::chrono::steady_clock::now();double maximum=0;
     for(int i=0;i<80;++i){auto before=std::chrono::steady_clock::now();double now=1.1+std::chrono::duration<double>(before-start).count();
         auto out=s.step(f,now);double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-before).count();maximum=std::max(maximum,ms);
         assert(out.motor>10000);std::this_thread::sleep_for(std::chrono::milliseconds(30));}
     std::cout<<"METRIC async selected-side control max_ms="<<maximum<<" (80 synthetic frames, real QR worker)\n";
     assert(maximum<500);}
    {AsyncParkingQR worker(cfg.path);Parking p;p.load(fs);p.side=1;p.boardGroundY=310;p.edgeWidthReady=true;p.edgeWidthA=0;p.edgeWidthB=440;
     LaneV2 l;l.leftValid=l.rightValid=true;l.lb=100;l.rb=540;
     auto q=cv::imread("tests/parking-qr.png");assert(!q.empty());worker.submit(q,l,p,1.);
     ParkingQrObservation seen;auto start=std::chrono::steady_clock::now();
     do{seen=worker.poll();if(seen.ready)break;std::this_thread::sleep_for(std::chrono::milliseconds(10));}
     while(std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<10);
     assert(seen.ready&&seen.found&&seen.side==1&&seen.capturedAt==1.);assert(!worker.poll().ready);
     std::cout<<"PASS real async QR observation uses submitted geometry and is consumed once\n";}
    {Parking p;p.load(fs);p.side=1;p.boardGroundY=315;auto f=road();
     auto start=std::chrono::steady_clock::now();cv::Mat points;p.detectQR(f,points);
     std::cout<<"METRIC unchanged synchronous QR search on no-QR road ms="<<std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()<<"\n";}
}
