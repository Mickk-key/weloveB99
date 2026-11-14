#include "suite.hpp"
#include <cassert>
#include <iostream>
LaneV2 bounds(){LaneV2 l;l.leftValid=l.rightValid=true;l.lb=100;l.rb=540;return l;}
cv::Mat road(){cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));cv::line(f,{80,479},{280,280},{240,240,240},8);cv::line(f,{560,479},{360,280},{240,240,240},8);return f;}
cv::Mat red(){auto f=road();cv::circle(f,{320,150},15,{0,0,255},-1);return f;}
int main(){cv::setNumThreads(2);SuiteConfig cfg;cfg.load("config/indoor.yaml");
    cv::FileStorage fs(cfg.path,cv::FileStorage::READ);cv::Mat plain(480,640,CV_8UC3,cv::Scalar(255,255,255));
    auto qr=cv::imread("tests/parking-qr.png");assert(!qr.empty());
    for(int side:{1,-1}){
        Parking p;p.load(fs);p.parking_edge_entry=0;p.parking_heading_kp=0;assert(p.parking_edge_follow&&!p.parking_fixed_maneuver);
        auto l=bounds();BlueBoardStop b;b.stopped=b.candidate=true;b.box={side>0?170:390,250,80,60};
        double t=1;for(int i=0;i<p.parking_side_frames;++i){p.update(b,l,plain,t);t+=.05;}
        assert(p.side==side&&p.edgeControl()&&!p.fixedControl());
        assert(std::abs(p.edgeError)<1e-6); p.edgeFeedback(true); for(int i=0;i<100;++i)p.updateEdge(l,.05); double goal=side*(.5-p.parking_edge_margin_fraction); assert(std::abs(p.offset-goal)<1e-6);
        b.candidate=false;p.update(b,l,plain,10);assert(p.side==side&&p.edgeControl());
        assert(std::abs(p.offset-goal)<1e-6); // No timed recenter even after 10 seconds.
        if(side>0){l.leftValid=false;l.rb-=50;}else{l.rightValid=false;l.lb+=50;}
        p.update(b,l,plain,10.1);assert(p.edgeControl());for(int i=0;i<60;++i)p.updateEdge(l,.05); assert(std::abs(p.edgeError-(goal*440-side*50)/8.)<.01);
        if(side>0)l.rightValid=false;else l.leftValid=false;
        p.update(b,l,plain,10.2);assert(p.hold&&!p.edgeControl());
        l=bounds();b.candidate=true;b.box.x=side>0?390:170;
        p.update(b,l,plain,10.3);assert(p.side==side&&!p.hold); // Opposite board cannot flip side.
        if(side>0)l.leftValid=false;else l.rightValid=false;
        for(int i=0;i<p.qr_confirm_frames+1;++i)p.update(b,l,qr,11+i*.05);
        assert(p.stopped==(side>0)); // Selected-half QR works using cached width with one edge.
        if(p.stopped){p.update(b,l,plain,15);assert(!p.edgeControl()&&p.state=="QR_STOP_LATCHED");}
        std::cout<<"PASS side="<<side<<" opposite selection, permanent edge target, boundary motion, single-line tracking, missing-edge wait, QR gate\n";
    }
    {auto c=cfg;c.drive.cone_slow_pwm=10001;c.drive.start_pwm=10002;c.drive.corner_pwm=10003;
     Suite s(c,Modules("parking,light"));s.parking.parking_edge_entry=0;s.parking.parking_heading_kp=0;s.parking.side=1;s.parking.parking_follow_pwm=11050;
     auto out=s.step(road(),1.1);assert(out.motor==11050&&out.state=="PARK_FOLLOW_RIGHT_EDGE"&&std::abs(out.error)<1e-6);
     out=s.step(red(),1.2);assert(out.motor==cfg.lightBrakePwm&&out.state=="LIGHT_BRAKE");
     s.parking.stopped=true;out=s.step(red(),1.3);assert(out.motor==cfg.drive.neutral_pwm&&out.state=="QR_STOP_LATCHED");}
    {SuiteConfig c;c.load("config/outdoor_v2.yaml");Parking p;cv::FileStorage f(c.path,cv::FileStorage::READ);p.load(f);assert(!p.parking_edge_follow&&p.parking_fixed_maneuver);}
    std::cout<<"PASS dedicated parking speed overrides old caps; traffic/final stop priority; outdoor config unchanged\n";
}
