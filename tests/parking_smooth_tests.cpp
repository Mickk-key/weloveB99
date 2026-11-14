#include "suite.hpp"
#include <cassert>
#include <iostream>
LaneV2 lane(double shift=0){LaneV2 l;l.leftValid=l.rightValid=true;l.lb=100+shift;l.rb=540+shift;return l;}
cv::Mat road(){cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));cv::line(f,{80,479},{280,280},{240,240,240},8);cv::line(f,{560,479},{360,280},{240,240,240},8);return f;}
cv::Mat light(bool red){auto f=road();cv::circle(f,{320,150},15,red?cv::Scalar(0,0,255):cv::Scalar(0,255,0),-1);if(!red)cv::circle(f,{320,150},4,{255,255,255},-1);return f;}
int main(){cv::setNumThreads(2);SuiteConfig cfg;cfg.load("config/indoor.yaml");cv::FileStorage fs(cfg.path,cv::FileStorage::READ);
    for(int side:{-1,1}){
        Parking p;p.load(fs);p.parking_edge_entry=0;p.parking_heading_kp=0;p.side=side;assert(p.updateEdge(lane(),.05));assert(std::abs(p.edgeError)<1e-9&&std::abs(p.offset)<1e-9);
        double goal=side*(.5-p.parking_edge_margin_fraction);
        p.edgeFeedback(true);
        for(int i=0;i<100;++i){double old=p.offset;assert(p.updateEdge(lane(),.05));assert(std::abs(p.offset-old)<=p.parking_shift_rate*.05+1e-9);assert(std::abs(p.edgeCorrection())<=p.parking_steer_limit+1e-9);}
        assert(std::abs(p.offset-goal)<1e-9);
        double held=p.offset;p.edgeFeedback(false);p.updateEdge(lane(),.1);assert(p.offset==held);
        // At the desired lateral position the steering converges to straight, not a continuous turn.
        p.edgeFeedback(true);for(int i=0;i<80;++i)p.updateEdge(lane(-goal*440),.05);
        assert(std::abs(p.edgeError)<.001&&std::abs(p.edgeCorrection())<.001);
        std::cout<<"PASS side="<<side<<" zero initial error, bounded gradual shift, steering cap, stop pause and straight-at-target\n";
    }
    {auto c=cfg;c.drive.lane_pd_kp=2;c.drive.lane_pd_kd=2;Suite s(c,Modules("parking,light"));s.parking.parking_edge_entry=0;s.parking.parking_heading_kp=0;s.parking.side=1;s.parking.parking_follow_pwm=11050;
     auto out=s.step(road(),1.1);assert(out.motor==11050&&std::abs(out.error)<1e-9&&out.steer==80);
     double t=1.1,previous=out.target;
     for(int i=0;i<15;++i){t+=.05;out=s.step(road(),t);assert(std::abs(out.target-previous)<=s.parking.parking_steer_rate*.05+1e-6);assert(std::abs(out.steer-80)<=3);previous=out.target;}
     t+=.05;out=s.step(light(true),t);assert(out.state=="LIGHT_BRAKE");double offset=s.parking.offset;
     out=s.step(light(true),t+10);assert(out.motor==10000&&s.parking.offset==offset);
     s.step(light(false),t+10.1);out=s.step(light(false),t+10.7);assert(s.parking.offset==offset&&std::abs(out.steer-80)<=3);
     cv::Mat blank(480,640,CV_8UC3,cv::Scalar(30,30,30));out=s.step(blank,t+10.8);assert(out.motor==10000&&s.parking.side==1);
     out=s.step(road(),t+10.9);assert(out.motor==11050&&out.steer==80&&std::abs(out.error)<1e-9);
     s.parking.stopped=true;out=s.step(road(),t+11);assert(out.state=="QR_STOP_LATCHED"&&out.motor==10000);}
    std::cout<<"PASS dedicated parking PD/slew, speed retained, red pause, missing-edge reacquisition and final stop\n";
}
