#include "suite.hpp"
#include <cassert>
cv::Mat scene(){
    cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));
    cv::line(f,{80,479},{280,280},{240,240,240},8);cv::line(f,{560,479},{360,280},{240,240,240},8);return f;
}
cv::Mat light(bool red){auto f=scene();cv::circle(f,{320,150},15,red?cv::Scalar(0,0,255):cv::Scalar(0,255,0),-1);if(!red)cv::circle(f,{320,150},4,{255,255,255},-1);return f;}
cv::Mat zebraFrame(){auto f=scene();for(int x=210;x<420;x+=40)cv::rectangle(f,{x,345,22,40},{255,255,255},-1);return f;}
int main(){
    cv::setNumThreads(2);
    for(const std::string env:{"indoor","outdoor","outdoor_v2","sim"}){
        SuiteConfig cfg;cfg.load("config/"+env+".yaml");
        // Every subset of four optional modules constructs and processes frames.
        for(int bits=0;bits<16;++bits){std::string m="lane";if(bits&1)m+=",light";if(bits&2)m+=",cone";if(bits&4)m+=",zebra";if(bits&8)m+=",parking";
            Suite s(cfg,Modules(m));auto c=s.step(scene(),0);assert(c.motor==10000);c=s.step(scene(),1.1);assert(c.motor>10000);
            if(!(bits&1)){c=s.step(light(true),1.2);assert(c.state!="LIGHT_STOP");}
        }
        {Suite s(cfg,Modules("light,cone"));auto c=s.step(light(true),1.1);assert(c.motor==10000&&c.state=="LIGHT_STOP");
         c=s.step(scene(),1.2);assert(c.motor==10000);c=s.step(light(false),1.3);assert(c.motor==10000);
         c=s.step(light(false),1.9);assert(c.motor>10000);}
        {Suite s(cfg,Modules("zebra"));Command c;for(int i=0;i<5;++i)c=s.step(zebraFrame(),1.1+i*.1);
         assert(c.state=="ZEBRA_BRAKE"&&c.motor==cfg.zebraBrakePwm);c=s.step(scene(),2.2);assert(c.state=="ZEBRA_HOLD"&&c.motor==10000);
         c=s.step(scene(),12.2);assert(c.motor>10000);c=s.step(zebraFrame(),12.3);assert(c.state!="ZEBRA_BRAKE");}
        {Suite s(cfg,Modules("light,zebra"));Command c;auto frame=zebraFrame();cv::circle(frame,{320,150},15,{0,0,255},-1);
         for(int i=0;i<5;++i)c=s.step(frame,1.1+i*.1);assert(c.zebra=="HOLD"&&c.motor==10000&&c.state=="LIGHT_STOP");}
        {Suite s(cfg,Modules("light"));auto c=s.step(scene(),1.1);assert(c.motor>10000);
         c=s.step(light(true),1.2);assert(c.state=="LIGHT_BRAKE"&&c.motor==9000&&std::abs(c.brakeRemaining-.8)<1e-6);
         c=s.step(light(true),1.9);assert(c.state=="LIGHT_BRAKE");c=s.step(light(true),2.01);assert(c.state=="LIGHT_STOP"&&c.motor==10000);
         c=s.step(light(true),3);assert(c.motor==10000);c=s.step(scene(),3.1);assert(c.motor==10000);
         c=s.step(light(false),3.2);assert(c.motor==10000);c=s.step(light(false),3.8);assert(c.motor>10000);
         c=s.step(light(true),4);assert(c.state=="LIGHT_BRAKE");}
        {Suite s(cfg,Modules("light"));s.step(scene(),1.1);s.step(light(true),1.2);s.step(light(false),1.25);
         auto c=s.step(light(false),1.8);assert(c.state=="LIGHT_BRAKE");c=s.step(light(false),2.01);assert(c.motor>10000);}
        {Suite s(cfg,Modules("zebra"));Command c;for(int i=0;i<3;++i)c=s.step(zebraFrame(),i*.1);assert(c.motor==10000&&c.zebra=="HOLD");}
        {Suite s(cfg,Modules("all"));auto c=s.step(scene(),1.1);assert(!c.parkingArmed&&c.parking=="WAIT_CONES");}
        // QR terminal stop cannot be overridden by red/green or a zebra command.
        {Suite s(cfg,Modules("light,zebra,parking"));s.parking.stopped=true;auto c=s.step(light(true),1.1);assert(c.state=="QR_STOP_LATCHED"&&c.motor==10000);c=s.step(light(false),2);assert(c.motor==10000);}
    }
    // Fractional precision and disabled/enabled timeout behavior are inherited.
    {Settings c;OriginalPd p;p.previous=-2;assert(std::abs(p.update(-2,c)-80.4)<1e-9);Output o(c,false);o.send(10900,79,"CONE");std::this_thread::sleep_for(std::chrono::milliseconds(380));assert(!o.aborted);}
    {Settings c;c.control_watchdog_enabled=1;Output o(c,false);o.send(10900,79,"CONE");std::this_thread::sleep_for(std::chrono::milliseconds(380));assert(o.aborted==2);}
    {Settings c;Output o(c,false);o.send(9000,80,"LIGHT_BRAKE",.08);std::this_thread::sleep_for(std::chrono::milliseconds(140));assert(o.brake_done&&!o.aborted);}
    {Settings c;Output o(c,false);o.send(9000,80,"LIGHT_BRAKE",.12);std::this_thread::sleep_for(std::chrono::milliseconds(80));
     o.send(9000,80,"LIGHT_BRAKE",.12);std::this_thread::sleep_for(std::chrono::milliseconds(80));assert(o.brake_done);
     o.send(9000,80,"LIGHT_BRAKE",.12);assert(o.brake_done);o.send(10000,80,"LIGHT_STOP");
     o.send(11000,80,"RUN");o.send(9000,80,"LIGHT_BRAKE",.12);assert(!o.brake_done);}
    // Depth module remains selectable, handles external stop feedback, and no
    // longer times out simply because earlier race modules took >45 seconds.
    {SuiteConfig c;c.load("config/outdoor.yaml");depth_parking::ParkingController p;depth_parking::ParkingObservation o;depth_parking::ParkingCommand x;
     for(int i=0;i<100;++i)x=p.step(o,0,.5,c.depthDrive,c.depth);assert(x.state=="SEEK");
     o.laneConfidence=.95;o.boardDistance=2.8;o.boardLateral=.3;o.entryDistance=1.3;
     for(int i=0;i<10;++i)x=p.step(o,0,.1,c.depthDrive,c.depth);assert(x.openSide==-1&&x.state=="ENTER");
     p.feedback(10000,80);o.entryDistance=-1;double before=x.remaining;x=p.step(o,0,.1,c.depthDrive,c.depth);assert(std::abs(x.remaining-before)<1e-9);}
    bool rejected=false;try{Modules invalid("lane,typo");}catch(...){rejected=true;}assert(rejected);
    {SuiteConfig c;c.load("config/outdoor.yaml");c.parkingMode="depth";c.drive.lane_pd_kp=0;bool rejected=false;try{Suite invalid(c,Modules("parking"));}catch(...){rejected=true;}assert(rejected);}
    {Parking p;p.boardGroundY=369;cv::Mat points;auto f=cv::imread("tests/small-qr.jpg");assert(!f.empty());assert(p.detectQR(f,points));
     cv::rectangle(f,{257,250,96,119},{180,180,180},-1);assert(!p.detectQR(f,points));}
    std::cout<<"PASS: 4 profiles x 16 module combinations; red/green; zebra brake/hold/rearm; arbitration; QR terminal priority; depth handoff; timeout switches\n";
}
