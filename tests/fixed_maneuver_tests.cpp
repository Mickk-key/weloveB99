#include "suite.hpp"
#include <cassert>
#include <iostream>

cv::Mat road(){
    cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));
    cv::line(f,{80,479},{280,280},{240,240,240},8);
    cv::line(f,{560,479},{360,280},{240,240,240},8);return f;
}
cv::Mat coneFrame(bool left=true){
    auto f=road();int x=left?245:395;
    std::vector<cv::Point> p={{x,340},{x-22,411},{x+22,411}};
    cv::fillConvexPoly(f,p,{255,0,0});return f;
}
cv::Mat traffic(int color){
    auto f=road();cv::circle(f,{320,150},15,color<0?cv::Scalar(0,0,255):cv::Scalar(0,255,0),-1);
    if(color>0)cv::circle(f,{320,150},4,{255,255,255},-1);return f;
}
cv::Mat zebraFrame(){auto f=road();for(int x=210;x<420;x+=40)cv::rectangle(f,{x,345,22,40},{255,255,255},-1);return f;}
LaneV2 bounds(){LaneV2 l;l.leftValid=l.rightValid=true;l.la=l.ra=0;l.lb=100;l.rb=540;return l;}
void eq(double a,double b){assert(std::abs(a-b)<1e-6);}
Parking parkingConfig(const SuiteConfig& c){Parking p;cv::FileStorage f(c.path,cv::FileStorage::READ);p.load(f);return p;}
int main(){
    cv::setNumThreads(2);SuiteConfig cfg;cfg.load("config/outdoor_v2.yaml");
    assert(cfg.cone.cone_fixed_maneuver==1&&cfg.cone.fixedPlan.steps.size()==5);
    auto lane=bounds();auto empty=road();
    // Pause/resume counts issued forward commands. A delayed frame cannot skip unissued stages.
    {FixedPlan p;p.steps={{"A",5,.5,10900},{"B",-5,.2,10800},{"C",0,.3,11000}};
     FixedManeuver m;m.start(p,1);m.feedback(true);m.tick(1.2);eq(m.remaining(),.3);
     m.feedback(false);m.tick(90);eq(m.remaining(),.3);m.feedback(true);m.tick(90.3);
     assert(m.stepNumber()==2);eq(m.remaining(),.2);m.feedback(true);m.tick(99);
     assert(m.stepNumber()==3);eq(m.remaining(),.3);m.feedback(true);m.tick(99.3);assert(m.done());}
    std::cout<<"PASS fixed timer pause, boundary and delayed-frame sequence\n";

    // Both sides, locked turn sign, disappearance mid-action and exactly one pass per visible cone.
    for(bool left:{true,false}){
        ConeAvoidV2 c(cfg.cone);auto seen=coneFrame(left);double t=1;
        for(int i=0;i<3;++i,t+=.05)c.update(seen,lane,t);
        assert(c.fixedTurning&&c.phase=="FIXED_1_OUT");eq(c.fixedCorrection(),left?5:-5);
        c.fixedFeedback(true);c.update(empty,LaneV2(),t);assert(c.fixedTurning&&c.offsetFraction==0&&c.passedCount==0);
        // Seeing the same cone again keeps WAIT_CLEAR from double-triggering after all steps.
        for(int i=0;i<100;++i){t+=.05;c.fixedFeedback(true);c.update(seen,lane,t);}
        assert(!c.fixedTurning&&c.phase=="WAIT_CLEAR"&&c.passedCount==0);
        for(int i=0;i<20;++i){t+=.05;c.update(empty,lane,t);}
        assert(c.passedCount==1&&!c.fixedTurning);
        for(int i=0;i<20;++i){t+=.05;c.update(empty,lane,t);}assert(c.passedCount==1);
        for(int i=0;i<3;++i){t+=.05;c.update(coneFrame(!left),lane,t);}
        assert(c.fixedTurning);eq(c.fixedCorrection(),left?-5:5);
    }
    {ConeAvoidV2 c(cfg.cone);c.update(coneFrame(),lane,1);c.update(empty,lane,1.05);
     c.update(coneFrame(),lane,1.1);c.update(coneFrame(),lane,1.15);assert(!c.fixedTurning);
     c.update(coneFrame(),lane,1.2);assert(c.fixedTurning);}
    {auto config=cfg.cone;config.cone_fixed_side=-1;ConeAvoidV2 c(config);
     for(int i=0;i<3;++i)c.update(coneFrame(),lane,1+i*.05);eq(c.fixedCorrection(),-5);}
    std::cout<<"PASS cone mirrored directions, direction lock, dropout, rearm and consecutive confirmation\n";

    // Real Suite output must ignore lane PD/slew and use each stage's configured motor.
    {auto config=cfg;config.drive.steer_rate=.1;config.cone.fixedPlan.steps[0].motor=10820;
     Suite s(config,Modules("cone,light"));s.step(empty,0);Command c;
     for(int i=0;i<3;++i)c=s.step(coneFrame(),1.1+i*.05);
     assert(c.state=="CONE_FIXED_1_OUT"&&c.steer==75&&c.motor==10820&&c.offset==0);
     c=s.step(traffic(-1),1.25);assert(c.state=="LIGHT_BRAKE");double remain=s.cone.fixedRemaining();
     c=s.step(traffic(-1),12);assert(c.motor==10000);eq(s.cone.fixedRemaining(),remain);
     c=s.step(traffic(1),12.1);assert(c.motor==10000);
     c=s.step(traffic(1),12.7);assert(c.state=="CONE_FIXED_1_OUT"&&c.steer==75);eq(s.cone.fixedRemaining(),remain);
     c=s.step(empty,13.31);assert(c.state=="CONE_FIXED_2_LEVEL"&&c.steer==85&&c.motor==10900);
    }
    {Suite s(cfg,Modules("cone,zebra"));s.step(empty,0);Command c;
     for(int i=0;i<3;++i)c=s.step(coneFrame(),1.1+i*.05);
     for(int i=0;i<3;++i)c=s.step(zebraFrame(),1.25+i*.05);
     assert(c.state=="ZEBRA_BRAKE");double remain=s.cone.fixedRemaining();
     c=s.step(empty,2.3);assert(c.state=="ZEBRA_HOLD");eq(s.cone.fixedRemaining(),remain);
     c=s.step(empty,12.3);assert(c.state=="CONE_FIXED_1_OUT");eq(s.cone.fixedRemaining(),remain);}
    {Suite s(cfg,Modules("cone"));Command c;for(int i=0;i<10;++i)c=s.step(coneFrame(),i*.09);
     assert(!s.cone.fixedTurning&&c.motor==10000);}
    {auto config=cfg;config.cone.fixedPlan.steps[0].steer=25;
     Suite s(config,Modules("cone"));Command c;for(int i=0;i<3;++i)c=s.step(coneFrame(false),1+i*.05);
     assert(c.steer==86&&c.motor==10900);}
    std::cout<<"PASS Suite fixed PWM, steering limits, startup wait, red and zebra pause/resume\n";

    // Parking unlocks after two completed/disappeared cones, never just on detection.
    {Suite s(cfg,Modules("cone,parking"));double t=1;Command c;
     for(int n=0;n<2;++n){
         for(int i=0;i<3;++i){t+=.05;c=s.step(coneFrame(n==0),t);assert(!c.parkingArmed);}
         assert(s.cone.fixedTurning);
         for(int i=0;i<85;++i){t+=.05;c=s.step(empty,t);}
         assert(c.passed==n+1);if(n==0)assert(!c.parkingArmed);
     }
     c=s.step(empty,t+.05);assert(c.parkingArmed&&c.passed==2);}
    std::cout<<"PASS two completed cones unlock parking\n";

    auto qr=cv::imread("tests/parking-qr.png");assert(!qr.empty());
    cv::Mat plain(480,640,CV_8UC3,cv::Scalar(255,255,255));
    for(int side:{1,-1}){
        auto p=parkingConfig(cfg);BlueBoardStop b;b.stopped=b.candidate=true;
        b.box={side>0?170:390,240,80,60};
        for(int i=0;i<4;++i)p.update(b,lane,plain,1+i*.05);
        assert(p.side==side&&!p.fixedControl()&&p.state=="PARK_APPROACH_BOARD");
        b.box.y=300;p.update(b,lane,plain,1.3);assert(p.fixedControl());eq(p.fixedCorrection(),side*5);
        p.fixedFeedback(true);b.candidate=false;p.update(b,LaneV2(),empty,1.4);
        assert(!p.hold&&p.fixedControl());double remain=p.fixedRemaining();
        p.fixedFeedback(false);p.update(b,LaneV2(),empty,11.4,false);eq(p.fixedRemaining(),remain);
        double t=11.4;for(int i=0;i<50;++i){t+=.05;p.fixedFeedback(true);p.update(b,lane,plain,t);}
        assert(p.fixedControl()&&p.state=="PARK_FIXED_WAIT_QR"&&p.offset==0&&!p.stopped);eq(p.fixedCorrection(),0);
        assert(p.fixedMotor()==10900);
        for(int i=0;i<4;++i){t+=.05;p.update(b,lane,qr,t);}
        if(side>0){assert(p.stopped);p.update(b,lane,empty,t+.1);assert(p.stopped&&!p.fixedControl());}
        else assert(!p.stopped);
    }
    // Explicit side works without both lines while performing the maneuver.
    {auto p=parkingConfig(cfg);p.parking_fixed_side=-1;BlueBoardStop b;b.stopped=b.candidate=true;b.box={170,300,80,60};
     p.update(b,LaneV2(),empty,1);assert(p.side==-1&&p.fixedControl()&&!p.hold);}
    // QR terminal stop still has priority even if it is seen before the fixed maneuver finishes.
    {auto p=parkingConfig(cfg);BlueBoardStop b;b.stopped=b.candidate=true;b.box={170,300,80,60};
     for(int i=0;i<8;++i)p.update(b,lane,qr,1+i*.05);assert(p.stopped&&!p.fixedControl());}
    // Exercise parking's actual Suite arbitration, including an external traffic stop.
    {Suite s(cfg,Modules("parking,light"));s.parking.side=1;s.parking.fixedStarted=true;
     s.parking.fixed.start(s.parking.fixedPlan,1);auto c=s.step(empty,1.1);
     assert(c.state=="PARK_FIXED_1_OUT"&&c.steer==75&&c.motor==10900);
     c=s.step(traffic(-1),1.2);assert(c.state=="LIGHT_BRAKE");double remain=s.parking.fixedRemaining();
     c=s.step(traffic(-1),11);eq(s.parking.fixedRemaining(),remain);assert(c.motor==10000);
     s.step(traffic(1),11.1);c=s.step(traffic(1),11.7);assert(c.state=="PARK_FIXED_1_OUT");eq(s.parking.fixedRemaining(),remain);
     s.parking.stopped=true;c=s.step(traffic(1),11.8);assert(c.state=="QR_STOP_LATCHED"&&c.motor==10000);}
    std::cout<<"PASS parking start row, mirror/lock, dropout, pause, fixed seek and QR stop priority\n";
}
