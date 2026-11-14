#include "suite.hpp"
#include <cassert>
#include <iostream>
cv::Mat road(){cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));cv::line(f,{80,479},{280,280},{240,240,240},8);cv::line(f,{560,479},{360,280},{240,240,240},8);return f;}
cv::Mat obstacle(bool left){auto f=road();int x=left?245:395;std::vector<cv::Point> p={{x,340},{x-22,411},{x+22,411}};cv::fillConvexPoly(f,p,{255,0,0});return f;}
cv::Mat light(bool red){auto f=road();cv::circle(f,{320,150},15,red?cv::Scalar(0,0,255):cv::Scalar(0,255,0),-1);if(!red)cv::circle(f,{320,150},4,{255,255,255},-1);return f;}
int main(){cv::setNumThreads(2);SuiteConfig cfg;cfg.load("config/indoor.yaml");assert(cfg.cone.cone_s_course_once&&cfg.cone.fixedPlan.steps.size()==9);
    for(bool left:{true,false})for(int vision:{0,1,2}){
        // Shorter test durations preserve the plan's directions and complete every segment.
        auto c=cfg;for(auto& step:c.cone.fixedPlan.steps)step.seconds=.1;
        c.parkingAfterCones=0;Suite s(c,Modules("cone,parking"));Command out;double t=1;
        for(int i=0;i<c.cone.cone_confirm_frames;++i){out=s.step(obstacle(left),t);t+=.03;assert(!out.parkingArmed);}
        assert(s.cone.fixedTurning);assert(out.steer==(left?75:85));
        std::set<std::string> phases;
        for(int i=0;i<80&&!s.cone.sCourseCompleted();++i){
            auto frame=vision==0?road():obstacle(vision==1?left:!left);
            out=s.step(frame,t);t+=.03;assert(!out.parkingArmed);
            if(s.cone.fixedTurning)phases.insert(out.cone);
        }
        assert(s.cone.sCourseCompleted()&&s.cone.passedCount==1&&out.state=="RUN");assert(phases.size()==9);
        // Even a visible original/second cone cannot retrigger after completion.
        for(int i=0;i<10;++i){out=s.step(obstacle(!left),t);t+=.03;assert(!s.cone.fixedTurning&&s.cone.passedCount==1&&out.parkingArmed);}
        std::cout<<"PASS "<<(left?"left-first":"right-first")<<" vision="<<vision<<" all nine stages, no second detection, latch, lane return and parking gate\n";
    }
    {Suite s(cfg,Modules("cone,light"));Command c;double t=1;
     for(int i=0;i<cfg.cone.cone_confirm_frames;++i){c=s.step(obstacle(true),t);t+=.03;}
     c=s.step(light(true),t);assert(c.state=="LIGHT_BRAKE");double remaining=s.cone.fixedRemaining();
     c=s.step(light(true),t+10);assert(c.motor==10000&&std::abs(s.cone.fixedRemaining()-remaining)<1e-6);
     s.step(light(false),t+10.1);c=s.step(light(false),t+10.7);
     assert(c.state=="CONE_S_RIGHT_1_FIRST_OUT"&&std::abs(s.cone.fixedRemaining()-remaining)<1e-6);}
    {SuiteConfig old;old.load("config/outdoor_v2.yaml");assert(!old.cone.cone_s_course_once);
     for(auto& step:old.cone.fixedPlan.steps)step.seconds=.05;
     Suite s(old,Modules("cone"));double t=1;Command c;
     for(int i=0;i<old.cone.cone_confirm_frames;++i){c=s.step(obstacle(true),t);t+=.03;}
     assert(c.state=="CONE_FIXED_1_OUT");
     for(int i=0;i<50;++i){s.step(road(),t);t+=.03;}
     assert(s.cone.passedCount==1&&!s.cone.sCourseCompleted());
     for(int i=0;i<old.cone.cone_confirm_frames;++i){c=s.step(obstacle(false),t);t+=.03;}
     assert(c.state=="CONE_FIXED_1_OUT"&&c.steer==85);}
    std::cout<<"PASS red pause/resume and outdoor mode preserved\n";
}
