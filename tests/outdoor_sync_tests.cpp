// Synthetic frames only: no Output object, camera, pigpio or vehicle movement.
#include "suite.hpp"
#include <cassert>
#include <iostream>

cv::Mat road(int boardSide=0, bool lines=true) {
    cv::Mat f(480,640,CV_8UC3,cv::Scalar(55,65,90));
    // Cover the legacy backend's higher ROI as well as the two white-line ROIs;
    // extrapolating a fixture whose horizon lies inside that ROI crosses the fits.
    if(lines){cv::line(f,{80,479},{240,200},{240,240,240},8);cv::line(f,{560,479},{400,200},{240,240,240},8);}
    if(boardSide)cv::rectangle(f,{boardSide>0?200:360,260,70,55},{255,0,0},-1);
    return f;
}
cv::Mat obstacle(int side) {
    auto f=road();int x=side>0?245:395;
    std::vector<cv::Point> p={{x,340},{x-22,411},{x+22,411}};
    cv::fillConvexPoly(f,p,{255,0,0});return f;
}
int expectedSteer(const SuiteConfig& cfg,int side,double steer) {
    return std::lround(std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,cfg.drive.servo_center-side*steer)));
}
int main() {
    cv::setNumThreads(2);
    std::cout.setf(std::ios::unitbuf);
    for(const std::string env:{"indoor","outdoor","outdoor_v2"}) {
        SuiteConfig cfg;cfg.load("config/"+env+".yaml");
        assert(cfg.backend==env&&cfg.cone.cone_s_course_once&&cfg.cone.fixedPlan.steps.size()==9);
        {Suite offline(cfg,Modules("parking"));assert(!offline.parking.deferQr);}
        {Suite live(cfg,Modules("parking"),true);assert(live.parking.deferQr);}
        {Suite laneOnly(cfg,Modules("lane"),true);assert(!laneOnly.parking.deferQr);}
        // Force ambiguous, confirmed board: recognition must never command neutral.
        {Suite s(cfg,Modules("parking"));s.parking.deferQr=true;s.board.stopped=true;
         auto out=s.step(road(),1.1);assert(out.state=="WAIT_BOARD_SIDE");assert(out.motor>cfg.drive.neutral_pwm);}
        // Exercise each backend's own lane geometry with the same parking commands.
        for(int side:{1,-1}) {
            Suite s(cfg,Modules("parking"));s.parking.deferQr=true;s.parking.side=side;
            bool outSeen=false,alignSeen=false,settleSeen=false,followSeen=false;
            double t=1.1;
            for(int i=0;i<110;++i,t+=.05) {
                // Both lines disappear during the fixed shift. Indoor fallback must remain available outdoors.
                auto out=s.step(road(0,!(i>=3&&i<16)),t);
                if(out.motor<=cfg.drive.neutral_pwm)std::cerr<<env<<" side="<<side<<" frame="<<i<<" state="<<out.state<<" left="<<out.left<<" right="<<out.right<<"\n";
                assert(out.motor>cfg.drive.neutral_pwm);
                if(out.state=="PARK_SHIFT_1_OUT") {assert(out.steer==expectedSteer(cfg,side,s.parking.entryPlan.steps[0].steer));outSeen=true;}
                if(out.state=="PARK_SHIFT_2_ALIGN") {assert(outSeen&&out.steer==expectedSteer(cfg,side,s.parking.entryPlan.steps[1].steer));alignSeen=true;}
                if(out.state=="PARK_SHIFT_3_SETTLE") {assert(alignSeen&&out.steer==std::lround(cfg.drive.servo_center));settleSeen=true;}
                if(out.state==(side>0?"PARK_FOLLOW_RIGHT_EDGE":"PARK_FOLLOW_LEFT_EDGE")) {
                    assert(settleSeen&&out.motor==s.parking.parking_follow_pwm);followSeen=true;
                    assert(std::abs(out.steer-cfg.drive.servo_center)<=s.parking.parking_steer_limit);
                    assert(std::abs(out.offset-side*(.5-s.parking.parking_edge_margin_fraction))<1e-9);
                }
            }
            assert(outSeen&&alignSeen&&settleSeen&&followSeen&&s.parking.side==side);
            // Feed independent confirmed observations through the real QR latch and final arbiter.
            ParkingQrObservation seen;seen.ready=seen.found=true;seen.side=side;seen.box={side>0?400:150,160,50,50};
            for(int i=0;i<s.parking.qr_confirm_frames;++i) {seen.capturedAt=t+i*.05;s.parking.applyQR(seen,seen.capturedAt);}
            auto out=s.step(road(),t+1);assert(out.state=="QR_STOP_LATCHED"&&out.motor==cfg.drive.neutral_pwm);
            out=s.step(road(-side),t+2);assert(out.motor==cfg.drive.neutral_pwm&&s.parking.side==side);
        }
        std::cout<<"PASS "<<env<<" async selection, recognition while moving, mirrored entry/settle/edge follow, missing-line fallback, QR stop latch\n";
        // The legacy grayscale backend has its own camera geometry. Real synthetic
        // blue-object detection is checked on the two white-line backends here.
        if(env=="outdoor")continue;
        for(int side:{1,-1}) {
            {Suite s(cfg,Modules("parking"));s.parking.deferQr=true;Command out;
             for(int i=0;i<20&&!s.parking.side;++i){out=s.step(road(side),1.1+i*.05);assert(out.motor>cfg.drive.neutral_pwm);}
             assert(s.parking.side==side&&out.state=="PARK_SHIFT_1_OUT");}
            auto c=cfg;for(auto& step:c.cone.fixedPlan.steps)step.seconds=.1;
            Suite s(c,Modules("cone,parking"));s.parking.deferQr=true;Command out;double t=1.1;
            for(int i=0;i<c.cone.cone_confirm_frames;++i,t+=.03){out=s.step(obstacle(side),t);assert(!out.parkingArmed);}
            assert(s.cone.fixedTurning&&out.steer==expectedSteer(c,side,c.cone.fixedPlan.steps[0].steer));
            std::set<std::string> phases;
            for(int i=0;i<80&&!s.cone.sCourseCompleted();++i,t+=.03){out=s.step(obstacle(-side),t);if(s.cone.fixedTurning)phases.insert(out.cone);assert(!out.parkingArmed);}
            assert(s.cone.sCourseCompleted()&&s.cone.passedCount==1&&phases.size()==9&&out.state=="RUN");
            for(int i=0;i<10;++i,t+=.03){out=s.step(obstacle(-side),t);assert(!s.cone.fixedTurning&&out.parkingArmed&&s.cone.passedCount==1);}
        }
        std::cout<<"PASS "<<env<<" real synthetic board side detection, nine-stage S both directions, second cone ignored, parking armed only after S\n";
    }
}
