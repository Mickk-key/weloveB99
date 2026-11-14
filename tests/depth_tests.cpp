#include "suite.hpp"
using namespace depth_parking;
#include <cassert>
#include <iostream>
int main(){SuiteConfig suite;suite.load("config/outdoor.yaml");auto l=suite.depthDrive;auto c=suite.depth;
    // A broad yellow line by itself must never be enough to select a bay.
    ParkingController noBoard;ParkingObservation o;o.entryDistance=.5;o.laneConfidence=.95;
    ParkingCommand x;for(int i=0;i<20;++i)x=noBoard.step(o,0,.1,l,c);assert(x.state=="FAULT"&&x.motor==10000);
    // Preserve lane-only outputs, even when a parking prop is in view.
    ParkingController laneOnly;o.boardDistance=2;o.boardLateral=.3;for(int i=0;i<30;++i)x=laneOnly.step(o,0,.1,l,c,true);
    assert(x.state=="SEEK"&&x.steer==80&&x.motor==11000);
    for(int side:{-1,1}){ParkingController p;o={};o.laneConfidence=.95;o.boardDistance=2.8;o.boardLateral=side*.30;o.entryDistance=1.3;
        for(int i=0;i<15;++i)x=p.step(o,0,.1,l,c);assert(x.openSide==-side&&x.state=="ENTER");
        // Once entrance disappears, the exit line must not reset the distance.
        o.entryDistance=3.5;
        bool brake=false;for(int i=0;i<100&&x.state!="DONE"&&x.state!="FAULT";++i){x=p.step(o,0,.1,l,c);brake|=x.state=="BRAKE";}
        // Long blind travel is rejected rather than pretending to have parked.
        assert(x.state=="FAULT"&&x.reason=="ENTRY_ODOMETRY_LIMIT"&&!brake);
    }
    c.watchdogEnabled=true;ParkingController stale;o={};x=stale.step(o,0,.31,l,c);assert(x.state=="FAULT"&&x.reason=="STALE_FRAME"&&x.motor==10000);
    ParkingController ambiguous;o.laneConfidence=.95;o.entryDistance=.5;o.ambiguousBoard=true;for(int i=0;i<20;++i)x=ambiguous.step(o,0,.1,l,c);assert(x.state=="FAULT");
    // Successful entrance tracking, full configured brake interval and terminal latch.
    for(int side:{-1,1}){ParkingController p;double distance=1.3,brakeSeconds=0;int previousMotor=10000;o={};o.laneConfidence=.95;o.boardDistance=2.8;o.boardLateral=side*.30;
        for(int i=0;i<200;++i){distance-=std::max(0,previousMotor-10000)*c.speedScale*.1;o.entryDistance=distance>.30?distance:-1;
            x=p.step(o,0,.1,l,c);previousMotor=x.motor;if(x.state=="BRAKE"){assert(x.motor==9000);brakeSeconds+=.1;}if(x.state=="DONE")break;}
        assert(x.state=="DONE"&&x.openSide==-side&&x.motor==10000&&brakeSeconds>=.79&&brakeSeconds<=.91);
        o.boardLateral=-o.boardLateral;o.entryDistance=1;for(int i=0;i<10;++i)x=p.step(o,0,.1,l,c);assert(x.state=="DONE"&&x.motor==10000);
    }
    std::cout<<"Parking safety, selection, lane bypass, stale input and ambiguity checks passed\n";
}
