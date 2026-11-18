#pragma once
#include "cone_config.hpp"
#include "lane_v2.hpp"
#include <set>
#include <limits>

// Pure image-space controller. It does not infer metres or drive any hardware.
// Both cones are associated every frame; a passed track remains a tombstone while
// visible, preventing the same clipped cone from being counted twice.
class ConeLaneVisual {
    struct Track {
        int id=0,hits=0,side=0,order=0;
        cv::Rect box;
        double first=0,last=0,minY=480,maxY=0;
        double firstX=0,vx=0,vy=0,movingLost=0;
        int motionSamples=0;
        bool velocityValid=false;
        bool confirmed=false,engaged=false,passed=false,seen=false;
    };
    ConeConfig cfg;
    std::vector<Track> tracks;
    int nextId=1,currentId=0,nextPrimaryOrder=1;
    std::set<int> passedOrders;
    double lastTime=-1;
    bool driving=false;
    LaneV2 realLane;
    std::vector<cv::Point> leftPath,rightPath,targetPath;
    static double clip(double x,double lo,double hi){return std::max(lo,std::min(hi,x));}
    static double foot(const cv::Rect& r){return r.y+r.height;}
    static double cx(const cv::Rect& r){return r.x+r.width*.5;}
    static double ease(double x){x=clip(x,0,1);return x*x*(3-2*x);}
    Track* find(int id){for(auto& t:tracks)if(t.id==id)return &t;return nullptr;}
    bool fresh(const Track& t,double now)const{return now-t.last<=cfg.cone_lost_grace_s;}
    // Keep identity longer than the steering hold: at the measured ~4 FPS, a
    // single missed frame already exceeds the old 0.3 s association lifetime.
    double identityGrace()const{return std::max(1.2,cfg.cone_lost_grace_s*3);}
    int observedSide(const Track& t)const {
        return cx(t.box)<=(realLane.leftAt(foot(t.box))+realLane.rightAt(foot(t.box)))*.5?1:-1;
    }
    int courseSide(int order)const{return firstPassSide*(order%2?1:-1);}
    void commit(Track& t,int order){
        t.order=order;
        if(!firstPassSide)firstPassSide=t.side;
        t.side=courseSide(order);
    }
    bool exitEligible(const Track& t,double now)const {
        bool sideExit=t.maxY>=cfg.cone_full_avoid_y&&
            (t.side>0?t.box.x<=6:t.box.br().x>=634);
        return t.engaged&&t.confirmed&&(t.maxY>=cfg.cone_pass_y||sideExit)&&
               t.maxY-t.minY>=cfg.cone_approach_pixels&&now-t.first>=cfg.cone_approach_seconds;
    }
    int predictedImageExit(const Track& t,double now)const {
        // At ~4 FPS a near cone can leave between observations, before its
        // last measured bottom reaches pass_y. Require measured approach AND
        // outward translation, then extrapolate only a short moving interval.
        // A static/far disappearance, a stopped interval, or time alone cannot pass.
        double gap=now-t.last;
        double horizon=std::min(.8,std::max(.65,3*cfg.cone_lost_grace_s));
        if(t.seen||!t.engaged||!t.confirmed||!t.order||!t.velocityValid||t.motionSamples<2||
           gap<=cfg.cone_lost_grace_s||gap>horizon+.3||t.movingLost+1e-6<gap||
           t.maxY<cfg.cone_full_avoid_y-cfg.cone_close_depth_px||
           t.maxY-t.minY<cfg.cone_approach_pixels||now-t.first<cfg.cone_approach_seconds||
           t.vy<=0||t.side*t.vx>=-20||
           t.side*(t.firstX-cx(t.box))<std::max(20.,t.box.width*.5))return false;
        double project=std::min(gap,horizon);
        double x=t.box.x+t.vx*project,bottom=foot(t.box)+t.vy*project;
        bool alreadyBeside=t.side>0?t.box.br().x<256:t.box.x>384;
        bool exits=t.side>0?x<=6:x+t.box.width>=634;
        if(!alreadyBeside||bottom<cfg.cone_full_avoid_y)return 0;
        if(exits)return 1;
        // A cone can leave through the bottom while still horizontally inside
        // the image. Keep all approach/outward-motion/forward-evidence guards.
        return bottom>=478?2:0;
    }
    void associate(const std::vector<cv::Rect>& boxes,double now,bool moving) {
        struct Pair {double cost;int track,observation;};std::vector<Pair> pairs;
        for(size_t i=0;i<tracks.size();++i){
            auto& t=tracks[i];t.seen=false;
            double gap=now-t.last;if(gap>identityGrace())continue;
            double xGate=std::max(45.,t.box.width*2.)+std::min(gap,.5)*80;
            double yGate=std::max(30.,t.box.height*1.7)+std::min(gap,.5)*60;
            for(size_t j=0;j<boxes.size();++j){
                const auto& b=boxes[j];double dx=std::abs(cx(b)-cx(t.box)),dy=foot(b)-foot(t.box);
                double residualY=std::abs(dy);
                if(t.velocityValid){
                    double predict=std::min(gap,.6);
                    dx=std::min(dx,std::abs(cx(b)-cx(t.box)-t.vx*predict));
                    residualY=std::min(residualY,std::abs(dy-t.vy*predict));
                }
                double ratio=double(b.width)/std::max(1,t.box.width);
                if(dx>xGate||residualY>yGate||dy< -std::max(20.,t.box.height*.45)||ratio<.35||ratio>2.8)continue;
                pairs.push_back({dx/xGate+residualY/yGate+.15*std::abs(std::log(ratio)),int(i),int(j)});
            }
        }
        std::sort(pairs.begin(),pairs.end(),[](const Pair& a,const Pair& b){return a.cost<b.cost;});
        std::set<int> usedTracks,usedBoxes;
        for(const auto& p:pairs){
            if(usedTracks.count(p.track)||usedBoxes.count(p.observation))continue;
            auto& t=tracks[p.track];const auto& observed=boxes[p.observation];double gap=now-t.last;
            if(gap>1e-6){
                double vx=clip((cx(observed)-cx(t.box))/gap,-500,500);
                double vy=clip((foot(observed)-foot(t.box))/gap,-500,500);
                t.vx=t.velocityValid?.25*t.vx+.75*vx:vx;
                t.vy=t.velocityValid?.25*t.vy+.75*vy:vy;t.velocityValid=true;
                t.motionSamples=moving?t.motionSamples+1:0;
            }
            t.box=observed;t.last=now;t.movingLost=0;t.seen=true;++t.hits;
            t.minY=std::min(t.minY,foot(t.box));t.maxY=std::max(t.maxY,foot(t.box));
            usedTracks.insert(p.track);usedBoxes.insert(p.observation);
        }
        for(auto& t:tracks)if(!t.seen&&!t.confirmed){t.hits=0;t.first=now;}
        for(size_t j=0;j<boxes.size();++j)if(!usedBoxes.count(j)&&tracks.size()<12){
            Track t;t.id=nextId++;t.box=boxes[j];t.hits=1;t.first=t.last=now;
            t.firstX=cx(t.box);
            t.minY=t.maxY=foot(t.box);t.seen=true;tracks.push_back(t);
        }
        for(auto& t:tracks){
            if(!t.confirmed&&t.hits>=cfg.cone_confirm_frames&&now-t.first+1e-9>=cfg.cone_confirm_seconds){
                t.confirmed=true;
                // Side belongs to the track, not to the latest steering-induced image shift.
                t.side=observedSide(t);
            }
        }
    }
    bool clearBeside(const Track& t)const {
        double width=std::max(1.,realLane.rightAt(foot(t.box))-realLane.leftAt(foot(t.box)));
        double margin=cfg.cone_clearance_fraction*width;
        return t.side>0?320>=t.box.br().x+margin:320<=t.box.x-margin;
    }
    // Replace only the boundary on the cone side. The opposite real boundary
    // remains. Project the anchor through the current lane's perspective.
    void corridor(const Track& t,double y,double& l,double& r)const {
        l=realLane.leftAt(y);r=realLane.rightAt(y);double w=std::max(1.,r-l);
        double anchorLeft=realLane.leftAt(foot(t.box));
        double anchorWidth=std::max(1.,realLane.rightAt(foot(t.box))-anchorLeft);
        if(t.side>0){
            double u=(t.box.br().x-anchorLeft)/anchorWidth+cfg.cone_clearance_fraction;
            l+=clip(u,0,1-cfg.cone_min_gap_fraction)*w;
        }else{
            double u=(t.box.x-anchorLeft)/anchorWidth-cfg.cone_clearance_fraction;
            r=l+clip(u,cfg.cone_min_gap_fraction,1)*w;
        }
    }
public:
    int passed=0,targetId=0,nextTargetId=0,trackCount=0,side=0;
    int firstPassSide=0,index=0,nextSide=0,rawSide=0,nextRawSide=0;
    double offset=0,errorDelta=0,handoff=0;
    bool near=false;
    cv::Rect box,nextBox;
    std::string phase="SEARCH";
    std::string passReason="none";
    explicit ConeLaneVisual(ConeConfig c):cfg(c){}
    bool controlling()const{return near||std::abs(offset)>1e-5;}
    bool completed()const{return passed>=cfg.cone_lane_count&&std::abs(offset)<1e-4;}
    bool searchFinished()const{return nextPrimaryOrder>cfg.cone_lane_count||passed>=cfg.cone_lane_count;}
    void feedback(bool moving){
        driving=moving;
        if(!moving)for(auto& t:tracks){t.motionSamples=0;t.movingLost=0;}
    }
    void update(const std::vector<cv::Rect>& boxes,const LaneV2& lane,double now,bool canStart){
        double elapsed=lastTime<0?0:std::max(0.,now-lastTime);
        double dt=lastTime<0?.05:clip(elapsed,0,.1);lastTime=now;realLane=lane;
        for(auto& t:tracks)if(driving&&canStart)t.movingLost+=elapsed;else{t.motionSamples=0;t.movingLost=0;}
        targetId=nextTargetId=side=index=nextSide=rawSide=nextRawSide=0;
        handoff=0;near=false;box=nextBox={};
        if(!searchFinished())associate(boxes,now,driving&&canStart);
        // Actual forward-command feedback is required. Waiting for a light or a
        // static preview must not complete a cone merely because a timer expired.
        if(driving&&canStart&&passed<cfg.cone_lane_count)for(auto& t:tracks){
            bool atExit=t.seen&&foot(t.box)>=478;
            bool lost=now-t.last>cfg.cone_lost_grace_s;
            bool observed=exitEligible(t,now)&&(atExit||lost);
            int predicted=predictedImageExit(t,now);
            if(!t.passed&&t.order>0&&t.order<=cfg.cone_lane_count&&(observed||predicted)){
                t.passed=true;
                if(passedOrders.insert(t.order).second){
                    ++passed;passReason=predicted&&!observed?(predicted==1?"predicted_side_exit":"predicted_bottom_exit"):atExit?"observed_bottom":"observed_near_exit";
                }
                nextPrimaryOrder=std::max(nextPrimaryOrder,t.order+1);
                if(passed>=cfg.cone_lane_count)break;
            }
        }
        tracks.erase(std::remove_if(tracks.begin(),tracks.end(),[&](const Track& t){
            return now-t.last>identityGrace()+.8&&t.id!=currentId;
        }),tracks.end());
        Track* first=find(currentId);
        if(!first||first->passed||!fresh(*first,now)||(first->order&&first->order<nextPrimaryOrder)){
            Track* previous=first;
            first=nullptr;currentId=0;
            // Prefer an already assigned physical cone over a fresh proposal.
            // Once cone 2 takes over, an old cone-1 track cannot take control back.
            if(passed<cfg.cone_lane_count&&nextPrimaryOrder<=cfg.cone_lane_count){
                for(auto& t:tracks)
                    if(!t.passed&&t.confirmed&&fresh(t,now)&&t.order>=nextPrimaryOrder&&
                       (!first||t.order<first->order||(t.order==first->order&&foot(t.box)>foot(first->box))))first=&t;
                if(!first)for(auto& t:tracks)
                    if(!t.passed&&t.confirmed&&fresh(t,now)&&!t.order&&(!first||foot(t.box)>foot(first->box)))first=&t;
            }
            if(first){
                // A distinct farther target can become cone 2 after a near,
                // laterally clear cone 1 disappears. Merely losing a far target
                // never advances the route or increments the passage count.
                if(!first->order&&previous&&previous->engaged&&!previous->passed&&
                   previous->order==nextPrimaryOrder&&nextPrimaryOrder<cfg.cone_lane_count&&
                   previous->maxY>=cfg.cone_full_avoid_y&&clearBeside(*previous)&&
                   foot(first->box)<previous->maxY-12&&
                   std::abs(cx(first->box)-cx(previous->box))>std::max(first->box.width,previous->box.width)*1.5)
                    commit(*first,nextPrimaryOrder+1);
                if(first->order)nextPrimaryOrder=std::max(nextPrimaryOrder,first->order);
                currentId=first->id;
            }else if(previous&&!previous->passed&&previous->order>=nextPrimaryOrder)
                currentId=previous->id; // retain identity/route evidence while the next proposal confirms
        }
        Track* second=nullptr;
        if(first&&nextPrimaryOrder<cfg.cone_lane_count){
            for(auto& t:tracks){
                if(t.id==first->id||t.passed||!t.confirmed||!fresh(t,now))continue;
                if(t.order==nextPrimaryOrder+1){second=&t;break;}
                if(!t.order&&foot(t.box)<=foot(first->box)-12&&(!second||foot(t.box)>foot(second->box)))second=&t;
            }
        }
        trackCount=0;for(const auto& t:tracks)if(t.confirmed&&!t.passed&&fresh(t,now))++trackCount;
        bool use=first&&canStart&&passed<cfg.cone_lane_count&&foot(first->box)>=cfg.cone_avoid_start_y;
        if(use){
            if(!first->order)commit(*first,nextPrimaryOrder);
            // Course is known to alternate left/right. Only cone 1 chooses its
            // side from the image; cone 2 uses the opposite committed side even
            // if turning or missing lane edges put both boxes in the same half.
            if(second)commit(*second,first->order+1);
            first->engaged=true;near=true;
            if(second&&foot(second->box)>=cfg.cone_avoid_start_y&&clearBeside(*first)){
                handoff=ease((foot(first->box)-cfg.cone_handoff_y)/(478-cfg.cone_handoff_y));
                if(handoff>0)second->engaged=true;
            }
        }
        if(first){targetId=first->id;box=first->box;side=first->side;index=first->order;rawSide=observedSide(*first);}
        if(second){nextTargetId=second->id;nextBox=second->box;nextSide=second->side;nextRawSide=observedSide(*second);}
        phase=passed>=cfg.cone_lane_count?"RECENTER":use?(handoff>0?"HANDOFF":side>0?"LANE_RIGHT":"LANE_LEFT"):
              first?"FAR":boxes.empty()?"SEARCH":"CONFIRM";
        if(!first&&firstPassSide&&!searchFinished()){
            index=std::min(nextPrimaryOrder,cfg.cone_lane_count);
            phase="SEARCH_C"+std::to_string(index)+"_OF_"+std::to_string(cfg.cone_lane_count);
        }
        if(searchFinished()&&passed<cfg.cone_lane_count)phase="END_UNCONFIRMED";
        if(searchFinished())index=cfg.cone_lane_count;
        if(use&&!first->seen)phase="OCCLUDED";
        // Keep near rows constrained by the first cone, and gradually bring the
        // farther cone into distant rows. No intermediate return-to-centre step.
        double sum=0,weights=0,widthSum=0;
        leftPath.clear();rightPath.clear();targetPath.clear();
        for(int i=10;i<86;i+=2){
            double y=lane.top+(lane.bottom-lane.top-1)*i/95.;
            double l=lane.leftAt(y),r=lane.rightAt(y),vl=l,vr=r;
            double w=std::min(8,1+std::min(i,95-i)/6);
            if(use){
                corridor(*first,y,vl,vr);
                if(second&&handoff>0){
                    double sl,sr;corridor(*second,y,sl,sr);
                    double q=handoff*(1-ease((y-foot(second->box))/std::max(12.,foot(first->box)-foot(second->box))));
                    vl+=(sl-vl)*q;vr+=(sr-vr)*q;
                }
                double strength=cfg.cone_start_blend+(1-cfg.cone_start_blend)*ease((foot(first->box)-cfg.cone_avoid_start_y)/std::max(1.,cfg.cone_full_avoid_y-cfg.cone_avoid_start_y));
                vl=l+(vl-l)*strength;vr=r+(vr-r)*strength;
            }
            sum+=((vl+vr-l-r)*.5)*w;widthSum+=std::max(1.,r-l)*w;weights+=w;
            leftPath.push_back({int(vl),int(y)});rightPath.push_back({int(vr),int(y)});
        }
        double wanted=sum/std::max(1.,widthSum);
        double change=(wanted-offset)*(1-std::exp(-dt/cfg.cone_smooth_seconds));
        offset+=clip(change,-cfg.cone_offset_rate*dt,cfg.cone_offset_rate*dt);
        if(!use&&std::abs(offset)<1e-4)offset=0;
        errorDelta=offset*(widthSum/std::max(1.,weights))/8.;
        if(completed())phase="DONE";
        for(const auto& p:leftPath){double l=lane.leftAt(p.y),r=lane.rightAt(p.y);targetPath.push_back({int((l+r)*.5+offset*(r-l)),p.y});}
    }
    void draw(cv::Mat& frame)const {
        for(const auto& t:tracks){
            // Retained association history is not a detection in this frame.
            // Show a missed target only during its short steering hold.
            if(searchFinished()||t.passed||(!t.seen&&!(t.id==targetId&&fresh(t,lastTime))))continue;
            cv::rectangle(frame,t.box,t.passed?cv::Scalar(128,128,128):cv::Scalar(255,180,0),1);
            std::string label;
            if(!t.confirmed)label="CAND "+std::to_string(t.hits)+"/"+std::to_string(cfg.cone_confirm_frames);
            else if(!t.order)label="CONFIRMED FAR";
            else label="C"+std::to_string(t.order)+(t.side>0?" PASS RIGHT":" PASS LEFT");
            if(!t.seen)label+=" HOLD";
            cv::putText(frame,label,{std::max(0,std::min(t.box.x,frame.cols-180)),std::max(12,t.box.y-5)},cv::FONT_HERSHEY_SIMPLEX,.4,{0,255,255},1);
        }
        if(controlling()){
            for(size_t i=1;i<leftPath.size();++i){
                cv::line(frame,leftPath[i-1],leftPath[i],{0,140,255},2);
                cv::line(frame,rightPath[i-1],rightPath[i],{0,140,255},2);
                cv::circle(frame,targetPath[i],2,{255,0,255},-1);
            }
        }
    }
};
