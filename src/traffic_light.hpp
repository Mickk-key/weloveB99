#pragma once
#include "traffic_light_detector.h"
#include "light_night_detector.hpp"
#include "light_sequence.hpp"
#include <opencv2/opencv.hpp>
#include <string>

// 识别器只给结果，停车器只管时序，Suite负责最终电机仲裁；本模块不接触GPIO。
struct TrafficControlConfig {
    LightSequenceConfig sequence;
    int brakePwm=8000;
    double brakeSeconds=.8,greenSeconds=.5;
    void load(const cv::FileStorage& fs,int neutral){
        sequence.afterZebra=1;sequence.load(fs,neutral);
        if(!fs["light_brake_pwm"].empty())fs["light_brake_pwm"]>>brakePwm;
        if(!fs["light_brake_seconds"].empty())fs["light_brake_seconds"]>>brakeSeconds;
        if(!fs["light_green_seconds"].empty())fs["light_green_seconds"]>>greenSeconds;
        if(brakePwm<7000||brakePwm>neutral||!std::isfinite(brakeSeconds)||brakeSeconds<0||brakeSeconds>2||
           !std::isfinite(greenSeconds)||greenSeconds<0||greenSeconds>10)
            throw std::runtime_error("Invalid traffic-light brake/green settings");
    }
};

struct TrafficObservation {
    int state=0,hits=0,red=0,green=0; // -1红、0未知、1绿；hits来自原检测器。
    bool confirmed=false,panelFound=false;
    std::string evidence="none";
    cv::Rect box,roi;
};

// 默认：用户原文件 + 默认参数 + 整帧，不额外筛框、改阈值或重做目标确认。
// 夜间：V2颜色/亮芯算法，直接判断发光，不要求灯罩。两个识别器共用同一停车流程。
class TrafficRecognizer {
    bool nightMode;
    TrafficLightDetector supplied;
    NightLightDetector night;
public:
    static void checkNightConfig(){
        cv::FileStorage fs("config/light_night_v2.yaml",cv::FileStorage::READ);
        if(!fs.isOpened())throw std::runtime_error("Missing config/light_night_v2.yaml");
    }
    explicit TrafficRecognizer(bool nightMode=false):nightMode(nightMode){
        if(nightMode){checkNightConfig();night.load("config/light_night_v2.yaml");}
    }
    TrafficObservation detect(const cv::Mat& frame){
        TrafficObservation out;
        if(nightMode){
            out.state=night.detect(frame);out.confirmed=out.state!=0;out.hits=out.confirmed?1:0;
            out.red=night.red;out.green=night.green;out.roi=night.roi;
            out.evidence=out.state<0?"v2_red_pixels":out.state>0?"v2_green_core":"none";
        }else{
            TrafficLightConfig defaults;
            const auto result=supplied.detect(frame,defaults);
            out.state=result.state==TrafficLightState::Red?-1:result.state==TrafficLightState::Green?1:0;
            out.confirmed=result.stateConfirmed;out.hits=result.confirmationHits;out.evidence=result.lightEvidence;
            out.panelFound=result.panelFound;out.roi={0,0,frame.cols,cvRound(frame.rows*defaults.roiBottom)};
            if(result.panelFound){
                std::vector<cv::Point2f> points(result.panelCorners.begin(),result.panelCorners.end());
                out.box=cv::boundingRect(points); // 仅显示位置，不用框或y再次筛选结果。
            }
        }
        return out;
    }
};

class TrafficStop {
    bool stopped=false,finished=false;
    double brakeUntil=-1,greenSince=-1,lastSample=-1;
public:
    bool active()const{return stopped;}
    bool done()const{return finished;}
    bool braking(double now)const{return stopped&&now<brakeUntil;}
    double brakeRemaining(double now)const{return braking(now)?brakeUntil-now:0.;}
    std::string phase(double now)const{return finished?"DONE":braking(now)?"BRAKE":stopped?"WAIT_GREEN":"SEARCH";}
    void update(const TrafficObservation& seen,double now,bool mayStart,bool wasMoving,const TrafficControlConfig& c){
        if(finished)return;
        // 未采样的时间不能算作一直看到了绿灯；只在新检测时更新本状态机。
        if(lastSample>=0&&now-lastSample>1.)greenSince=-1;
        lastSample=now;
        if(!seen.confirmed||seen.state!=1)greenSince=-1;
        if(!stopped&&mayStart&&seen.confirmed&&seen.state==-1){
            stopped=true;brakeUntil=now+(wasMoving?c.brakeSeconds:0.);
        }
        // 红灯反复出现不延长制动。未知/丢灯保持停车，不设置10秒自动放行。
        if(stopped&&mayStart&&seen.confirmed&&seen.state==1){
            if(greenSince<0)greenSince=now;
            if(now-greenSince+1e-9>=c.greenSeconds&&now>=brakeUntil){stopped=false;finished=true;}
        }
    }
};

struct TrafficStatus {
    TrafficObservation observation;
    std::string mode="disabled",phase="DISABLED",sequence="DISABLED";
    bool detectionRan=false;int detectionCalls=0;
    double sequenceRemaining=0;
};

class TrafficLightControl {
    TrafficControlConfig cfg;
    bool nightMode;
    TrafficRecognizer recognizer;
    LightSequence sequence;
    TrafficStop stop;
    TrafficObservation seen;
    bool sampled=false;
public:
    TrafficLightControl(TrafficControlConfig c,bool night):cfg(c),nightMode(night),recognizer(night){}
    void update(const cv::Mat& frame,double now,bool zebraEnabled,bool zebraDone,bool wasMoving){
        sequence.update(now,zebraEnabled,zebraDone,stop.done(),cfg.sequence);
        sampled=!stop.done()&&sequence.detectNow(now,cfg.sequence);
        if(sampled){seen=recognizer.detect(frame);stop.update(seen,now,now>=1,wasMoving,cfg);}
        if(stop.done())sequence.update(now,zebraEnabled,zebraDone,true,cfg.sequence);
    }
    void feedback(bool forward,bool slow){sequence.feedback(forward,slow);}
    bool slowing()const{return sequence.slowing();}
    bool active()const{return stop.active();}
    bool braking(double now)const{return stop.braking(now);}
    double brakeRemaining(double now)const{return stop.brakeRemaining(now);}
    TrafficStatus status(double now)const{
        TrafficStatus s;s.observation=seen;s.mode=nightMode?"night_v2":"signal_color";
        s.phase=stop.phase(now);s.sequence=sequence.name();s.sequenceRemaining=sequence.remaining(cfg.sequence);
        s.detectionRan=sampled;s.detectionCalls=sequence.detectionCalls;return s;
    }
    void draw(cv::Mat& view,const TrafficStatus& s)const{
        const auto& o=s.observation;
        if(!o.roi.empty())cv::rectangle(view,o.roi,{0,200,255},1);
        if(o.panelFound)cv::rectangle(view,o.box,{255,255,0},2);
        cv::rectangle(view,{0,78,640,40},{0,0,0},-1);
        const std::string label=std::string(nightMode?"NIGHT_V2 ":"SIGNAL ")+(o.state<0?"RED":o.state>0?"GREEN":"UNKNOWN")+(o.confirmed?" OK":" ?");
        cv::putText(view,label,{5,94},cv::FONT_HERSHEY_SIMPLEX,.45,{255,255,255},1);
        cv::putText(view,s.sequence+" / "+s.phase+cv::format(" %.1fs",s.sequenceRemaining),{5,112},cv::FONT_HERSHEY_SIMPLEX,.45,{0,255,255},1);
    }
};
