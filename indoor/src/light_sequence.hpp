#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

// 只负责何时启用红绿灯检测、何时降低前进油门；不替代巡线和停车仲裁。
struct LightSequenceConfig {
    int afterZebra=0,slowPwm=10700,detectIntervalMs=100;
    double cruiseSeconds=3.,slowSettleSeconds=.3;
    void load(const cv::FileStorage& f,int neutral){
#define L(key,field) if(!f[key].empty())f[key]>>field
        L("light_after_zebra_enabled",afterZebra);
        L("light_after_zebra_seconds",cruiseSeconds);
        L("light_slow_pwm",slowPwm);
        L("light_slow_settle_seconds",slowSettleSeconds);
        L("light_detect_interval_ms",detectIntervalMs);
#undef L
        if((afterZebra!=0&&afterZebra!=1)||!std::isfinite(cruiseSeconds)||cruiseSeconds<0||cruiseSeconds>60||
           !std::isfinite(slowSettleSeconds)||slowSettleSeconds<0||slowSettleSeconds>5||
           slowPwm<=neutral||slowPwm>12000||detectIntervalMs<0||detectIntervalMs>500)
            throw std::runtime_error("Invalid light sequence settings");
    }
};

class LightSequence {
    enum Phase { STARTUP,WAIT_ZEBRA,CRUISE,SLOWDOWN,SEARCH,DONE } phase=STARTUP;
    double elapsed=0,last=-1,lastDetect=-1;
    bool forwardFeedback=false,slowFeedback=false;
public:
    int detectionCalls=0;
    // forward/slow 来自上一帧仲裁后的实际命令。停车等待不算“已经前进几秒”。
    void feedback(bool forward,bool slow){forwardFeedback=forward;slowFeedback=slow;}
    void update(double now,bool zebraEnabled,bool zebraDone,bool lightDone,const LightSequenceConfig& c){
        double dt=last<0?0:std::max(0.,now-last);last=now;
        if(lightDone){phase=DONE;return;}
        if(now<1)return;
        if(phase==STARTUP){phase=c.afterZebra&&zebraEnabled?WAIT_ZEBRA:SLOWDOWN;elapsed=0;dt=0;}
        if(phase==WAIT_ZEBRA){
            if(!zebraDone)return;
            phase=CRUISE;elapsed=0;dt=0; // 从斑马线等待结束这帧开始，不从识别斑马线那一刻开始。
        }
        if(phase==CRUISE){
            if(forwardFeedback)elapsed+=dt;
            if(elapsed+1e-9<c.cruiseSeconds)return;
            phase=SLOWDOWN;elapsed=0;dt=0;
        }
        if(phase==SLOWDOWN){
            if(slowFeedback)elapsed+=dt;
            if(elapsed+1e-9<c.slowSettleSeconds)return;
            phase=SEARCH;
        }
    }
    bool slowing()const{return phase==SLOWDOWN||phase==SEARCH;}
    bool searching()const{return phase==SEARCH;}
    std::string name()const{
        return phase==STARTUP?"STARTUP":phase==WAIT_ZEBRA?"WAIT_ZEBRA":phase==CRUISE?"CRUISE":
               phase==SLOWDOWN?"SLOWDOWN":phase==SEARCH?"SEARCH":"DONE";
    }
    double remaining(const LightSequenceConfig& c)const{
        return phase==CRUISE?std::max(0.,c.cruiseSeconds-elapsed):
               phase==SLOWDOWN?std::max(0.,c.slowSettleSeconds-elapsed):0.;
    }
    // 最多按配置频率跑昂贵检测。跳过的帧只继续巡线，不重复使用旧框累计 hits 或触发停车。
    bool detectNow(double now,const LightSequenceConfig& c){
        if(!searching()||(lastDetect>=0&&now-lastDetect+1e-9<c.detectIntervalMs*.001))return false;
        lastDetect=now;++detectionCalls;return true;
    }
};
