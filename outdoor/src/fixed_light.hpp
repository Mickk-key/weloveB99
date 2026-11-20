#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

// all 的默认灯阶段只按固定动作计时，不读取灯色；部分模块仍使用原识别器。
struct FixedLightConfig {
    int enabled=1,cruisePwm=11000;
    double cruiseSeconds=10.,stopSeconds=10.;
    void load(const cv::FileStorage& fs,int neutral){
#define F(key,field) if(!fs[key].empty())fs[key]>>field
        F("all_light_fixed_enabled",enabled);
        // 兼容上一版配置名；新名称优先，动作始终为巡线前进。
        F("all_light_straight_pwm",cruisePwm);
        F("all_light_straight_seconds",cruiseSeconds);
        F("all_light_cruise_pwm",cruisePwm);
        F("all_light_cruise_seconds",cruiseSeconds);
        F("all_light_stop_seconds",stopSeconds);
#undef F
        if((enabled!=0&&enabled!=1)||cruisePwm<=neutral||cruisePwm>12000||
           !std::isfinite(cruiseSeconds)||cruiseSeconds<0||cruiseSeconds>60||
           !std::isfinite(stopSeconds)||stopSeconds<0||stopSeconds>60)
            throw std::runtime_error("Invalid all_light_fixed settings (seconds: 0..60, PWM: neutral+1..12000)");
    }
};

class FixedLightSequence {
    enum Phase { WAIT_STAGE,CRUISE,BRAKE,HOLD,DONE } phase_=WAIT_STAGE;
    double last_=-1,elapsed_=0,brakeUntil_=-1,holdSince_=-1;
public:
    // 巡线只累计上一帧实际输出前进命令的时间。制动单独限时；
    // HOLD 从首次输出中位的这一帧开始，保证完整等待 stopSeconds 后再走。
    void update(double now,bool allowed,bool wasMoving,const FixedLightConfig& c,double brakeSeconds){
        if(!allowed||done())return;
        double dt=last_<0?0:std::max(0.,now-last_);last_=now;
        if(phase_==WAIT_STAGE){phase_=CRUISE;elapsed_=0;dt=0;}
        if(phase_==CRUISE){
            if(wasMoving)elapsed_+=dt;
            if(elapsed_+1e-9<c.cruiseSeconds)return;
            phase_=BRAKE;brakeUntil_=now+(wasMoving?brakeSeconds:0.);
        }
        if(phase_==BRAKE){
            if(now+1e-9<brakeUntil_)return;
            phase_=HOLD;holdSince_=now;
        }
        if(phase_==HOLD&&now-holdSince_+1e-9>=c.stopSeconds)phase_=DONE;
    }
    bool cruising()const{return phase_==CRUISE;}
    bool braking()const{return phase_==BRAKE;}
    bool holding()const{return phase_==HOLD;}
    bool active()const{return cruising()||braking()||holding();}
    bool done()const{return phase_==DONE;}
    const char* name()const{
        return phase_==WAIT_STAGE?"WAIT_STAGE":cruising()?"FIXED_CRUISE":
               braking()?"FIXED_BRAKE":holding()?"FIXED_HOLD":"DONE";
    }
    double brakeRemaining(double now)const{return braking()?std::max(0.,brakeUntil_-now):0.;}
    double remaining(double now,const FixedLightConfig& c)const{
        return cruising()?std::max(0.,c.cruiseSeconds-elapsed_):braking()?brakeRemaining(now):
               holding()?std::max(0.,c.stopSeconds-(now-holdSince_)):0.;
    }
};
