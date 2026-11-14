#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// 固定动作不使用巡线误差。steer 是相对绕行侧的 PWM 修正，seconds 为秒。
// side=+1 从右侧绕，side=-1 从左侧绕；舵机目标 = center - side * steer。
struct FixedStep {
    std::string name;
    double steer=0,seconds=0;
    int motor=10900;
};

struct FixedPlan {
    std::vector<FixedStep> steps;
    void load(const cv::FileStorage& fs,const std::string& key) {
        auto list=fs[key];
        if(!list.isSeq()||list.empty()||list.size()>20)
            throw std::runtime_error(key+" must contain 1..20 steps");
        steps.clear();double total=0;
        for(const auto& node:list) {
            FixedStep s;
            if(!node.isMap()||!node["name"].isString()||
               !(node["steer"].isReal()||node["steer"].isInt())||
               !(node["seconds"].isReal()||node["seconds"].isInt())||!node["motor_pwm"].isInt())
                throw std::runtime_error(key+": each step requires name, steer, seconds, integer motor_pwm");
            node["name"]>>s.name;node["steer"]>>s.steer;node["seconds"]>>s.seconds;node["motor_pwm"]>>s.motor;
            if(s.name.empty()||s.name.size()>32||s.name.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_")!=std::string::npos||
               !std::isfinite(s.steer)||std::abs(s.steer)>31||!std::isfinite(s.seconds)||s.seconds<=0||s.seconds>15||s.motor<7000||s.motor>12000)
                throw std::runtime_error(key+": invalid step "+s.name);
            total+=s.seconds;steps.push_back(s);
        }
        if(total>60)throw std::runtime_error(key+": total duration exceeds 60 seconds");
    }
    void validateMotor(int neutral,const std::string& key) const {
        for(const auto& s:steps)if(s.motor<=neutral)
            throw std::runtime_error(key+": motor_pwm must be greater than speed.neutral_pwm");
    }
};

class FixedManeuver {
    FixedPlan plan;
    size_t index=0;
    double elapsed=0,last=-1;
    bool started=false,driving=false;
public:
    void start(const FixedPlan& p,double now) {
        if(p.steps.empty())throw std::runtime_error("Cannot start empty fixed maneuver");
        plan=p;index=0;elapsed=0;last=now;started=true;driving=false;
    }
    bool active()const {return started&&index<plan.steps.size();}
    bool done()const {return started&&!active();}
    const FixedStep& step()const {return plan.steps.at(index);}
    size_t stepNumber()const {return index+1;}
    double remaining()const {return active()?std::max(0.,step().seconds-elapsed):0.;}
    void feedback(bool applied) {driving=active()&&applied;}
    void tick(double now) {
        if(!std::isfinite(now)||now<0||(last>=0&&now<last))throw std::runtime_error("Invalid fixed maneuver clock");
        double dt=last<0?0:now-last;last=now;
        // 只累计上一帧仲裁后实际发出本动作前进命令的时间，红灯/斑马线等待不耗时。
        if(active()&&driving) {
            elapsed+=dt;
            if(elapsed+1e-9>=step().seconds) {
                ++index;elapsed=0;driving=false;
                // 卡顿时上一段实际被保持；不能跳过尚未输出的后续段。
            }
        }
    }
    std::string phase()const {return active()?std::to_string(stepNumber())+"_"+step().name:"DONE";}
};
