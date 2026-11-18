#pragma once
#include <opencv2/core.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

// 固定动作不使用巡线误差。steer 是相对绕行侧的 PWM 修正，seconds 为秒。
// side=+1 计划从右侧绕，side=-1 计划从左侧绕；舵机目标 = center - side * steer。
// 这是程序使用的方向约定，实际前轮方向取决于机械安装和舵机标定。
struct FixedStep {
    std::string name;
    double steer=0,seconds=0;
    int motor=10900;
};

struct FixedPlan {
    std::vector<FixedStep> steps;
    // 启动时从 YAML 读入动作表；运行途中不重新加载文件。
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
               !std::isfinite(s.steer)||std::abs(s.steer)>31||!std::isfinite(s.seconds)||s.seconds<=0||s.seconds>15||s.motor<5000||s.motor>12000)
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
        // 复制整张表并锁定本轮动作；启动帧尚未经过命令仲裁，所以 driving=false。
        plan=p;index=0;elapsed=0;last=now;started=true;driving=false;
    }
    bool active()const {return started&&index<plan.steps.size();}
    bool done()const {return started&&!active();}
    const FixedStep& step()const {return plan.steps.at(index);}
    size_t stepNumber()const {return index+1;}
    double remaining()const {return active()?std::max(0.,step().seconds-elapsed):0.;}
    // applied 来自 Suite 的最终命令仲裁，不是轮速、里程或舵机位置反馈。
    void feedback(bool applied) {driving=active()&&applied;}
    // 每处理一帧才调用一次：seconds 是换段阈值，不是独立硬件定时中断。
    // 例如阈值 0.25 秒，若前后检查在 0.24/0.30 秒，会到 0.30 秒才换段。
    // 同样的 PWM/秒数并不测量或保证同样的实际车速、位移和车头角度。
    void tick(double now) {
        if(!std::isfinite(now)||now<0||(last>=0&&now<last))throw std::runtime_error("Invalid fixed maneuver clock");
        double dt=last<0?0:now-last;last=now;
        // 只在上一帧最终命令仍选中本动作前进时累计帧时间差；交通等待暂停累计。
        if(active()&&driving) {
            elapsed+=dt;
            if(elapsed+1e-9>=step().seconds) {
                ++index;elapsed=0;driving=false;
                // 超出的时间不挪给下一段；一次只前进一步，卡顿也不会跳过尚未输出的段。
            }
        }
    }
    std::string phase()const {return active()?std::to_string(stepNumber())+"_"+step().name:"DONE";}
};
