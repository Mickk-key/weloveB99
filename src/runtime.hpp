// 公共运行与硬件层：读取标定、计算 PD/分段油门，管理 GPIO、制动截止时间和退出清理。
// Output 是本程序唯一发送电机/转向指令的位置；相机采集本身由 main.cpp 管理。
#pragma once
// White lane + blue cone v2.1: bounded diagnostics outside control path.
#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <deque>
#include <sstream>
#include "async_console.hpp"
#include <unistd.h>
#include <limits.h>
#include <sys/stat.h>
using namespace std;using namespace cv;
#include "lane_v2.hpp"
#include "cone_v2.hpp"
int lane_left_count=0,lane_right_count=0;
// Keep the existing camera, GPIO calibration, timed throttle and neutral cleanup.
// Runtime owns the camera and GPIO. Single-side image-edge fallback is preserved.
// PWM 数值按现有硬件标定使用：电机中位默认 10000，舵机中心默认 80。
// 默认值会被 YAML 覆盖；watchdog_ms 为毫秒，steer_rate 为每秒舵机 PWM 变化量。
struct Settings {
    double pd_limit=15;
    int control_watchdog_enabled=0,control_watchdog_ms=300;
    double cone_pd_kp=.35,cone_pd_kd=.2,cone_min_correction=1.;
    string camera_device;int width=640,height=480,set_auto_exposure=3,set_exposure=1500;
    int camera_home_enabled=0,camera_pan_pwm=75,camera_tilt_pwm=65;
    double lane_pd_kp=.2,lane_pd_kd=.5,servo_center=80,servo_min=55,servo_max=86;
    int neutral_pwm=10000,start_pwm=11000,corner_pwm=11500,corner_fast_pwm=12000,corner_brake_pwm=7500,after_corner_pwm=10900;
    int gear=1,start_s=2,corner_end_s=27,corner_brake_s=1;
    int cone_slow_pwm=10900;double steer_rate=40;
};
// 先加载顶层相机/控制参数，再读取 speed 子映射；随后检查尺寸、范围和时序关系。
// 该函数只读取配置，尚未初始化 GPIO 或相机。
Settings load_settings(const string& path){
    Settings c;FileStorage f(path,FileStorage::READ);if(!f.isOpened())throw runtime_error("Cannot read "+path);
#define R(n) if(!f[#n].empty())f[#n]>>c.n
    R(camera_device);R(width);R(height);R(set_auto_exposure);R(set_exposure);R(camera_home_enabled);R(camera_pan_pwm);R(camera_tilt_pwm);
    R(control_watchdog_enabled);R(control_watchdog_ms);R(cone_pd_kp);R(cone_pd_kd);R(cone_min_correction);R(pd_limit);R(cone_slow_pwm);R(steer_rate);R(lane_pd_kp);R(lane_pd_kd);R(servo_center);R(servo_min);R(servo_max);
#undef R
    auto speeds=f["speed"];if(!speeds.isMap())throw runtime_error("Missing speed mapping");
#define S(n) if(!speeds[#n].empty())speeds[#n]>>c.n
    S(neutral_pwm);S(start_pwm);S(corner_pwm);S(corner_fast_pwm);S(corner_brake_pwm);S(after_corner_pwm);
    S(gear);S(start_s);S(corner_end_s);S(corner_brake_s);
#undef S
    if(c.camera_device.empty()||c.width!=640||c.height!=480)throw runtime_error("Original lane requires configured camera and 640x480");
    for(int p:{c.neutral_pwm,c.start_pwm,c.corner_pwm,c.corner_fast_pwm,c.corner_brake_pwm,c.after_corner_pwm})
        if(p<7000||p>12000)throw runtime_error("PWM outside 7000..12000");
    if(!isfinite(c.lane_pd_kp)||!isfinite(c.lane_pd_kd)||c.lane_pd_kp<0||c.lane_pd_kd<0||c.servo_min<55||c.servo_max>86||!(c.servo_min<=c.servo_center&&c.servo_center<=c.servo_max))throw runtime_error("Invalid PD/steering settings");
    if((c.gear!=1&&c.gear!=2&&c.gear!=10)||c.start_s<0||c.corner_end_s<c.start_s||c.corner_brake_s<0)throw runtime_error("Invalid original throttle timing");
    if((c.camera_home_enabled!=0&&c.camera_home_enabled!=1)||c.camera_pan_pwm<50||c.camera_pan_pwm>100||c.camera_tilt_pwm<50||c.camera_tilt_pwm>100)throw runtime_error("Camera home PWM must be in 50..100");
    if(c.cone_slow_pwm<c.neutral_pwm||c.cone_slow_pwm>12000||!isfinite(c.steer_rate)||c.steer_rate<=0||c.steer_rate>200)throw runtime_error("Invalid cone speed/steering slew");
    if(!isfinite(c.pd_limit)||c.pd_limit<=0||c.pd_limit>15)throw runtime_error("pd_limit must be in (0,15]");
    if((c.control_watchdog_enabled!=0&&c.control_watchdog_enabled!=1)||c.control_watchdog_ms<50||c.control_watchdog_ms>10000)throw runtime_error("Invalid control watchdog settings");
    if(!isfinite(c.cone_pd_kp)||!isfinite(c.cone_pd_kd)||!isfinite(c.cone_min_correction)||c.cone_pd_kp<0||c.cone_pd_kd<0||c.cone_min_correction<0||c.cone_min_correction>c.pd_limit)throw runtime_error("Invalid cone PD settings");
    return c;
}
// Original live PD: previous error starts at zero; missing boundaries do not reset it.
// 离散 PD：Kp×当前误差 + Kd×相邻帧误差差值；这里没有再除以 dt。
// 先限制修正量，再按 servo_center-correction 换算舵机 PWM 并限制机械范围。
struct OriginalPd {
    double previous=0;
    double update(double error,const Settings& c,bool avoiding=false){
        const double kp=avoiding?c.cone_pd_kp:c.lane_pd_kp;
        const double kd=avoiding?c.cone_pd_kd:c.lane_pd_kd;
        double correction=kp*error+kd*(error-previous);
        // Only compensate a nonzero correction that agrees with the current target.
        // Do not force a turn against the lane error or suppress derivative braking.
        // 最小转向补偿仅用于绕桶，且要求修正方向与当前误差一致，避免覆盖反向的 D 项。
        if(avoiding&&std::abs(error)>.25&&correction*error>0)
            correction=std::copysign(std::max(std::abs(correction),c.cone_min_correction),correction);
        correction=max(-c.pd_limit,min(c.pd_limit,correction));
        previous=error;return max(c.servo_min,min(c.servo_max,c.servo_center-correction));
    }
};
// 分段油门：起步 → 按 gear 选弯道 PWM → 可选定时制动 → 后续 PWM。
// 入参毫秒先整除为秒；corner_brake_s=0 时该定时制动区间为空。
int lane_throttle(long long driving_ms,const Settings& c){
    auto seconds=driving_ms/1000;
    if(seconds<c.start_s)return c.start_pwm;
    if(seconds<c.corner_end_s)return c.gear==1?c.corner_pwm:c.gear==2?c.corner_fast_pwm:c.start_pwm;
    if(seconds<c.corner_end_s+c.corner_brake_s)return c.corner_brake_pwm;
    return c.after_corner_pwm;
}
// 信号处理只置位；主循环和监控线程读取标志，再执行正常退出和中位清理。
volatile sig_atomic_t interrupted=0;
void on_signal(int){interrupted=1;}
using Clock=chrono::steady_clock;
const char* phase_name(int p){
    switch(p){case 1:return "CAPTURE";case 2:return "LANE";case 3:return "CONE";case 4:return "CONTROL";case 5:return "PREVIEW";default:return "IDLE";}
}
// 输出对象持有硬件生命周期；enabled=false 时仍维护状态，但不发送 GPIO PWM。
// 同一把互斥锁保护主线程 send 与监控线程的停车操作，避免同时写电机。
struct Output {
    Settings c;bool enabled=false,gpio_active=false,gimbal_active=false;mutex lock;atomic<bool> done{false},brake_done{false};atomic<int> aborted{0};
    atomic<int> phase{0},stale_phase{0},stale_ms{0};
    bool moving=false,braking=false;string activeBrakeState;Clock::time_point last=Clock::now(),deadline;thread watcher;
    void neutral(){if(enabled){gpioPWM(13,c.neutral_pwm);gpioPWM(12,int(c.servo_center));}moving=false;braking=false;}
    Output(const Settings& settings,bool motion,bool home_camera=false):c(settings){
        try{
        // 只有显式运动或回位请求才初始化 pigpio；先拒绝与现有 GPIO 控制进程同时运行。
        // GPIO12 是转向，GPIO13 是电机，各自 PWM 频率和量程按现有标定设置。
        if(motion||home_camera){
            //if(system("pgrep -f '/network-rc/lib/[g]pio.js|[p]igpiod' >/dev/null")==0)throw runtime_error("Stop other GPIO controllers first");
            if(gpioInitialise()<0)throw runtime_error("GPIO init failed");gpio_active=true;enabled=motion;
            if(motion){
                gpioSetMode(12,PI_OUTPUT);gpioSetPWMfrequency(12,50);gpioSetPWMrange(12,1000);
                gpioSetMode(13,PI_OUTPUT);gpioSetPWMfrequency(13,200);gpioSetPWMrange(13,40000);neutral();
            }
            // Preserve the user-set camera pose by default. Explicit camera homing is opt-in.
            // 支架回位只在此条件成立时触发 GPIO22/23；默认 camera_home_enabled=0 保留手调姿态。
            if(home_camera||(motion&&c.camera_home_enabled)){
            // Restore /home/pi/11.11/1.cpp gimbal_init sequence; home PWM is separately configurable for the installed bracket.
            gimbal_active=true;
            auto axis=[](unsigned pin,unsigned duty){
                if(gpioSetMode(pin,PI_OUTPUT)<0||gpioSetPWMfrequency(pin,50)!=50||gpioSetPWMrange(pin,1000)<0||gpioPWM(pin,duty)<0)
                    throw runtime_error("Camera gimbal initialization failed");
                for(int i=0;i<100&&!interrupted;++i)this_thread::sleep_for(chrono::milliseconds(10));
            };
            cout<<"[CAMERA_HOME] GPIO22="<<c.camera_pan_pwm<<" GPIO23="<<c.camera_tilt_pwm<<" frequency=50 range=1000 | "<<(motion?"drive mode, motor neutral during initialization":"CAMERA ONLY: no motor/steering commands")<<endl;
            axis(22,c.camera_pan_pwm);if(!interrupted)axis(23,c.camera_tilt_pwm);
            } else cout<<"[CAMERA] preserving manually set pose; no GPIO22/23 commands"<<endl;
        }
        // 每约 5 毫秒检查退出信号、可选控制超时和反向制动截止时间。
        // 看门狗开关只控制更新超时检查；制动脉冲到期归中位不依赖这个开关。
        watcher=thread([this]{while(!done){
            {lock_guard<mutex> guard(lock);auto now=Clock::now();
             int reason=interrupted?3:c.control_watchdog_enabled&&moving&&now-last>chrono::milliseconds(c.control_watchdog_ms)?2:0;
             if(reason){if(!aborted){stale_phase=phase.load();stale_ms=int(chrono::duration_cast<chrono::milliseconds>(now-last).count());aborted=reason;}neutral();}
             else if(braking&&now>=deadline){neutral();brake_done=true;}}
            this_thread::sleep_for(chrono::milliseconds(5));
        }});}catch(...){neutral();if(gimbal_active){gpioPWM(22,0);gpioPWM(23,0);}if(gpio_active)gpioTerminate();throw;}
    }
    // 每帧刷新控制心跳；若此前已经中止或本帧发现更新超时，立即归中位并停止主循环。
    // brakeRemaining>=0 且 motor<neutral_pwm 才作为有截止时间的反向制动处理。
    bool send(int motor,int steer,const string& state,double brakeRemaining=-1){
        lock_guard<mutex> guard(lock);if(aborted||interrupted){neutral();return false;}
        auto now=Clock::now();
        if(c.control_watchdog_enabled&&moving&&now-last>chrono::milliseconds(c.control_watchdog_ms)){
            stale_phase=phase.load();stale_ms=int(chrono::duration_cast<chrono::milliseconds>(now-last).count());aborted=2;neutral();return false;
        }
        last=now;
        bool timedBrake=motor<c.neutral_pwm&&brakeRemaining>=0;
        if(timedBrake){
            auto requested=now+chrono::milliseconds(std::max(0LL,std::llround(brakeRemaining*1000)));
            // 同一制动状态只允许缩短已定的截止时间，重复帧不能把反向脉冲不断续长。
            // 截止已到或监控线程已结束该脉冲时，维持中位，直到切换到其他命令状态。
            if(activeBrakeState!=state){activeBrakeState=state;brake_done=false;deadline=requested;}
            else if(brake_done||now>=deadline){neutral();brake_done=true;return true;}
            else deadline=std::min(deadline,requested); // A slow/repeated frame cannot extend the pulse.
        }else{activeBrakeState.clear();brake_done=false;}
        moving=motor!=c.neutral_pwm;braking=timedBrake;
        if(enabled){gpioPWM(13,motor);gpioPWM(12,steer);}return true;
    }
    // 先停止并等待监控线程，再归中位和释放 GPIO；析构也调用此函数用于异常收尾。
    // 只有本对象初始化过的 GPIO 才会被终止。
    void finish(){done=true;if(watcher.joinable())watcher.join();neutral();if(gpio_active){if(enabled)this_thread::sleep_for(chrono::milliseconds(300));if(gimbal_active){gpioPWM(22,0);gpioPWM(23,0);}gpioTerminate();gpio_active=false;enabled=false;gimbal_active=false;}}
    ~Output(){finish();}
};
// 图片先写临时文件，再重命名发布，供需要完整图片快照的调用方使用。
void save_image(const Mat& image,const string& path){
    if(!imwrite(path+".tmp.jpg",image)||rename((path+".tmp.jpg").c_str(),path.c_str()))throw runtime_error("Cannot publish image");
}
// 创建输出前缀的父目录；拒绝绝对路径和包含 .. 的路径，限制输出在当前项目下。
void parent_dirs(const string& path){
    if(path.empty()||path[0]=='/'||path.find("..")!=string::npos)throw runtime_error("--out must be relative inside project");
    for(size_t i=path.find('/');i!=string::npos;i=path.find('/',i+1))if(mkdir(path.substr(0,i).c_str(),0755)&&errno!=EEXIST)throw runtime_error("Cannot create output folder");
}
// 保留的旧配置定位辅助函数；当前集成入口在 main.cpp 按 env 选择 config/<env>.yaml。
string default_config(){char p[PATH_MAX];auto n=readlink("/proc/self/exe",p,sizeof(p)-1);if(n<0)throw runtime_error("Cannot locate executable");p[n]=0;string s=p;return s.substr(0,s.find_last_of('/'))+"/../config.yaml";}
