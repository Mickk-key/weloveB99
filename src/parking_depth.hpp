// 可选的几何深度停车后端，由 parking_mode=depth 选择；qr 模式不执行此停车状态机。
// 它依靠相机平面投影、入口线和 PWM 行程估计，没有测距传感器或编码器实测反馈。
// 米制几何与速度映射依赖配置标定，不能将估算距离当作实测距离。
#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include "ground_vision.h"

namespace depth_parking {
// 此命名空间内保留旧停车入口所用的巡线标定结构；不要与全局室内 LaneConfig 混淆。
// 当前 Suite 直接从 drive 同步 PD/舵机/中位等字段，未调用下方固定标定 load。
struct LaneConfig {
    double lane_pd_kp=.2,lane_pd_kd=.5,servo_center=80,servo_min=55,servo_max=86;
    int neutral_pwm=10000,start_pwm=11000,corner_pwm=11500,corner_fast_pwm=12000,after_corner_pwm=10900;
    int gear=1,start_s=2,corner_end_s=27,zebra_brake_pwm=9000,zebra_test_brake_ms=800;
    int set_auto_exposure=3,set_exposure=1500;
    std::string camera_device;
    void load(const std::string& path) {
        cv::FileStorage f(path,cv::FileStorage::READ);if(!f.isOpened())throw std::runtime_error("Cannot read frozen lane config: "+path);
#define R(n) f[#n]>>n
        R(lane_pd_kp);R(lane_pd_kd);R(servo_center);R(servo_min);R(servo_max);
        R(zebra_brake_pwm);R(zebra_test_brake_ms);R(camera_device);
        R(set_auto_exposure);R(set_exposure);
#undef R
        auto s=f["speed"];
#define R(n) s[#n]>>n
        R(neutral_pwm);R(start_pwm);R(corner_pwm);R(corner_fast_pwm);R(after_corner_pwm);R(gear);R(start_s);R(corner_end_s);
#undef R
        if(lane_pd_kp!=.2||lane_pd_kd!=.5||servo_center!=80||servo_min!=55||servo_max!=86||neutral_pwm!=10000||zebra_brake_pwm!=9000||zebra_test_brake_ms!=800)
            throw std::runtime_error("Protected calibration differs; review baseline before running");
    }
};
// Verbatim OriginalPd and throttle expressions from the frozen outdoor entry.
// 深度停车内部的离散 PD 保留既有修正限幅和舵机机械范围，D 项同样是相邻误差差值。
struct OriginalPd {
    double previous=0;
    double update(double error,const LaneConfig& c){
        double correction=std::max(-15.0,std::min(15.0,c.lane_pd_kp*error+c.lane_pd_kd*(error-previous)));
        previous=error;return std::max(c.servo_min,std::min(c.servo_max,c.servo_center-correction));
    }
};
// 旧深度控制器的 SEEK 油门分段辅助函数，没有主 runtime 中的 corner_brake 阶段。
inline int lane_throttle(long long driving_ms,const LaneConfig& c){
    auto seconds=driving_ms/1000;
    if(seconds<c.start_s)return c.start_pwm;
    if(seconds<c.corner_end_s)return c.gear==1?c.corner_pwm:c.gear==2?c.corner_fast_pwm:c.start_pwm;
    return c.after_corner_pwm;
}
// 长度/宽度/深度/前视距离为米，pitch/hfov 为角度；maxSeconds 为秒。
// speedScale 把正向 PWM 差值近似换成米/秒，steerScale 把舵机修正换成弧度。
struct ParkingConfig {
    GroundConfig ground;
    double wheelbase=.26,length=.4,width=.28,speedScale=.0005,steerScale=.032;
    double bayLength=1.5,depth=.75,lookahead=.65,brakeDistance=.07,maxSeconds=45,maxBlind=1.2;
    bool watchdogEnabled=false;
    int approachPwm=10900,enterPwm=10900,boardFrames=3,entryFrames=3,blueSMin=140;
    void load(const std::string& p){
        cv::FileStorage f(p,cv::FileStorage::READ);if(!f.isOpened())throw std::runtime_error("Cannot read parking config");
#define R(k,v) if(!f[k].empty()) f[k]>>v
        R("camera_height_m",ground.cameraHeight);R("camera_forward_m",ground.cameraForward);
        R("camera_pitch_deg",ground.pitchDegrees);R("camera_hfov_deg",ground.horizontalFov);R("lane_width_m",ground.laneWidth);
        R("wheelbase_m",wheelbase);R("vehicle_length_m",length);R("vehicle_width_m",width);
        R("pwm_speed_scale",speedScale);R("steering_rad_per_pwm",steerScale);
        R("parking_length_m",bayLength);R("parking_depth_m",depth);R("parking_lookahead_m",lookahead);
        R("parking_approach_pwm",approachPwm);R("parking_enter_pwm",enterPwm);
        R("parking_board_confirm_frames",boardFrames);R("parking_entry_confirm_frames",entryFrames);
        R("parking_blue_s_min",blueSMin);
        R("parking_brake_distance_m",brakeDistance);R("parking_max_seconds",maxSeconds);R("parking_max_blind_m",maxBlind);
#undef R
        for(double x:{ground.cameraHeight,ground.cameraForward,ground.pitchDegrees,ground.horizontalFov,ground.laneWidth,wheelbase,length,width,speedScale,steerScale,bayLength,depth,lookahead,brakeDistance,maxSeconds,maxBlind})
            if(!std::isfinite(x))throw std::runtime_error("Non-finite parking calibration");
        if(ground.cameraHeight<.03||ground.cameraHeight>1||ground.horizontalFov<30||ground.horizontalFov>160||ground.pitchDegrees<0||ground.pitchDegrees>80||ground.laneWidth<.7||ground.laneWidth>2||depth<=length/2+.05||depth>=bayLength-length/2-.05||width>=ground.laneWidth/2||wheelbase<=0||speedScale<=0||steerScale<=0||lookahead<.25||boardFrames<2||entryFrames<2||blueSMin<0||blueSMin>255||maxSeconds<=0||maxBlind<=0||approachPwm<=10000||approachPwm>11000||enterPwm<=10000||enterPwm>approachPwm)
            throw std::runtime_error("Invalid parking geometry/control config");
    }
};
// board/entryDistance 为投影距离，负值表示未检测；横向正方向为车体左侧。
// ambiguousBoard 表示两侧均有占位挡板证据，控制器不能据此确认单侧空位。
struct ParkingObservation {
    double boardDistance=-1,boardLateral=0,entryDistance=-1,laneCenter=0,laneHeading=0,laneConfidence=0;
    bool ambiguousBoard=false;
};
class ParkingVision {
    GroundVision ground;
public:
    // 先借助 GroundVision 得到鸟瞰车道几何，再单独筛选停车入口黄线和蓝挡板。
    ParkingObservation process(const cv::Mat& image,const ParkingConfig& c){
        auto g=ground.process(image,c.ground,true,false,false,true);
        ParkingObservation o; o.boardDistance=g.boardDistance;o.boardLateral=g.boardLateral;
        o.laneCenter=g.laneError;o.laneHeading=g.laneHeading;o.laneConfidence=g.laneConfidence;
        // Require a nearly full-width transverse marking. The 10m edge markers
        // and the longitudinal yellow divider must not act as an entrance.
        cv::Mat hsv,yellow;cv::cvtColor(ground.bird,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(17,100,100),cv::Scalar(38,255,255),yellow);
        std::vector<std::vector<cv::Point>> contours;cv::findContours(yellow,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        for(const auto& contour:contours){
            auto r=cv::boundingRect(contour);
            // Divider can join the entrance as a T: inspect rows, not the whole box aspect.
            // 逐行检查黄线横向跨度和像素数，允许入口与纵向分隔线连成 T 形。
            // 入口必须覆盖足够大的车道宽度，避免将短边标记或纵向黄线当入口。
            for(int v=r.y;v<r.y+r.height;++v){
                int first=-1,last=-1,n=0;for(int u=0;u<yellow.cols;++u)if(yellow.at<uchar>(v,u)){if(first<0)first=u;last=u;++n;}
                double span=(last-first)*3.2/(yellow.cols-1);
                if(n<.65*c.ground.laneWidth*(yellow.cols-1)/3.2||span<.70*c.ground.laneWidth)continue;
                double d=4.0-v/double(yellow.rows-1)*3.75;
                if(d>.28&&(o.entryDistance<0||d<o.entryDistance))o.entryDistance=d;
            }
        }
        // 用本模块更严格的蓝色饱和度和投影尺寸约束，重新筛选挡板并覆盖通用观测。
        // 按距车最近的合格挡板给出位置，同时记录左右是否都有占位证据。
        o.boardDistance=-1;o.boardLateral=0;
        cv::Mat blue,imgHsv;cv::cvtColor(image,imgHsv,cv::COLOR_BGR2HSV);cv::inRange(imgHsv,cv::Scalar(90,c.blueSMin,35),cv::Scalar(135,255,255),blue);
        contours.clear();cv::findContours(blue,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        bool left=false,right=false;double f=image.cols/(2*std::tan(c.ground.horizontalFov*CV_PI/360));
        for(auto& contour:contours){auto r=cv::boundingRect(contour);cv::Point2d pt;
            if(cv::contourArea(contour)<35||!ground.unproject(r.x+r.width*.5,r.y+r.height,image.size(),c.ground,pt)||pt.x<.38||pt.x>6)continue;
            double physicalHeight=r.height*std::max(.1,pt.x-c.ground.cameraForward)/f;
            double physicalWidth=r.width*std::max(.1,pt.x-c.ground.cameraForward)/f;
            if(physicalHeight<.21||physicalHeight>1.2||physicalWidth<.20||physicalWidth>.85)continue;
            if(o.boardDistance<0||pt.x<o.boardDistance){o.boardDistance=pt.x;o.boardLateral=pt.y;}
            double center=o.laneCenter+std::tan(o.laneHeading)*(pt.x-.9),y=pt.y-center;
            if(y>.08)left=true;if(y<-.08)right=true;
        }
        o.ambiguousBoard=left&&right;return o;
    }
};
struct ParkingCommand {int motor=10000,steer=80;std::string state="WAIT",reason;double error=0,remaining=-1;int openSide=0;};
// 状态顺序：WAIT 等待 → SEEK 搜索 → ALIGN 选侧对齐 → ENTER 入位 → BRAKE → DONE。
// FAULT 锁存异常；所需停车指令还会经过 Suite 的交通停车等优先级仲裁。
class ParkingController {
    enum Phase {WAIT,SEEK,ALIGN,ENTER,BRAKE,DONE,FAULT} phase=WAIT;
    OriginalPd pd;int side=0,candidate=0,boardHits=0,entryHits=0,lastMotor=10000,lastSteer=80;
    double parkingStart=-1;
    double now=0,runStart=-1,remaining=-1,entryEstimate=-1,blindDistance=0,brakeStart=0,lastParkError=0;
    std::string fault;double faultBrakeStart=-1;bool faultNeedsBrake=false;
    // 首次进入 FAULT 时保存原因，并依据上一条电机指令判断是否需要一次制动。
    void fail(const std::string& why){if(phase!=FAULT){faultNeedsBrake=lastMotor>10000;fault=why;}phase=FAULT;}
public:
    // 反馈的是上层最终实际发出的指令，避免把被红灯等覆盖的前进命令继续算作行程。
    void feedback(int motor,int steer){lastMotor=motor;lastSteer=steer;}
    // 控制器累计传入的 dt；Suite 在交通等待期间不调用此函数，恢复时传入该帧的 dt。
    // laneOnly 是此独立接口的旁路选项；当前集成调用保持默认 false。
    ParkingCommand step(const ParkingObservation& o,double laneError,double dt,const LaneConfig& lane,const ParkingConfig& c,bool laneOnly=false){
        ParkingCommand out;out.motor=lane.neutral_pwm;out.steer=int(lane.servo_center);
        if(!std::isfinite(dt)||dt<=0||(c.watchdogEnabled&&dt>.30))fail("STALE_FRAME");
        if(!std::isfinite(laneError))fail("INVALID_LANE_ERROR");
        now+=std::max(0.0,dt);
        if(phase==WAIT&&now>=1){phase=SEEK;runStart=now;}
        if(!laneOnly&&phase!=DONE&&phase!=FAULT&&parkingStart>=0&&now-parkingStart>c.maxSeconds)fail("PARKING_TIMEOUT");
        // 用上一帧正向 PWM 与中位的差乘速度系数及 dt 估计行程，反向指令不累计前进距离。
        // ENTER 阶段用估计行程递减剩余距离，匹配到同一入口时再用视觉观测校正。
        double travel=std::max(0,lastMotor-lane.neutral_pwm)*c.speedScale*dt;
        if(phase==ENTER){remaining-=travel;entryEstimate-=travel;blindDistance+=travel;
            // Match the tracked entrance, never relabel the far boundary as the entrance.
            // 入口足够远且与预测位置相近时才重新关联，避免把远端边界误认成原入口。
            // 长距离没有有效校正触发 ENTRY_ODOMETRY_LIMIT；达到制动距离则进入 BRAKE。
            if(entryEstimate>.30&&o.entryDistance>.28&&std::abs(o.entryDistance-entryEstimate)<.25){
                entryEstimate=.5*entryEstimate+.5*o.entryDistance;remaining=entryEstimate+c.depth;blindDistance=0;
            }
            if(blindDistance>c.maxBlind)fail("ENTRY_ODOMETRY_LIMIT");
            else if(remaining<=c.brakeDistance){phase=BRAKE;brakeStart=now;}
        }
        // 连续确认挡板相对车道中心的位置并选择另一半车位；已选侧发生变化则进入故障。
        // 确认入口和边界可信后进入 ENTER；临近入口仍无已确认空位也会故障停车。
        if(!laneOnly&&(phase==SEEK||phase==ALIGN)){
            if(o.ambiguousBoard){boardHits=0;entryHits=0;if(o.entryDistance>0&&o.entryDistance<.8)fail("AMBIGUOUS_OCCUPANCY");}
            else if(o.boardDistance>.4&&o.laneConfidence>=.5){
                double center=o.laneCenter+std::tan(o.laneHeading)*(o.boardDistance-.9);
                double relative=o.boardLateral-center;
                int next=relative>.08?-1:relative<-.08?1:0; // left positive; choose opposite half.
                if(next&&next==candidate)++boardHits;else{candidate=next;boardHits=next?1:0;}
                if(boardHits>=c.boardFrames){if(side&&side!=next)fail("OCCUPANCY_CHANGED");else{side=next;if(parkingStart<0)parkingStart=now;phase=ALIGN;}}
            }else if(phase==SEEK)boardHits=0;
            bool valid=phase==ALIGN&&!o.ambiguousBoard&&o.entryDistance>.35&&o.entryDistance<2.5&&o.laneConfidence>=.5;
            entryHits=valid?entryHits+1:0;
            if(entryHits>=c.entryFrames){entryEstimate=o.entryDistance;remaining=entryEstimate+c.depth;blindDistance=0;phase=ENTER;}
            if(phase==SEEK&&o.entryDistance>0&&o.entryDistance<.65)fail("NO_CONFIRMED_FREE_BAY");
        }
        if(phase==BRAKE&&now-brakeStart>=lane.zebra_test_brake_ms/1000.0)phase=DONE;
        const char* names[]={"WAIT","SEEK","ALIGN","ENTER","BRAKE","DONE","FAULT"};out.state=names[phase];out.reason=fault;
        out.error=laneError;
        if(phase==SEEK){out.motor=lane_throttle((long long)((now-runStart)*1000),lane);out.steer=int(std::lround(pd.update(laneError,lane)));}
        // 停车时根据空闲半区中心、车道朝向和前视距离求转向角，再换算成既有 PD 的误差输入。
        // 停车误差变化率限制为每秒 60 单位；边界置信度不足时进入 FAULT。
        if(phase==ALIGN||phase==ENTER){
            if(o.laneConfidence<.5){fail("PARKING_BOUNDARIES_LOST");out.state="FAULT";out.reason=fault;}
            else{
                double target=o.laneCenter+side*c.ground.laneWidth*.25+std::tan(o.laneHeading)*(c.lookahead-.9);
                double angle=std::atan2(2*c.wheelbase*target,c.lookahead*c.lookahead+target*target);
                double desired=-angle/(c.steerScale*lane.lane_pd_kp);
                // Only the NEW parking reference is rate-limited; original PD/limits stay unchanged.
                double maxDelta=60*dt;lastParkError=std::max(lastParkError-maxDelta,std::min(lastParkError+maxDelta,desired));
                out.error=lastParkError;out.motor=phase==ENTER?c.enterPwm:c.approachPwm;out.steer=int(std::lround(pd.update(out.error,lane)));
            }
        }
        // BRAKE 保持上一舵机位置并输出既有反向 PWM；时长结束进入 DONE 中位停车。
        // FAULT 若发生于前进状态也先执行一次有时限的制动，再维持中位。
        if(phase==BRAKE){out.motor=lane.zebra_brake_pwm;out.steer=lastSteer;}
        if(phase==FAULT||phase==DONE){out.motor=lane.neutral_pwm;out.steer=int(lane.servo_center);}
        if(phase==FAULT&&faultNeedsBrake){if(faultBrakeStart<0)faultBrakeStart=now;if(now-faultBrakeStart<lane.zebra_test_brake_ms/1000.0){out.motor=lane.zebra_brake_pwm;out.steer=lastSteer;}}
        out.remaining=remaining;out.openSide=side;lastMotor=out.motor;lastSteer=out.steer;return out;
    }
};

}
