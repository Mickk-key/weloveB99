// 室外总控：挡板发车 → 斑马线 → 红绿灯（含红灯超时）→ 巡线前进 → 视觉绕桶 → parking3。
// 本次新增时序只在outdoor维护，室内副本不随之改动；视觉算法保持原样。
#pragma once
#include "runtime.hpp"
#include "lane_backend.hpp"
#include "cone_v2.hpp"
#include "parking3.hpp"
#include "zebra_original.hpp"
#include "traffic_light.hpp"
#include "race_sequence.hpp"

// 巡线始终启用；all 为当前完整比赛组合，停车实现固定为 parking3。
// 仍保留现用模块的单项测试，不保留旧 parking/parking2 名称或模式分支。
struct Modules {
    bool light=false,cone=false,zebra=false,parking3=false;
    explicit Modules(std::string list="lane"){
        std::stringstream in(list);std::string item;
        while(std::getline(in,item,',')){
            if(item=="all")light=cone=zebra=parking3=true;
            else if(item=="lane"){}
            else if(item=="light")light=true;
            else if(item=="cone")cone=true;
            else if(item=="zebra")zebra=true;
            else if(item=="parking3")parking3=true;
            else throw std::runtime_error("Unknown module: "+item);
        }
        if(list.empty())throw std::runtime_error("Empty module list");
    }
    bool fullCourse()const{return light&&cone&&zebra&&parking3;}
    std::string str()const{return std::string("lane")+(light?",light":"")+(cone?",cone":"")+(zebra?",zebra":"")+(parking3?",parking3":"");}
};
// 当前唯一停车为parking3；旧方案字段不参与加载。非巡线参数沿用室外现值。
struct SuiteConfig {
    TrafficControlConfig light;bool lightNight=false;
    Parking3Config parking3;Settings drive;LaneBackendConfig lane;
    ConeConfig cone;BoardConfig board;zebra::Config zebra;
    std::string backend=LaneBackend::name();
    double parkingPhotoDelay=1.0;
    double coneAfterLightSeconds=5.; // 仅all：灯阶段完成后，先巡线再启用锥桶识别。
    // 终点白板确认后只制动一次；参数与jgy/race室外配置一致。
    int parking3BrakePwm=9500;double parking3BrakeSeconds=.3;
    double zebraY=.70,zebraHold=10,zebraBrake=.8;
    int zebraBrakePwm=9000,cameraFps=30;
    int brightness=-1,gain=-1,whiteBalanceAuto=-1;
    void load(const std::string& file){
        drive=load_settings(file);cv::FileStorage fs(file,cv::FileStorage::READ);
        if(!fs["parking3_brake_pwm"].empty())fs["parking3_brake_pwm"]>>parking3BrakePwm;
        if(!fs["parking3_brake_seconds"].empty())fs["parking3_brake_seconds"]>>parking3BrakeSeconds;
        if(parking3BrakePwm<8000||parking3BrakePwm>=drive.neutral_pwm||!std::isfinite(parking3BrakeSeconds)||parking3BrakeSeconds<0||parking3BrakeSeconds>3)throw std::runtime_error("Invalid parking3 brake PWM/duration");
        parking3.load(fs);lane.load(fs);cone.load(fs);board.load(fs);light.load(fs,drive.neutral_pwm);
#define R(key,field) if(!fs[key].empty())fs[key]>>field
        R("parking_photo_delay_seconds",parkingPhotoDelay);R("lane_backend",backend);
        R("cone_after_light_seconds",coneAfterLightSeconds);
        R("zebra_stop_y",zebraY);R("zebra_hold_seconds",zebraHold);R("zebra_brake_seconds",zebraBrake);R("zebra_brake_pwm",zebraBrakePwm);
        R("camera_fps",cameraFps);R("camera_brightness",brightness);R("camera_gain",gain);R("camera_white_balance_auto",whiteBalanceAuto);
#undef R
        if(!std::isfinite(parkingPhotoDelay)||parkingPhotoDelay<0||parkingPhotoDelay>60)throw std::runtime_error("parking_photo_delay_seconds must be 0..60");
        if(!std::isfinite(coneAfterLightSeconds)||coneAfterLightSeconds<0||coneAfterLightSeconds>60)throw std::runtime_error("cone_after_light_seconds must be 0..60");
#define Z(n) if(!fs[#n].empty())fs[#n]>>zebra.n
        Z(zebra_white_s_max);Z(zebra_white_v_min);Z(zebra_min_stripes);Z(zebra_confirm_frames);
        Z(zebra_roi_left);Z(zebra_roi_right);Z(zebra_roi_top);Z(zebra_roi_bottom);
        Z(zebra_min_stripe_width);Z(zebra_max_stripe_width);Z(zebra_min_span);Z(zebra_min_support_height);
        Z(zebra_max_width_ratio);Z(zebra_max_pitch_ratio);Z(zebra_min_gap_ratio);Z(zebra_max_gap_ratio);
#undef Z
        zebra::validate_zebra_config(zebra);
        if(backend!=LaneBackend::name())throw std::runtime_error("Configuration does not match this folder lane backend");
        for(double v:{zebraY,zebraHold,zebraBrake})if(!std::isfinite(v))throw std::runtime_error("Non-finite module configuration");
        if(zebraY<.2||zebraY>.98||zebraHold<0||zebraHold>60||zebraBrake<0||zebraBrake>2||zebraBrakePwm<3000||zebraBrakePwm>10000||cameraFps<1||cameraFps>60||whiteBalanceAuto< -1||whiteBalanceAuto>1)throw std::runtime_error("Invalid module/camera configuration");
    }
};
// 斑马线一次性流程：连续确认且底边达到 zebraY → 制动 → 中位等待 → DONE。
// 若触发前已静止，则制动时长为 0；等待结束后本轮不再重复触发。
struct ZebraGate {
    int hits=0;bool done=false;double began=-1,brakeDuration=0;
    void update(bool found,double y,double now,const SuiteConfig& c,bool wasMoving){
        if(done)return;
        if(began<0){hits=found&&y>=c.zebraY?hits+1:0;if(hits>=c.zebra.zebra_confirm_frames){began=now;brakeDuration=wasMoving?c.zebraBrake:0;}}
        if(began>=0&&now-began>=brakeDuration+c.zebraHold)done=true;
    }
    std::string phase(double now,const SuiteConfig& c)const{
        if(done)return "DONE";if(began<0)return "SEARCH";return now-began<brakeDuration?"BRAKE":"HOLD";
    }
};
// 一帧的最终控制结果；各视觉模块不直接操作GPIO。
struct Command {
    TrafficStatus light;
    std::string raceStage="INDEPENDENT";
    bool startBoardConfirmed=false;int startClearHits=0;
    int motor=10000,steer=78,left=0,right=0,passed=0;
    double error=0,baseError=0,offset=0,target=78,zebraY=0,brakeRemaining=-1;
    int coneTarget=0,coneNext=0,coneTracks=0,coneSide=0;double coneHandoff=0;
    bool parkingArmed=false;double frameDt=0,processingMs=0;
    double coneCruiseRemaining=0;
    std::string parkingPath="NONE";
    bool panelChecked=false,panelFound=false;int panelHits=0,parkingSide=0;
    std::string state="WAIT",cone="DISABLED",zebra="DISABLED",parking="DISABLED";
};
class Suite {
    SuiteConfig cfg;Modules modules;LaneBackend lane;

    TrafficLightControl traffic;RaceSequence race;
    double lastInputTime=-1,lastTime=-1,runStart=-1,filtered=78;
    int lastMotor=10000;
    double parking3BrakeAt=-1; // 锁存首次白板确认时刻，后续帧不能延长或重复制动。
    OriginalPd pd;ZebraGate zebraGate;Parking3 parking3;

public:
    ConeAvoidV2 cone;BlueBoardStop board;LaneV2 geometry;
    zebra::ZebraResult stripes;cv::Mat laneMask;
    explicit Suite(const SuiteConfig& c,Modules m):cfg(c),modules(m),lane(c.lane),
        traffic(c.light,c.lightNight,m.fullCourse()),race(m.fullCourse(),c.board),filtered(c.drive.servo_center),
        parking3(c.parking3,c.board),cone(c.cone),board(c.board){}
    // 输入时间必须非负且单调；跨帧状态保存在 Suite 中，首次 dt 按 0.05 秒处理。
    // 每帧先以电机中位、舵机居中初始化，再逐级生成和覆盖命令。
    Command step(cv::Mat frame,double now){
        if(!std::isfinite(now)||now<0||(lastInputTime>=0&&now<lastInputTime))throw std::runtime_error("Invalid frame clock");
        lastInputTime=now;
        Command out;out.motor=cfg.drive.neutral_pwm;out.steer=int(cfg.drive.servo_center);out.target=filtered;
        race.updateStart(frame,now);
        out.raceStage=race.name();out.startBoardConfirmed=race.boardConfirmed();out.startClearHits=race.clearHits();
        if(race.waiting()){
            // 提前返回：等发车时不更新斑马线、红绿灯、绕桶或终点挡板状态，
            // 也不累计这些模块的计时。输出始终为电机中位、转向中位。
            out.state=race.name();out.zebra=out.cone=out.parking="WAIT_START";
            out.light.phase=out.light.sequence="WAIT_START";
            out.target=filtered=cfg.drive.servo_center;lastMotor=cfg.drive.neutral_pwm;
            if(laneMask.empty())laneMask=cv::Mat::zeros(frame.size(),CV_8UC1);
            return out;
        }
        now=race.runTime(now);
        double dt=lastTime<0?.05:std::max(.001,now-lastTime);lastTime=now;
        lane.detect(frame,geometry,laneMask,out.baseError,out.left,out.right);
        out.error=out.baseError;
        // 斑马线距离条件使用检测框底边/整图高度的比例，不是米。
        // 先更新斑马线，红绿灯从等待完成后才按light_after_zebra_seconds累计前进时间。
        if(modules.zebra){
            if(race.zebraAllowed()){
                stripes=zebra::detect_frame(frame,cfg.zebra);out.zebraY=stripes.box.empty()?0:double(stripes.box.br().y)/frame.rows;
                zebraGate.update(stripes.found,out.zebraY,now,cfg,lastMotor>cfg.drive.neutral_pwm);
            }
            out.zebra=zebraGate.phase(now,cfg);race.finishZebra(zebraGate.done);
        }
        const bool runLight=modules.light&&race.lightAllowed();
        if(runLight){
            traffic.update(frame,now,modules.zebra,zebraGate.done,lastMotor>cfg.drive.neutral_pwm);
            out.light=traffic.status(now);
            race.finishLight(traffic.done(),now);
        }else if(modules.light){
            out.light=traffic.status(now);out.light.detectionRan=false;
            if(!traffic.done())out.light.phase=out.light.sequence="WAIT_STAGE";
        }
        race.updateConeCruise(now,cfg.coneAfterLightSeconds,lastMotor>cfg.drive.neutral_pwm);
        out.coneCruiseRemaining=race.coneCruiseRemaining(cfg.coneAfterLightSeconds);
        // 视觉确认通过两个桶并回中后解锁停车；本帧新计数到下一帧使用。
        bool trafficHold=(modules.light&&traffic.active())||(modules.zebra&&!zebraGate.done&&zebraGate.began>=0);
        bool conesDone=!modules.cone||cone.laneCompleted();
        race.finishCones(conesDone);
        bool runCones=modules.cone&&race.coneAllowed();
        out.parkingArmed=modules.parking3&&race.parkingAllowed()&&conesDone;
        if(out.parkingArmed){
            board.update(frame,now);parking3.update(frame,now,!trafficHold&&now>=1,geometry);out.parking=parking3.state;out.panelChecked=parking3.side!=0;out.panelFound=parking3.panelFound();out.panelHits=parking3.panelHits();out.parkingPath=parking3.path;
        }
        else if(modules.parking3)out.parking=race.enabled()&&race.stage()!=RaceSequence::CONE?"WAIT_STAGE":"WAIT_CONES";
        bool parkingActive=out.parkingArmed&&parking3.engaged();
        if(runCones&&!parkingActive)cone.update(frame,geometry,now,!trafficHold&&now>=1);
        out.cone=modules.cone?cone.phase:"DISABLED";out.passed=cone.passedCount;
        if(modules.cone&&race.enabled()&&!runCones&&!race.parkingAllowed())out.cone="WAIT_STAGE";
        if(race.stage()==RaceSequence::CONE_CRUISE)out.cone="CRUISE";
        if(modules.cone){const auto& v=cone.laneVisual();out.coneTarget=v.targetId;out.coneNext=v.nextTargetId;out.coneTracks=v.trackCount;out.coneSide=v.side;out.coneHandoff=v.handoff;}
        out.offset=runCones&&!parkingActive?cone.offsetFraction:0;
        out.error=out.baseError+geometry.targetError(out.offset)-geometry.targetError(0);
        if(runCones&&!parkingActive&&cone.laneOverride())out.error=out.baseError+cone.laneErrorDelta();
        // 基础巡线输出保留原PD、分段油门、转向限速及取整顺序。
        out.state=now<1?"WAIT":"RUN";
        if(out.state=="RUN"){
            if(runStart<0)runStart=now;
            out.motor=lane_throttle(std::llround((now-runStart)*1000),cfg.drive);
            if(runCones&&!parkingActive&&(cone.near||std::abs(out.offset)>.01)){
                out.state="CONE_"+cone.phase;
                if(out.motor>cfg.drive.neutral_pwm)out.motor=std::min(out.motor,cfg.drive.cone_slow_pwm);
            }
            bool parkingOwnSteer=parkingActive&&!parking3.useLane;
            double requested;
            if(parkingOwnSteer){requested=filtered;pd.previous=out.baseError;}
            else requested=pd.update(out.error,cfg.drive);
            double step=cfg.drive.steer_rate*std::min(.1,dt);
            if(!parkingOwnSteer)filtered+=std::max(-step,std::min(step,requested-filtered));
            out.steer=std::lround(filtered);out.target=filtered;

        }else pd.previous=out.error;
        if(race.stage()==RaceSequence::CONE_CRUISE&&out.state=="RUN")out.state="CONE_CRUISE";
        if(out.parkingArmed&&parking3.engaged()){
            out.state=out.parking=parking3.state;out.parkingPath=parking3.path;
            out.motor=parking3.stopped?cfg.drive.neutral_pwm:parking3.motor();
            if(parking3.useLane&&!parking3.stopped){
                // 蓝挡板离开视野或缺少可信白边时，保留上方正常巡线的误差与舵机结果。
            }else{
            out.error=parking3.error;
            double target=cfg.drive.servo_center-parking3.correction;
            if(!parking3.stopped){double delta=parking3.rate()*std::min(.1,dt);
                filtered+=std::max(-delta,std::min(delta,target-filtered));
                filtered=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,filtered));
            }else filtered=cfg.drive.servo_center;
            out.target=filtered;out.steer=std::lround(filtered);
            }
        }

        // 只限制接近红绿灯时的前进油门，巡线转向继续工作。
        if(modules.light&&traffic.slowing()&&!parkingActive&&out.motor>cfg.drive.neutral_pwm){
            out.motor=std::min(out.motor,cfg.light.sequence.slowPwm);
            if(out.state=="RUN")out.state="LIGHT_APPROACH";
        }
        // 停车优先级：斑马线 → 红绿灯 → 终点锁停；最后处理起步和中位转向。
        if(modules.zebra&&out.zebra=="BRAKE"){out.state="ZEBRA_BRAKE";out.motor=cfg.zebraBrakePwm;out.brakeRemaining=std::max(0.,zebraGate.began+zebraGate.brakeDuration-now);}
        if(modules.zebra&&out.zebra=="HOLD"){out.state="ZEBRA_HOLD";out.motor=cfg.drive.neutral_pwm;}
        if(modules.light&&traffic.active()){
            out.state=traffic.braking(now)?"LIGHT_BRAKE":"LIGHT_WAIT_GREEN";
            out.motor=traffic.braking(now)?cfg.light.brakePwm:cfg.drive.neutral_pwm;
            out.brakeRemaining=traffic.braking(now)?traffic.brakeRemaining(now):-1;
        }
        // 终点停车优先于斑马线/红灯；制动剩余时间交给现有Output截止计时。
        if(out.parkingArmed&&parking3.stopped){
            if(parking3BrakeAt<0)parking3BrakeAt=now;
            double remaining=std::max(0.,cfg.parking3BrakeSeconds-(now-parking3BrakeAt));
            bool braking=remaining>1e-9;
            out.state=out.parking=braking?"PARK3_BRAKE":"PARK3_PANEL_STOP";
            out.motor=braking?cfg.parking3BrakePwm:cfg.drive.neutral_pwm;
            out.brakeRemaining=braking?remaining:-1;
            out.steer=int(cfg.drive.servo_center);out.target=filtered=cfg.drive.servo_center;
        }
        if(now<1)out.motor=cfg.drive.neutral_pwm;
        if(out.motor==cfg.drive.neutral_pwm){out.steer=int(cfg.drive.servo_center);filtered=cfg.drive.servo_center;pd.previous=out.error;}
        if(out.motor>=cfg.drive.neutral_pwm)out.brakeRemaining=-1;
        // 反馈的是最终控制命令，不是编码器测得的实际运动。
        cone.laneFeedback(runCones&&!parkingActive&&out.motor>cfg.drive.neutral_pwm);
        out.frameDt=dt;out.parkingSide=parking3.side;
        traffic.feedback(runLight&&out.motor>cfg.drive.neutral_pwm&&!parkingActive,
            runLight&&out.motor>cfg.drive.neutral_pwm&&out.motor<=cfg.light.sequence.slowPwm&&!parkingActive);
        // 制动期间保持PARKING，回到中位才进入FINISHED并允许停车照片计时。
        race.finishParking(out.parkingArmed&&parking3.stopped&&out.motor==cfg.drive.neutral_pwm);
        out.raceStage=race.name();lastMotor=out.motor;
        return out;
    }
    // 标注仅画在副本上，不把标注图送回识别器。
    cv::Mat draw(const cv::Mat& raw,const Command& c)const{
        cv::Mat view=raw.clone();
        if(race.waiting())race.drawStart(view);
        else{
            if(c.parking!="PARK3_FOLLOW_WHITE")geometry.draw(view,c.offset);
            if(modules.cone)cone.draw(view,c.cone);
            if(modules.parking3&&board.candidate)cv::rectangle(view,board.box,{255,200,0},2);
            if(modules.parking3)parking3.draw(view);
            if(modules.zebra&&stripes.found)cv::rectangle(view,stripes.box,{255,0,255},2);
            if(modules.light)traffic.draw(view,c.light);
        }
        cv::putText(view,c.state+cv::format(" M%d S%d",c.motor,c.steer),{5,20},cv::FONT_HERSHEY_SIMPLEX,.5,{0,255,255},1);
        if(race.enabled())cv::putText(view,"RACE "+c.raceStage,{5,42},cv::FONT_HERSHEY_SIMPLEX,.5,{0,255,255},1);
        return view;
    }
};
