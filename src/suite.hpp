// 功能集成层：将巡线、红绿灯、斑马线、锥桶和停车模块组合成一帧 Command。
// 各检测器不直接操作 GPIO；step() 的末尾按现有优先级确定最终输出。
#pragma once
#include "parking_white_pair.hpp"
#include <set>
#include "runtime.hpp"
#include "lane_original.hpp"
#include "lane_outdoor_v2.hpp"
#include "parking_qr.hpp"
#include "parking_yellow.hpp"
#include "parking2.hpp"
#include "parking3.hpp"
#include "async_parking_qr.hpp"
#include "parking_depth.hpp"
#include "zebra_original.hpp"
#include "traffic_light.hpp"

// lane 始终是基础功能；列表中的 light/cone/zebra/parking 仅打开对应附加模块。
// all 启用全部附加模块；只写 parking 也仍然保留基础巡线。
struct Modules {
    bool light=false,cone=false,zebra=false,parking=false,parking2=false,parking3=false;
    explicit Modules(std::string list="lane"){
        std::stringstream in(list);std::string item;
        while(std::getline(in,item,',')){
            if(item=="all"){light=cone=zebra=parking=true;}
            else if(item=="lane"){}
            else if(item=="light")light=true;
            else if(item=="cone")cone=true;
            else if(item=="zebra")zebra=true;else if(item=="parking")parking=true;else if(item=="parking2")parking2=true;else if(item=="parking3")parking3=true;
            else throw std::runtime_error("Unknown module: "+item);
        }
        if(list.empty())throw std::runtime_error("Empty module list");
        if(int(parking)+int(parking2)+int(parking3)>1)throw std::runtime_error("Choose one of parking, parking2, parking3");
    }
    std::string str() const {return std::string("lane")+(light?",light":"")+(cone?",cone":"")+(zebra?",zebra":"")+(parking?",parking":"")+(parking2?",parking2":"")+(parking3?",parking3":"");}
};
// 配置汇总：drive 为相机/PWM/PD，lane/outdoor 为不同巡线后端，其余为任务识别参数。
// 这里的等待和制动时长为秒；depthDrive 的旧接口需要毫秒，加载时显式换算。
struct SuiteConfig {
    TrafficControlConfig light;bool lightNight=false;
    YellowParkingConfig yellow;Parking2Config parking2;Parking3Config parking3;ParkingWhiteConfig whitePairConfig;
    Settings drive;LaneConfig lane;OutdoorLaneConfig outdoor;ConeConfig cone;BoardConfig board;zebra::Config zebra;
    std::string backend="indoor",parkingMode="qr";
    double parkingPhotoDelay=1.0;
    double zebraY=.70,zebraHold=10,zebraBrake=.8;
    int zebraBrakePwm=9000,parkingAfterCones=2,cameraFps=30;
    int brightness=-1,gain=-1,whiteBalanceAuto=-1;
    depth_parking::ParkingConfig depth;depth_parking::LaneConfig depthDrive;
    std::string path;
    void load(const std::string& file){
        path=file;drive=load_settings(file);cv::FileStorage fs(file,cv::FileStorage::READ);
        // 各模块从同一环境 YAML 读取自己的字段；缺少可选字段时保留各自默认值。
        yellow.load(fs);parking2.load(fs);parking3.load(fs);whitePairConfig.load(fs);lane.load(fs);outdoor.load(fs);cone.load(fs);board.load(fs);light.load(fs,drive.neutral_pwm);
        if(cone.cone_fixed_maneuver)cone.fixedPlan.validateMotor(drive.neutral_pwm,cone.cone_s_course_once?"cone_s_steps":"cone_maneuver_steps");
        // --check-config 也校验停车动作，不等到摄像头/GPIO 初始化后才报 YAML 错误。
        Parking parkingCheck;parkingCheck.load(fs);parkingCheck.validateFixedMotor(drive.neutral_pwm);
#define R(key,field) if(!fs[key].empty())fs[key]>>field
        R("parking_photo_delay_seconds",parkingPhotoDelay);R("lane_backend",backend);R("parking_mode",parkingMode);
        R("zebra_stop_y",zebraY);R("zebra_hold_seconds",zebraHold);R("zebra_brake_seconds",zebraBrake);R("zebra_brake_pwm",zebraBrakePwm);
        R("parking_after_cones",parkingAfterCones);R("camera_fps",cameraFps);
        R("camera_brightness",brightness);R("camera_gain",gain);R("camera_white_balance_auto",whiteBalanceAuto);
#undef R
        if(!std::isfinite(parkingPhotoDelay)||parkingPhotoDelay<0||parkingPhotoDelay>60)throw std::runtime_error("parking_photo_delay_seconds must be 0..60");
#define Z(n) if(!fs[#n].empty())fs[#n]>>zebra.n
        Z(zebra_white_s_max);Z(zebra_white_v_min);Z(zebra_min_stripes);Z(zebra_confirm_frames);
        Z(zebra_roi_left);Z(zebra_roi_right);Z(zebra_roi_top);Z(zebra_roi_bottom);
        Z(zebra_min_stripe_width);Z(zebra_max_stripe_width);Z(zebra_min_span);Z(zebra_min_support_height);
        Z(zebra_max_width_ratio);Z(zebra_max_pitch_ratio);Z(zebra_min_gap_ratio);Z(zebra_max_gap_ratio);
#undef Z
        zebra::validate_zebra_config(zebra);
        if((backend!="indoor"&&backend!="outdoor"&&backend!="outdoor_v2")||(parkingMode!="qr"&&parkingMode!="timed"&&parkingMode!="yellow"&&parkingMode!="depth"&&parkingMode!="parking2"&&parkingMode!="parking3"))throw std::runtime_error("Invalid lane_backend or parking_mode");
        for(double v:{zebraY,zebraHold,zebraBrake})if(!std::isfinite(v))throw std::runtime_error("Non-finite module configuration");
        if(zebraY<.2||zebraY>.98||zebraHold<0||zebraHold>60||zebraBrake<0||zebraBrake>2||zebraBrakePwm<3000||zebraBrakePwm>10000||parkingAfterCones<0||parkingAfterCones>20||cameraFps<1||cameraFps>60||whiteBalanceAuto< -1||whiteBalanceAuto>1)throw std::runtime_error("Invalid module/camera configuration");
        // 深度停车配置也在启动时校验；其控制标定从 drive 同步。
        // 集成入口不调用 depth_parking::LaneConfig::load 的旧版固定标定检查。
        depth.load(file);depth.watchdogEnabled=drive.control_watchdog_enabled;
        depthDrive.lane_pd_kp=drive.lane_pd_kp;depthDrive.lane_pd_kd=drive.lane_pd_kd;
        depthDrive.servo_center=drive.servo_center;depthDrive.servo_min=drive.servo_min;depthDrive.servo_max=drive.servo_max;
        depthDrive.neutral_pwm=drive.neutral_pwm;depthDrive.zebra_brake_pwm=zebraBrakePwm;depthDrive.zebra_test_brake_ms=std::lround(zebraBrake*1000);
    }
};
// Extract actual original fitted boundaries for obstacle/parking geometry. Lane driving
// continues to use the original TUxiang_Init3 error, including original edge fallback.
// 把原版 320×96 拟合掩码还原到 640×480 图像坐标，供绕桶和停车判断边界位置。
// 这份几何不替代原版巡线误差；原版的实际转向误差仍来自 TUxiang_Init3。
inline LaneV2 originalGeometry(){
    LaneV2 l;l.top=210;l.bottom=402;std::vector<cv::Point2f> left,right;
    const auto& mask=original_lane::lane_mask;
    int split=std::max(1,std::min(318,original_lane::mid_line));
    for(int y=0;y<mask.rows;y+=2){
        if(original_lane::lane_left_count)for(int x=split;x>=0;--x)if(mask.at<uchar>(y,x)){left.emplace_back(x*2,y*2+210);break;}
        if(original_lane::lane_right_count)for(int x=split;x<mask.cols;++x)if(mask.at<uchar>(y,x)){right.emplace_back(x*2,y*2+210);break;}
    }
    auto fit=[](const std::vector<cv::Point2f>& pts,double& a,double& b){
        if(pts.size()<8)return false;cv::Vec4f v;cv::fitLine(pts,v,cv::DIST_HUBER,0,.01,.01);
        if(std::abs(v[1])<.1)return false;a=v[0]/v[1];b=v[2]-a*v[3];return std::isfinite(a)&&std::isfinite(b);
    };
    l.leftValid=fit(left,l.la,l.lb);l.rightValid=fit(right,l.ra,l.rb);return l;
}
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
// 每帧结果：baseError 是纯巡线误差，error 是叠加绕桶/停车目标后的控制误差。
// offset 是车道宽度比例；brakeRemaining 是当前反向制动剩余秒数，-1 表示无定时脉冲。
struct Command {
    TrafficStatus light;
    int motor=10000,steer=78,left=0,right=0,passed=0;
    double error=0,baseError=0,offset=0,target=78,zebraY=0,brakeRemaining=-1;
    int coneTarget=0,coneNext=0,coneTracks=0,coneSide=0;double coneHandoff=0;
    bool parkingArmed=false;double fixedRemaining=0,frameDt=0,processingMs=0,edgeHeading=0,qrAge=-1;
    std::string parkingPath="NONE";
    bool panelChecked=false,panelFound=false;int panelHits=0;
    int parkingSide=0,qrHits=0;
    std::string state="WAIT",cone="DISABLED",zebra="DISABLED",parking="DISABLED",reason;
};
class Suite {
    SuiteConfig cfg;Modules modules;LaneDetectorV2 indoor;OutdoorLaneDetector outdoor;
    TrafficLightControl traffic;
    double lastTime=-1,runStart=-1,filtered=78;
    int lastMotor=10000;
    ParkingWhitePair whitePair;OriginalPd pd;ZebraGate zebraGate;YellowParking yellow;Parking2 parking2;Parking3 parking3;
    depth_parking::ParkingVision depthVision;depth_parking::ParkingController depthControl;
    depth_parking::ParkingCommand depthLast;
    std::unique_ptr<AsyncParkingQR> asyncQr;double nextQrSubmit=0;
    // 固定动作的唯一 PWM 换算入口。S 弯和固定停车共用，不叠加巡线 PD。
    // correction 对于 S 弯等于 side*steer；限位之后取整并直接覆盖本帧命令。
    // servo_center 只代表前轮配置中位，不代表车身已经朝正或位于车道中央。
    void applyFixedCommand(Command& out,double correction,int motor,
                           const std::string& state,double remaining) {
        filtered=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,cfg.drive.servo_center-correction));
        out.steer=std::lround(filtered);out.target=filtered;
        out.motor=motor;out.state=state;
        out.offset=0;out.error=out.baseError;pd.previous=out.baseError;
        out.fixedRemaining=remaining;
    }
public:
    ConeAvoidV2 cone;BlueBoardStop board;Parking parking;LaneV2 geometry;
    zebra::ZebraResult stripes;cv::Mat laneMask;
    explicit Suite(const SuiteConfig& c,Modules m,bool realtime=false):cfg(c),modules(m),indoor(c.lane),outdoor(c.outdoor),traffic(c.light,c.lightNight),filtered(c.drive.servo_center),yellow(c.yellow),parking2(c.parking2),parking3(c.parking3,c.board),whitePair(c.lane,c.whitePairConfig),cone(c.cone),board(c.board){
        if(m.parking&&c.parkingMode=="depth"&&c.drive.lane_pd_kp<=0)throw std::runtime_error("Depth parking requires lane_pd_kp > 0");
        cv::FileStorage fs(c.path,cv::FileStorage::READ);parking.load(fs);parking.timedStop=c.parkingMode=="timed";parking.yellowMode=c.parkingMode=="yellow";
        if(parking.timedStop&&!parking.parking_edge_entry)throw std::runtime_error("Timed parking requires parking_edge_entry=1");
        // 室内和两套室外共用同一实时停车流程；离线图像/回放仍同步识别。
        if(realtime&&m.parking&&c.parkingMode=="qr"&&parking.parking_edge_follow&&parking.parking_qr_async){
            asyncQr.reset(new AsyncParkingQR(c.path));parking.deferQr=true;
        }
    }
    // 输入时间必须非负且单调；跨帧状态保存在 Suite 中，首次 dt 按 0.05 秒处理。
    // 每帧先以电机中位、舵机居中初始化，再逐级生成和覆盖命令。
    Command step(cv::Mat frame,double now){
        if(!std::isfinite(now)||now<0||(lastTime>=0&&now<lastTime))throw std::runtime_error("Invalid frame clock");
        double dt=lastTime<0?.05:std::max(.001,now-lastTime);lastTime=now;
        Command out;out.motor=cfg.drive.neutral_pwm;out.steer=int(cfg.drive.servo_center);out.target=filtered;
        // 后端选择：outdoor 为原版灰度边缘路径，outdoor_v2 为新版室外白线，indoor 为室内白线。
        // 主流程后续统一使用 geometry，但保留每个后端自身的 baseError 计算。
        if(cfg.backend=="outdoor"){
            out.baseError=original_lane::TUxiang_Init3(frame);geometry=originalGeometry();laneMask=original_lane::lane_mask;
            out.left=original_lane::lane_left_count;out.right=original_lane::lane_right_count;
        }else{geometry=cfg.backend=="outdoor_v2"?outdoor.detect(frame,now):indoor.detect(frame);out.baseError=geometry.error;laneMask=cfg.backend=="outdoor_v2"?outdoor.mask:indoor.mask;out.left=geometry.leftValid;out.right=geometry.rightValid;}
        out.error=out.baseError;
        // 斑马线距离条件使用检测框底边/整图高度的比例，不是米。
        // 先更新斑马线，红绿灯从斑马线等待完成后才累计3秒前进时间。
        if(modules.zebra){stripes=zebra::detect_frame(frame,cfg.zebra);out.zebraY=stripes.box.empty()?0:double(stripes.box.br().y)/frame.rows;zebraGate.update(stripes.found,out.zebraY,now,cfg,lastMotor>cfg.drive.neutral_pwm);out.zebra=zebraGate.phase(now,cfg);}
        if(modules.light){
            traffic.update(frame,now,modules.zebra,zebraGate.done,lastMotor>cfg.drive.neutral_pwm);
            out.light=traffic.status(now);
        }
        // 红灯或斑马线等待期间，固定绕行继续观测但暂停动作计时；旧绕桶/深度停车暂停更新。
        // parking 仅在未启用 cone，或已通过的锥桶数达到 parking_after_cones 后解锁。
        bool trafficHold=(modules.light&&traffic.active())||(modules.zebra&&!zebraGate.done&&zebraGate.began>=0);
        // S 模式以整套路线完成解锁停车，绝不再等待第二桶视觉计数。
        out.parkingArmed=(modules.parking||modules.parking2||modules.parking3)&&(!modules.cone||(cone.laneMode()?cone.laneCompleted():(cfg.cone.cone_s_course_once?cone.sCourseCompleted():cone.passedCount>=cfg.parkingAfterCones)));
        // 二维码停车先确认蓝挡板，再选空闲半边；深度停车使用独立的几何观测和状态机。
        // 二维码路径的视觉状态在 trafficHold 时仍更新；深度路径保留上一帧 depthLast。
        if(out.parkingArmed&&cfg.parkingMode=="parking2"){
            board.update(frame,now);parking2.update(frame,board,now,!trafficHold&&now>=1);out.parking=parking2.state;
        }
        if(out.parkingArmed&&cfg.parkingMode=="parking3"){
            board.update(frame,now);parking3.update(frame,board,now,!trafficHold&&now>=1,geometry);out.parking=parking3.state;out.panelChecked=parking3.side!=0;out.panelFound=parking3.panelFound();out.panelHits=parking3.panelHits();out.parkingPath=parking3.path;
        }
        if(out.parkingArmed&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"){
            board.update(frame,now);
            if(cfg.parkingMode=="yellow"&&cfg.yellow.manualEnabled){
                if(!parking.side&&board.stopped&&board.candidate)parking.side=cfg.yellow.manualSide;
                LaneV2 noWhite;parking.update(board,noWhite,frame,now,!trafficHold&&now>=1);
            }else if(cfg.parkingMode=="yellow"&&cfg.yellow.sideBar.enabled){
                if(!parking.side)parking.side=yellow.selectBarSide(frame,board.box,board.candidate,board.stopped,now);
                LaneV2 noWhite;parking.update(board,noWhite,frame,now,!trafficHold&&now>=1);
            }else parking.update(board,geometry,frame,now,!trafficHold&&now>=1);
            if(asyncQr){
                parking.applyQR(asyncQr->poll(),now);
                if(parking.side&&!parking.hold&&!parking.stopped&&now>=nextQrSubmit){
                    asyncQr->submit(frame,geometry,parking,now);nextQrSubmit=now+.15;
                }
            }
            if(cfg.parkingMode=="yellow"){
                if(parking.parking_yellow_fixed_entry){
                    if(parking.side&&parking.entry.done()){
                        parking2.beginYellowWhite(parking.side);
                        parking2.update(frame,board,now,!trafficHold&&now>=1);
                    }
                    out.panelChecked=parking.side&&parking.entry.done();
                    out.panelFound=parking2.panelFound();out.panelHits=parking2.panelHits();
                }else{
                    yellow.update(frame,geometry,parking.side,now,!trafficHold&&now>=1,lastMotor>cfg.drive.neutral_pwm,parking.parking_reverse_seconds);
                    out.panelChecked=true;out.panelFound=yellow.panelFound();out.panelHits=yellow.panelHits();
                }
            }
            out.parking=cfg.parkingMode=="yellow"&&!parking.parking_yellow_fixed_entry&&(parking.side||yellow.panelActive())?yellow.state:parking.state;
        }
        else if((modules.parking||modules.parking2||modules.parking3)&&!out.parkingArmed)out.parking="WAIT_CONES";
        if(out.parkingArmed&&cfg.parkingMode=="yellow"&&!parking.parking_yellow_fixed_entry&&parking.side&&yellow.state=="PARK_FOLLOW"){
            geometry=whitePair.detect(frame,parking.side,now);laneMask=whitePair.mask;
            out.baseError=geometry.error;out.error=out.baseError;
            out.parkingPath=whitePair.source;out.left=whitePair.leftMeasured;out.right=whitePair.rightMeasured;
        }
        auto depth=depthLast;
        if(out.parkingArmed&&cfg.parkingMode=="depth"&&!trafficHold){auto seen=depthVision.process(frame,cfg.depth);depth=depthControl.step(seen,out.baseError,dt,cfg.depthDrive,cfg.depth);depthLast=depth;}
        if(out.parkingArmed&&cfg.parkingMode=="depth"){out.parking=depth.state;out.reason=depth.reason;}
        // 停车已接管时停止更新锥桶；固定绕桶的计时由末尾的实际命令反馈决定。
        // 本帧停车解锁在 cone.update 之前计算，因此本帧新增加的通过计数到下一帧用于解锁。
        bool parkingActive=out.parkingArmed&&(cfg.parkingMode=="parking3"?parking3.engaged():cfg.parkingMode=="parking2"?parking2.engaged():cfg.parkingMode!="depth"?(parking.side||board.stopped):(depth.state!="WAIT"&&depth.state!="SEEK"));
        if(modules.cone&&!parkingActive&&(!trafficHold||cone.fixedMode()||cone.laneMode()))
            cone.update(frame,geometry,now,!trafficHold&&now>=1);
        out.cone=modules.cone?cone.phase:"DISABLED";out.passed=cone.passedCount;
        if(modules.cone&&cone.laneMode()){
            const auto& v=cone.laneVisual();out.coneTarget=v.targetId;out.coneNext=v.nextTargetId;
            out.coneTracks=v.trackCount;out.coneSide=v.side;out.coneHandoff=v.handoff;
        }
        out.offset=parkingActive&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"?parking.offset:modules.cone&&!parkingActive?cone.offsetFraction:0;
        // 只叠加几何目标偏移带来的误差增量，避免用统一几何重算并覆盖原版 baseError。
        // 偏移为 0 时，增量相消，仍使用所选巡线后端原有的误差。
        out.error=out.baseError+geometry.targetError(out.offset)-geometry.targetError(0);
        // Visual cones replace a lane boundary, then use the SAME lane PD once.
        // Add only virtual-minus-real geometry, retaining each backend's native error units.
        if(modules.cone&&!parkingActive&&cone.laneOverride())out.error=out.baseError+cone.laneErrorDelta();
        // 目标持续保持在所选边线内侧，停车任务期间不再追车道中心。
        if(parkingActive&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"&&parking.edgeControl())out.error=parking.edgeError;
        // 最初 1 秒保持 WAIT；首次进入 RUN 时记录油门阶段起点。
        // 油门按 now-runStart 的经过时间分段，现有实现不会扣除中途等灯或斑马线等待时间。
        out.state=now<1?"WAIT":"RUN";
        if(out.state=="RUN"){
            if(runStart<0)runStart=now;out.motor=lane_throttle(std::llround((now-runStart)*1000),cfg.drive);
            if(modules.cone&&!parkingActive&&(cone.near||std::abs(out.offset)>.01)){out.state="CONE_"+cone.phase;if(out.motor>cfg.drive.neutral_pwm)out.motor=std::min(out.motor,cfg.drive.cone_slow_pwm);}
            // 普通巡线使用 lane PD；有锥桶横向偏移时使用 cone PD。
            // 直接绕桶模式在脉冲期间改用固定舵机修正，并同步 PD 历史误差以衔接恢复巡线。
            bool edgeDriving=parkingActive&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"&&parking.edgeControl();
            bool yellowPairOwnSteer=parkingActive&&cfg.parkingMode=="yellow"&&parking.parking_yellow_fixed_entry&&parking.entry.done()&&!parking2.useLane;
            bool parking2OwnSteer=parkingActive&&((cfg.parkingMode=="parking2"&&!parking2.useLane)||(cfg.parkingMode=="parking3"&&!parking3.useLane)||yellowPairOwnSteer);
            double requested;
            if(parking2OwnSteer){requested=filtered;pd.previous=out.baseError;}
            else if(cfg.parkingMode=="yellow"&&parking.side&&!parking.parking_yellow_fixed_entry&&yellow.state!="PARK_FOLLOW"){requested=filtered;pd.previous=out.baseError;}else if(edgeDriving){
                requested=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,cfg.drive.servo_center-parking.edgeCorrection()));
                pd.previous=out.baseError;
            }else requested=pd.update(out.error,cfg.drive,modules.cone&&!parkingActive&&!cone.laneMode()&&std::abs(out.offset)>.0001);
            if(modules.cone&&!parkingActive&&cone.directTurning){
                requested=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,cfg.drive.servo_center-cone.directCorrection()));
                pd.previous=out.baseError;
            }
            // 转向目标再经过每秒 PWM 变化率限制，单帧最多按 0.1 秒计算，随后取整输出。
            // 深度停车接管时会用它自己的电机和舵机结果覆盖这里的基础输出。
            double step=(edgeDriving?parking.parking_steer_rate:cfg.drive.steer_rate)*std::min(.1,dt);
            if(!parking2OwnSteer)filtered+=std::max(-step,std::min(step,requested-filtered));
            if(edgeDriving)filtered=std::max(cfg.drive.servo_center-parking.parking_steer_limit,std::min(cfg.drive.servo_center+parking.parking_steer_limit,filtered));
            out.steer=std::lround(filtered);out.target=filtered;
            if(parkingActive&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"){
                out.state=parking.state;
                if(parking.edgeControl())out.motor=parking.parking_follow_pwm;
                else if(parking.state=="WAIT_BOARD_SIDE")out.motor=parking.parking_wait_side_pwm;
                else if(out.motor>cfg.drive.neutral_pwm)out.motor=std::min(out.motor,cfg.drive.cone_slow_pwm);
            }
            if(parkingActive&&cfg.parkingMode=="depth"){out.state="PARK_"+depth.state;out.motor=depth.motor;out.steer=depth.steer;out.error=depth.error;out.target=out.steer;}
            // 固定动作在这里覆盖前面计算的巡线结果，不叠加 PD，也不经过 steer_rate。
            // lane_error 仍会显示，但不决定这一段转向；计时由逐帧 tick/feedback 推进。
            // fixedTurning 在表走完时变 false，之后保留前面计算的 RUN/巡线输出。
            bool fixedCone=modules.cone&&!parkingActive&&cone.fixedTurning;
            bool entryParking=parkingActive&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"&&parking.entryControl();
            bool fixedParking=parkingActive&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"&&parking.fixedControl();
            if(fixedCone||fixedParking||entryParking){
                double correction=entryParking?parking.entryCorrection():fixedParking?parking.fixedCorrection():cone.fixedCorrection();
                int motor=entryParking?parking.entryMotor():fixedParking?parking.fixedMotor():cone.fixedMotor();
                std::string state=(fixedParking||entryParking)?parking.state:"CONE_"+cone.phase;
                double remaining=entryParking?parking.entry.remaining():fixedParking?parking.fixedRemaining():cone.fixedRemaining();
                applyFixedCommand(out,correction,motor,state,remaining);
            }
        }else pd.previous=out.error;
        if(out.parkingArmed&&cfg.parkingMode=="timed"&&parking.state=="PARK_TIMED_REVERSE"){
            out.state=parking.state;out.motor=parking.parking_reverse_pwm;
            out.steer=int(cfg.drive.servo_center);filtered=cfg.drive.servo_center;out.target=filtered;
            out.brakeRemaining=std::max(0.,parking.reverseUntil-now);out.fixedRemaining=out.brakeRemaining;
        }
        if(out.parkingArmed&&cfg.parkingMode=="yellow"&&parking.parking_yellow_fixed_entry&&parking.side&&parking.entry.done()){
            out.state=out.parking=parking2.stopped?"PARK_PANEL_STOP":parking2.useLane?"PARK_FOLLOW_LANE":"PARK_FOLLOW_YELLOW_WHITE";
            out.parkingPath=parking2.path;out.motor=parking2.stopped?cfg.drive.neutral_pwm:cfg.yellow.motor;
            if(parking2.stopped){filtered=out.target=cfg.drive.servo_center;out.steer=int(filtered);}
            else if(!parking2.useLane){out.error=parking2.error;
                double target=cfg.drive.servo_center-parking2.correction;
                double delta=parking2.rate()*std::min(.1,dt);
                filtered+=std::max(-delta,std::min(delta,target-filtered));
                filtered=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,filtered));
                out.target=filtered;out.steer=std::lround(filtered);}
        }
        if(out.parkingArmed&&cfg.parkingMode=="yellow"&&!parking.parking_yellow_fixed_entry&&(parking.side||yellow.panelActive())){
            out.state=out.parking=yellow.state;
            if(yellow.state=="PARK_ENTRY_TURN"){
                filtered=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,cfg.drive.servo_center-yellow.correction));
                out.steer=std::lround(filtered);out.target=filtered;out.motor=yellow.entryMotor();out.fixedRemaining=yellow.entryRemaining();
            }else if(yellow.state=="PARK_FOLLOW"){
                // Preserve the ordinary white-lane error, PD and slew-limited steering above.
                out.motor=yellow.motor();
            }else{out.steer=int(cfg.drive.servo_center);filtered=out.target=cfg.drive.servo_center;out.motor=cfg.drive.neutral_pwm;
                if(yellow.state=="PARK_PANEL_BRAKE"){out.motor=parking.parking_reverse_pwm;out.brakeRemaining=yellow.remaining(now);}}
        }
        if(out.parkingArmed&&cfg.parkingMode=="parking2"&&parking2.engaged()){
            out.state=out.parking=parking2.state;out.parkingPath=parking2.path;
            out.motor=parking2.stopped?cfg.drive.neutral_pwm:parking2.motor();
            if(parking2.useLane&&!parking2.stopped){
                // 蓝挡板离开视野或缺少可信白边时，保留上方正常巡线的误差与舵机结果。
            }else{
            out.error=parking2.error;
            double target=cfg.drive.servo_center-parking2.correction;
            if(!parking2.stopped){double delta=parking2.rate()*std::min(.1,dt);
                filtered+=std::max(-delta,std::min(delta,target-filtered));
                filtered=std::max(cfg.drive.servo_min,std::min(cfg.drive.servo_max,filtered));
            }else filtered=cfg.drive.servo_center;
            out.target=filtered;out.steer=std::lround(filtered);
            }
        }
        if(out.parkingArmed&&cfg.parkingMode=="parking3"&&parking3.engaged()){
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
        // A single arbiter owns GPIO. Final parking stop outranks every other module.
        // 接近红绿灯时只限制前进油门，保留巡线转向；停车模块接管后仍用其既有控制。
        if(modules.light&&traffic.slowing()&&!parkingActive&&out.motor>cfg.drive.neutral_pwm){
            out.motor=std::min(out.motor,cfg.light.sequence.slowPwm);
            if(out.state=="RUN")out.state="LIGHT_APPROACH";
        }
        // 以下按代码顺序覆盖：锥桶停车 → 斑马线 → 红灯 → 二维码等待/终停 → 深度 DONE/FAULT。
        // 后面的已满足条件具有更高优先级；最后还统一处理起步等待和电机中位时的转向。
        if(modules.cone&&!parkingActive&&cone.holdStop&&now>=1){out.state="CONE_HOLD_STOP";out.motor=cfg.drive.neutral_pwm;}
        if(modules.zebra&&out.zebra=="BRAKE"){out.state="ZEBRA_BRAKE";out.motor=cfg.zebraBrakePwm;out.brakeRemaining=std::max(0.,zebraGate.began+zebraGate.brakeDuration-now);}
        if(modules.zebra&&out.zebra=="HOLD"){out.state="ZEBRA_HOLD";out.motor=cfg.drive.neutral_pwm;}
        if(modules.light&&traffic.active()){
            out.state=traffic.braking(now)?"LIGHT_BRAKE":"LIGHT_WAIT_GREEN";
            out.motor=traffic.braking(now)?cfg.light.brakePwm:cfg.drive.neutral_pwm;
            out.brakeRemaining=traffic.braking(now)?traffic.brakeRemaining(now):-1;
        }
        if(out.parkingArmed&&cfg.parkingMode!="depth"&&cfg.parkingMode!="parking2"&&cfg.parkingMode!="parking3"&&(parking.hold||parking.stopped)){out.state=parking.state;out.motor=cfg.drive.neutral_pwm;}
        if(out.parkingArmed&&cfg.parkingMode=="depth"&&(depth.state=="DONE"||depth.state=="FAULT")){out.state="PARK_"+depth.state;out.motor=depth.motor;out.steer=depth.steer;out.brakeRemaining=-1;}
        if(out.parkingArmed&&cfg.parkingMode=="parking2"&&parking2.stopped){out.state=out.parking="PARK2_PANEL_STOP";out.motor=cfg.drive.neutral_pwm;out.brakeRemaining=-1;}
        if(out.parkingArmed&&cfg.parkingMode=="parking3"&&parking3.stopped){out.state=out.parking="PARK3_PANEL_STOP";out.motor=cfg.drive.neutral_pwm;out.brakeRemaining=-1;}
        if(out.parkingArmed&&cfg.parkingMode=="yellow"&&yellow.stopped){out.state=out.parking="PARK_PANEL_STOP";out.motor=cfg.drive.neutral_pwm;out.brakeRemaining=-1;}
        if(now<1)out.motor=cfg.drive.neutral_pwm;
        // 最终电机为中位时，舵机也居中，并重置平滑目标及 PD 历史误差。
        // 深度控制器收到的是仲裁后的实际指令，用于下一帧估计行程；这不是编码器反馈。
        if(out.motor==cfg.drive.neutral_pwm){out.steer=int(cfg.drive.servo_center);filtered=cfg.drive.servo_center;pd.previous=out.error;}
        if(out.parkingArmed&&cfg.parkingMode=="depth")depthControl.feedback(out.motor,out.steer);
        if(out.motor>=cfg.drive.neutral_pwm)out.brakeRemaining=-1;
        // 仲裁后的命令决定下一帧是否累计绕行时间。交通停车不会让整套动作空耗完。
        cone.laneFeedback(modules.cone&&!parkingActive&&out.motor>cfg.drive.neutral_pwm);
        cone.fixedFeedback(modules.cone&&!parkingActive&&cone.fixedTurning&&out.motor>cfg.drive.neutral_pwm&&out.state=="CONE_"+cone.phase);
        parking.fixedFeedback(out.parkingArmed&&cfg.parkingMode!="depth"&&parking.fixedControl()&&out.motor>cfg.drive.neutral_pwm&&out.state==parking.state);
        parking.edgeFeedback(out.parkingArmed&&cfg.parkingMode!="depth"&&parking.edgeControl()&&out.motor>cfg.drive.neutral_pwm&&out.state==parking.state);
        parking.entryFeedback(out.parkingArmed&&cfg.parkingMode!="depth"&&parking.entryControl()&&out.motor>cfg.drive.neutral_pwm&&out.state==parking.state);
        out.frameDt=dt;out.parkingSide=cfg.parkingMode=="parking3"?parking3.side:cfg.parkingMode=="parking2"?parking2.side:parking.side;if(cfg.parkingMode=="yellow"&&parking.side&&yellow.state!="PARK_FOLLOW")out.parkingPath=yellow.pathSource;out.edgeHeading=parking.edgeHeading;out.qrHits=parking.qrHits;
        out.qrAge=parking.lastQrAt<0?-1:now-parking.lastQrAt;
        yellow.entryFeedback(out.parkingArmed&&cfg.parkingMode=="yellow"&&out.state=="PARK_ENTRY_TURN"&&out.motor>cfg.drive.neutral_pwm);
        traffic.feedback(out.motor>cfg.drive.neutral_pwm&&!parkingActive,
            out.motor>cfg.drive.neutral_pwm&&out.motor<=cfg.light.sequence.slowPwm&&!parkingActive);
        lastMotor=out.motor;
        return out;
    }
    // 诊断叠加只在原图副本上画边界、检测框和状态，不将画出的标记回送识别。
    cv::Mat draw(const cv::Mat& raw,const Command& c,int sent_motor_us=0)const{
        cv::Mat view=raw.clone();if(cfg.parkingMode=="yellow"&&cfg.yellow.sideBar.enabled&&!parking.side)yellow.drawSideBar(view);if(cfg.parkingMode=="yellow"&&!parking.parking_yellow_fixed_entry)yellow.draw(view);if(cfg.parkingMode=="yellow"&&!parking.parking_yellow_fixed_entry&&parking.side&&c.parking=="PARK_FOLLOW")whitePair.draw(view,geometry);else if(c.parking!="PARK3_FOLLOW_WHITE"&&!(modules.parking2&&(c.parking=="PARK2_FOLLOW_BLUE_WHITE"||c.parking=="PARK2_FOLLOW_YELLOW_WHITE"))&&c.parking!="PARK_FOLLOW_YELLOW_WHITE")geometry.draw(view,c.offset);if(modules.cone)cone.draw(view,c.cone);
        if((modules.parking||modules.parking2||modules.parking3)&&board.candidate)cv::rectangle(view,board.box,{255,200,0},2);
        if(modules.parking2||(modules.parking&&cfg.parkingMode=="yellow"&&parking.parking_yellow_fixed_entry&&parking.entry.done()))parking2.draw(view);
        if(cfg.parkingMode=="parking3")parking3.draw(view);
        if(modules.zebra&&stripes.found)cv::rectangle(view,stripes.box,{255,0,255},2);
        if(modules.light)traffic.draw(view,c.light);
        cv::putText(view,c.state+cv::format(" M%d S%d",c.motor,c.steer),{5,20},cv::FONT_HERSHEY_SIMPLEX,.5,{0,255,255},1);
        return view;
    }
};

