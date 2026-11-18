// 锥桶识别、目标跟踪与绕行决策；输入为 640×480 BGR 和巡线提供的边界。
// 室外按形状识别并使用连续视觉临时边界；室内保留蓝色识别和原有固定S动作。
// 未开启时保留其他配置使用的旧直接转向/偏移分支。
#pragma once
#include "lane_v2.hpp"
#include "fixed_maneuver.hpp"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "cone_config.hpp"
#include "cone_lane_visual.hpp"
#include "cone_shape.hpp"

class ConeAvoidV2 {
    struct Candidate { cv::Rect box; int pixels; double bottom() const {return box.y+box.height;} };
    struct Interval { double lo,hi; };
    ConeConfig cfg;
    ConeLaneVisual visual;
    bool tracking=false, confirmed=false, timeReady=false, exitEligible=false;
    int hits=0, side=0; // -1 passes on left; +1 passes on right; locked per tracked cone.
    double lastTime=0, lastSeen=0, firstSeen=0, minBottom=0, maxBottom=0;
    double directStarted=-1;
    FixedManeuver fixed;
    bool fixedStarted=false,sCompleted=false;
    double fixedFinished=-1;
    cv::Rect trackedBox;
    std::vector<Candidate> candidates;

    static double clamp(double v,double lo,double hi) {return std::max(lo,std::min(hi,v));}
    static double bottom(const cv::Rect& b) {return b.y+b.height;}
    static double centerX(const cv::Rect& b) {return b.x+b.width*0.5;}
    // 保守通道模式只认可两侧真实拟合；先要求 ROI 某处可见有序双线，再外推到锥桶底边。
    // 不把 LaneV2 的画面边缘补位当成已确认的道路边界，也不把出画面的近端线裁窄。
    bool bounds(const LaneV2& lane,double y,double& l,double& r) const {
        if(!lane.leftValid || !lane.rightValid) return false;
        // Both REAL fits must be visible at a reference row. Near the vehicle,
        // a genuine boundary may extend outside the image; never clip it into a
        // falsely narrower corridor. This does not certify image-edge fallbacks.
        // Different camera poses can place the horizon inside the original
        // outdoor ROI. Require a visible, ordered pair somewhere IN the ROI,
        // rather than assuming top+30 is always below the horizon.
        bool visiblePair=false;
        for(int row=lane.top;row<lane.bottom;row+=5){
            double a=lane.la*row+lane.lb,b=lane.ra*row+lane.rb;
            if(std::isfinite(a)&&std::isfinite(b)&&a>=0&&b<=639&&b-a>=30){visiblePair=true;break;}
        }
        l=lane.la*y+lane.lb;r=lane.ra*y+lane.rb;
        return visiblePair&&std::isfinite(l)&&std::isfinite(r)&&r-l>=30&&
               l>=-cfg.cone_lane_extrapolation_px&&r<=639+cfg.cone_lane_extrapolation_px;
    }
    // 下半幅 ROI 内按蓝色 HSV 提取连通块，保留小远桶，不先用腐蚀把它们抹掉。
    // 随后按面积、宽高、填充率筛选；只有真实通道边界可信时才排除完全在通道外的候选。
    void detect(const cv::Mat& frame,const LaneV2& lane) {
        candidates.clear(); mask=cv::Mat::zeros(frame.size(),CV_8UC1);
        if(cfg.cone_shape_mode){
            ConeShapeDetector shape;
            for(const auto& b:shape.detect(frame,cfg,mask)){
                double l,r;if(bounds(lane,bottom(b),l,r)&&(b.br().x<l||b.x>r))continue;
                candidates.push_back({b,b.area()});
            }
            std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){
                return a.bottom()!=b.bottom()?a.bottom()>b.bottom():a.pixels>b.pixels;
            });
            return;
        }
        cv::Mat hsv,roiMask;
        const cv::Rect roi(0,cfg.cone_roi_top,frame.cols,frame.rows-cfg.cone_roi_top);
        cv::cvtColor(frame(roi),hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(cfg.cone_h_min,cfg.cone_s_min,cfg.cone_v_min),
                    cv::Scalar(cfg.cone_h_max,255,255),roiMask);
        // Intentionally no opening/erosion: a distant cone can be only 6x9 pixels.
        roiMask.copyTo(mask(roi));
        cv::Mat labels,stats,centroids;
        int n=cv::connectedComponentsWithStats(roiMask,labels,stats,centroids,8,CV_32S);
        for(int i=1;i<n;++i) {
            cv::Rect b(stats.at<int>(i,cv::CC_STAT_LEFT),stats.at<int>(i,cv::CC_STAT_TOP)+roi.y,
                       stats.at<int>(i,cv::CC_STAT_WIDTH),stats.at<int>(i,cv::CC_STAT_HEIGHT));
            int pixels=stats.at<int>(i,cv::CC_STAT_AREA);
            double aspect=double(b.height)/b.width, fill=double(pixels)/b.area();
            // At the lower image edge a passing cone may be vertically truncated.
            double minAspect=bottom(b)>=frame.rows-1 ? std::min(0.25,cfg.cone_aspect_min) : cfg.cone_aspect_min;
            if(pixels<cfg.cone_min_pixels || b.width<2 || b.height<cfg.cone_min_height ||
               b.width>cfg.cone_max_width || aspect<minAspect || aspect>cfg.cone_aspect_max || fill<cfg.cone_fill_min) continue;
            double l,r;
            if(bounds(lane,bottom(b),l,r) && (b.x+b.width<l || b.x>r)) continue;
            candidates.push_back({b,pixels});
        }
        // 底边越靠下优先级越高；同一底边位置时优先蓝色像素更多的候选。
        std::sort(candidates.begin(),candidates.end(),[](const Candidate& a,const Candidate& b){
            if(a.bottom()!=b.bottom()) return a.bottom()>b.bottom();
            return a.pixels>b.pixels;
        });
    }
    // 用中心横向位移、底边位移及宽度比例关联上一帧目标；短时漏检会适当放宽匹配门限。
    // 限制底边明显回退，避免远处第二只桶替换当前近桶。
    int match(double now) const {
        int best=-1; double costBest=std::numeric_limits<double>::infinity();
        double gap=std::max(0.0,now-lastSeen);
        double xGate=std::max(28.0,trackedBox.width*2.0)+std::min(gap,0.5)*60;
        double yGate=std::max(24.0,trackedBox.height*1.7)+std::min(gap,0.5)*50;
        for(size_t i=0;i<candidates.size();++i) {
            const cv::Rect& b=candidates[i].box;
            double dx=std::abs(centerX(b)-centerX(trackedBox));
            double dy=bottom(b)-bottom(trackedBox);
            // A distant second cone must not replace a nearby tracked cone.
            if(dx>xGate || dy>yGate || dy < -std::max(16.0,trackedBox.height*0.45)) continue;
            double ratio=double(b.width)/std::max(1,trackedBox.width);
            if(ratio<0.40 || ratio>2.6) continue;
            double cost=dx/xGate+std::abs(dy)/yGate+0.15*std::abs(std::log(ratio));
            if(cost<costBest) {costBest=cost;best=int(i);}
        }
        return best;
    }
    // 建立新跟踪目标时清除选侧和通过资格；firstSeen/lastSeen 用于确认与漏检计时。
    void begin(const Candidate& c,double now) {
        tracking=true; confirmed=false; hits=1; side=0; exitEligible=false;
        trackedBox=c.box; firstSeen=lastSeen=now; minBottom=maxBottom=c.bottom();
    }
    // 清除当前桶的跟踪状态，不清零累计 passedCount；偏移回中由调用分支单独处理。
    void clearTrack() {
        tracking=false; confirmed=false; hits=0; side=0; exitEligible=false;
        trackedBox=cv::Rect(); active=false; near=false; box=cv::Rect();
    }
    // 横向偏移先限制在保留边缘余量的范围内，再做指数平滑和每秒最大变化率限制。
    // 此平滑用于通道偏移模式；直接转向模式每帧将 offsetFraction 设为 0。
    void smooth(double desired,double dt) {
        const double limit=0.5-cfg.cone_clearance_fraction;
        desired=clamp(desired,-limit,limit);
        double change=(desired-offsetFraction)*(1-std::exp(-dt/cfg.cone_smooth_seconds));
        change=clamp(change,-cfg.cone_offset_rate*dt,cfg.cone_offset_rate*dt);
        offsetFraction=clamp(offsetFraction+change,-limit,limit);
        if(std::abs(offsetFraction)<0.0001 && desired==0) offsetFraction=0;
    }
    // Remove every other close cone's footprint from a possible passage interval.
    // 将该深度处车道横向位置归一化到 0..1，扣除当前桶和车道边缘的余量。
    // 再从候选通道中减去纵向接近的其他桶占用区间，结果可包含多个剩余空隙。
    std::vector<Interval> intervals(int which,const LaneV2& lane,int selected) const {
        double l,r;
        if(!bounds(lane,bottom(trackedBox),l,r)) return {};
        const double m=cfg.cone_clearance_fraction,w=r-l;
        const double a=(trackedBox.x-l)/w-m, b=(trackedBox.x+trackedBox.width-l)/w+m;
        std::vector<Interval> available;
        Interval initial=which<0 ? Interval{m,std::min(1-m,a)} : Interval{std::max(m,b),1-m};
        if(initial.hi-initial.lo<cfg.cone_min_gap_fraction) return available;
        available.push_back(initial);
        for(size_t i=0;i<candidates.size();++i) {
            if(int(i)==selected) continue;
            const Candidate& other=candidates[i];
            if(other.bottom()<cfg.cone_avoid_start_y ||
               std::abs(other.bottom()-bottom(trackedBox))>cfg.cone_close_depth_px) continue;
            double ol,orr;
            if(!bounds(lane,other.bottom(),ol,orr)) return {};
            double ca=(other.box.x-ol)/(orr-ol)-m;
            double cb=(other.box.x+other.box.width-ol)/(orr-ol)+m;
            std::vector<Interval> next;
            for(const Interval& gap:available) {
                if(cb<=gap.lo || ca>=gap.hi) {next.push_back(gap);continue;}
                if(ca-gap.lo>=cfg.cone_min_gap_fraction) next.push_back({gap.lo,std::min(ca,gap.hi)});
                if(gap.hi-cb>=cfg.cone_min_gap_fraction) next.push_back({std::max(cb,gap.lo),gap.hi});
            }
            available.swap(next);
        }
        return available;
    }
    static Interval widest(const std::vector<Interval>& v) {
        Interval best{0,0}; for(const Interval& i:v) if(i.hi-i.lo>best.hi-best.lo) best=i; return best;
    }
public:
    // 公开结果由 Suite 消费：directTurning 请求固定舵机修正，holdStop 请求电机中位。
    // offsetFraction 请求相对车道宽度的目标偏移；passedCount 可解锁后续停车模块。
    bool laneMode()const {return cfg.cone_lane_mode!=0;}
    bool laneOverride()const {return laneMode()&&visual.controlling();}
    double laneErrorDelta()const {return visual.errorDelta;}
    bool laneCompleted()const {return laneMode()&&visual.completed();}
    void laneFeedback(bool moving){if(laneMode())visual.feedback(moving);}
    const ConeLaneVisual& laneVisual()const {return visual;}
    bool sCourseCompleted()const {return sCompleted;}
    std::string fixedPhase()const {
        return cfg.cone_s_course_once?std::string("S_")+(side>0?"RIGHT_":"LEFT_")+fixed.phase():"FIXED_"+fixed.phase();
    }
    bool fixedTurning=false;
    size_t fixedStepNumber()const {return fixed.active()?fixed.stepNumber():0;}
    bool fixedMode()const {return cfg.cone_fixed_maneuver!=0;}
    // 固定 S 的转向来源只有：本轮锁定的 side × 当前 YAML 段的 steer。
    // 此处不读取 lane_error、不做 PD；Suite 再用 center-correction 得到 PWM。
    double fixedCorrection()const {return fixedTurning?side*fixed.step().steer:0.;}
    int fixedMotor()const {return fixed.step().motor;}
    double fixedRemaining()const {return fixed.remaining();}
    void fixedFeedback(bool applied){fixed.feedback(applied);}
    bool directTurning=false;
    double directCorrection() const {return directTurning?side*cfg.cone_turn_pwm:0.;}
    bool directMode() const {return cfg.cone_direct_turn!=0;}
    bool active=false, holdStop=false, near=false;
    cv::Rect box;
    cv::Mat mask;
    std::string phase="SEARCH";
    double offsetFraction=0;
    int passedCount=0;
    int hitCount() const {return hits;}
    explicit ConeAvoidV2(ConeConfig c=ConeConfig()):cfg(c),visual(c) {cfg.validate();}

    // 时间必须单调；用于偏移平滑的 dt 限制在 0..0.1 秒，识别状态仍使用实际时间差。
    // 一次触发 S 分三种状态：触发前识别/确认 → 启动后只按时间走表 → 完成锁存。
    // 启动前的距离判断是图像坐标条件，不是厘米；不锁定相同的物理起步位置或车头角度。
    void update(const cv::Mat& frame,const LaneV2& lane,double timeSeconds,bool canStart=true) {
        if(frame.empty() || frame.type()!=CV_8UC3 || frame.cols!=640 || frame.rows!=480 || !std::isfinite(timeSeconds))
            throw std::invalid_argument("ConeAvoidV2 requires BGR 640x480 and finite time");
        if(timeReady && timeSeconds<lastTime) throw std::invalid_argument("Cone time must be monotonic");
        double dt=timeReady ? clamp(timeSeconds-lastTime,0.0,0.1) : 1.0/30;
        timeReady=true; lastTime=timeSeconds; holdStop=false;
        // 室内一次触发 S 路线：启动后完全跳过锥桶检测，不跟踪/确认第二桶。
        // 所有帧仍由主循环驱动，但这里不再看桶、不等待第二桶，也不因丢线改段。
        // 全部动作结束即恢复巡线，不等待目标消失；完成锁存到程序重启。
        if(cfg.cone_s_course_once&&fixedStarted){
            fixed.tick(timeSeconds);offsetFraction=0;directTurning=false;
            fixedTurning=fixed.active();candidates.clear();mask=cv::Mat::zeros(frame.size(),CV_8UC1);
            if(fixedTurning){phase=fixedPhase();return;}
            // S_DONE / passed=1 只表示动作表走完，不表示视觉确认绕桶或车身回正。
            if(!sCompleted){++passedCount;sCompleted=true;clearTrack();}
            phase="S_DONE";return;
        }
        if(laneMode()&&visual.searchFinished()){
            candidates.clear();mask=cv::Mat::zeros(frame.size(),CV_8UC1);
        }else detect(frame,lane);
        if(laneMode()){
            std::vector<cv::Rect> observations;for(const auto& c:candidates)observations.push_back(c.box);
            visual.update(observations,lane,timeSeconds,canStart);
            active=visual.controlling();near=visual.near;holdStop=false;box=visual.box;
            offsetFraction=visual.offset;passedCount=visual.passed;phase=visual.phase;
            fixedTurning=directTurning=false;return;
        }
        // 直接转向分支：确认目标接近后，按桶在车道中线哪侧，锁定从另一侧通过。
        // 该模式允许使用常规单侧边缘补位计算中线，不使用下面的双真实边界通道停走条件。
        if(cfg.cone_fixed_maneuver){
            fixed.tick(timeSeconds);fixedTurning=false;directTurning=false;offsetFraction=0;
            if(!tracking){
                if(!canStart||candidates.empty()){phase="SEARCH";near=active=false;box={};return;}
                begin(candidates[0],timeSeconds);fixedStarted=false;fixedFinished=-1;
            }else{
                int found=match(timeSeconds);
                if(found>=0){
                    if(!confirmed&&hits==0)firstSeen=timeSeconds;
                    trackedBox=candidates[found].box;lastSeen=timeSeconds;++hits;
                }else if(!confirmed){hits=0;firstSeen=timeSeconds;}
            }
            // 首桶候选达到连续确认条件，且框底边达到起绕像素行，才允许触发。
            box=trackedBox;near=bottom(box)>=cfg.cone_avoid_start_y;
            confirmed=confirmed||(hits>=cfg.cone_confirm_frames&&timeSeconds-firstSeen+1e-9>=cfg.cone_confirm_seconds);
            active=confirmed;
            if(!fixedStarted){
                if(timeSeconds-lastSeen>cfg.cone_lost_grace_s){clearTrack();phase="SEARCH";return;}
                if(!confirmed||!near||!canStart){phase=confirmed?"FAR":"CONFIRM";return;}
                double mid=(lane.leftAt(bottom(box))+lane.rightAt(bottom(box)))*.5;
                // 非零 cone_fixed_side 强制方向；0 才根据桶相对车道中线的位置选反侧。
                // side 在本轮 S 内不再修改；例如 fixed_side=1 始终使用计划右绕。
                side=cfg.cone_fixed_side?cfg.cone_fixed_side:(centerX(box)<=mid?1:-1);
                fixed.start(cfg.fixedPlan,timeSeconds);fixedStarted=true;
            }
            // 动作开始后不再因目标消失、缺线或误差变化而提前退出。
            if(fixed.active()){fixedTurning=true;phase=fixedPhase();return;}
            if(fixedFinished<0)fixedFinished=timeSeconds;
            phase="WAIT_CLEAR";
            // 完成全部动作 + 目标离开 + 冷却后才记一次通过，避免反复绕同一桶。
            if(canStart&&timeSeconds-fixedFinished>=cfg.cone_rearm_seconds&&timeSeconds-lastSeen>=cfg.cone_lost_grace_s){
                ++passedCount;clearTrack();fixedStarted=false;phase="PASS_RECENTER";
            }
            return;
        }
        if(cfg.cone_direct_turn){
            directTurning=false;offsetFraction=0;
            if(!tracking){
                if(candidates.empty()){phase="SEARCH";near=false;active=false;box=cv::Rect();return;}
                begin(candidates[0],timeSeconds);directStarted=-1;
            }else{
                int found=match(timeSeconds);
                if(found>=0){trackedBox=candidates[found].box;lastSeen=timeSeconds;++hits;}
            }
            box=trackedBox;near=bottom(box)>=cfg.cone_avoid_start_y;
            confirmed=confirmed||(hits>=cfg.cone_confirm_frames&&timeSeconds-firstSeen>=cfg.cone_confirm_seconds);
            active=confirmed;
            if(directStarted<0){
                if(timeSeconds-lastSeen>cfg.cone_lost_grace_s){clearTrack();phase="SEARCH";return;}
                if(!confirmed||!near){phase=confirmed?"FAR":"CONFIRM";return;}
                // Choose once from the observed cone position, with the normal single-line fallback.
                double mid=(lane.leftAt(bottom(box))+lane.rightAt(bottom(box)))*.5;
                side=centerX(box)<=mid?1:-1;directStarted=timeSeconds;
            }
            // 在 cone_turn_seconds 内持续请求 side×cone_turn_pwm；实际 PWM 和转向限速由 Suite 处理。
            // 脉冲到期恢复巡线，但继续跟踪同一桶，不因其仍然可见而重复触发。
            if(timeSeconds-directStarted<cfg.cone_turn_seconds){
                directTurning=true;phase=side>0?"AVOID_RIGHT":"AVOID_LEFT";return;
            }
            // Resume lane following after the pulse. Do not retrigger the same visible cone.
            phase="WAIT_CLEAR";
            // 直接模式在 WAIT_CLEAR 且目标消失超过宽限时累计一次通过；它没有通道模式的位移资格检查。
            if(timeSeconds-lastSeen>=cfg.cone_lost_grace_s){
                ++passedCount;clearTrack();directStarted=-1;phase="PASS_RECENTER";
            }
            return;
        }
        // 以下为 cone_direct_turn=0 的通道模式：跟踪最近候选，并基于真实边界选择通道。
        int selected=-1;
        if(!tracking) {
            if(!candidates.empty()) {selected=0;begin(candidates[0],timeSeconds);}
            else {active=false;near=false;box=cv::Rect();phase="SEARCH";smooth(0,dt);return;}
        } else {
            selected=match(timeSeconds);
            if(selected>=0) {
                if(!confirmed&&hits==0) firstSeen=timeSeconds;
                trackedBox=candidates[selected].box; lastSeen=timeSeconds; ++hits;
                minBottom=std::min(minBottom,bottom(trackedBox)); maxBottom=std::max(maxBottom,bottom(trackedBox));
            } else {
                if(!confirmed) {hits=0;firstSeen=timeSeconds;}
                active=confirmed;box=trackedBox;near=bottom(trackedBox)>=cfg.cone_avoid_start_y;
                double missed=timeSeconds-lastSeen;
                // 短时遮挡保留上一目标；近处边界不可信或又出现很近的桶时，请求停车等待。
                if(missed<=cfg.cone_lost_grace_s) {
                    phase="OCCLUDED";
                    double l,r;
                    holdStop=near&&!bounds(lane,bottom(trackedBox),l,r);
                    for(const Candidate& c:candidates) if(c.bottom()>=cfg.cone_full_avoid_y) holdStop=true;
                    return; // Preserve the last target during a short detection dropout.
                }
                // 通道模式只有曾满足接近位移/时间资格且底边到达通过阈值，失去目标后才计为通过。
                // 近桶无合格通过证据却长时间丢失时保持 LOST_NEAR_STOP，不把漏检直接当通过。
                if(confirmed && exitEligible && bottom(trackedBox)>=cfg.cone_pass_y) {
                    ++passedCount;clearTrack();phase="PASS_RECENTER";smooth(0,dt);
                    // Do not drive blindly into a second close obstacle while it is confirming.
                    for(const Candidate& c:candidates) if(c.bottom()>=cfg.cone_full_avoid_y) holdStop=true;
                    return;
                }
                if(near) {holdStop=true;phase="LOST_NEAR_STOP";return;}
                clearTrack();phase="SEARCH";smooth(0,dt);return;
            }
        }
        confirmed=confirmed || (hits>=cfg.cone_confirm_frames && timeSeconds-firstSeen+1e-9>=cfg.cone_confirm_seconds);
        active=confirmed;box=trackedBox;near=bottom(box)>=cfg.cone_avoid_start_y;
        // 通过资格要求：已确认、底边足够近、观察到足够向下移动，并持续足够接近时间。
        if(confirmed && bottom(box)>=cfg.cone_pass_y &&
           maxBottom-minBottom>=cfg.cone_approach_pixels && timeSeconds-firstSeen>=cfg.cone_approach_seconds)
            exitEligible=true;
        if(!confirmed) {
            phase="CONFIRM";holdStop=bottom(box)>=cfg.cone_full_avoid_y;smooth(0,dt);return;
        }
        if(!near) {phase="FAR";smooth(0,dt);return;}
        double l,r;
        if(!bounds(lane,bottom(box),l,r)) {holdStop=true;phase="LANE_UNCERTAIN_STOP";return;}
        // 各侧选最大可用空隙，第一次选择更宽的一边并锁定；已选侧被堵时停车，不突然换边。
        Interval left=widest(intervals(-1,lane,selected)), right=widest(intervals(1,lane,selected));
        if(side==0) {
            double lw=left.hi-left.lo,rw=right.hi-right.lo;
            if(std::max(lw,rw)<cfg.cone_min_gap_fraction) {holdStop=true;phase="BLOCKED_STOP";return;}
            side=rw>=lw ? 1 : -1;
        }
        Interval chosen=side>0?right:left;
        if(chosen.hi-chosen.lo<cfg.cone_min_gap_fraction) {
            // Never reverse the committed passing side merely because another cone appeared.
            holdStop=true;phase="BLOCKED_STOP";return;
        }
        // 接近时逐步增加绕行偏移，起始强度由 cone_start_blend 决定，再经 smooth 平滑。
        // 很近时若平滑后的目标还没进入可用空隙，先请求 TARGET_TRANSITION_STOP。
        double blend=clamp((bottom(box)-cfg.cone_avoid_start_y)/(cfg.cone_full_avoid_y-cfg.cone_avoid_start_y),0,1);
        blend=cfg.cone_start_blend+(1-cfg.cone_start_blend)*blend;
        smooth(((chosen.lo+chosen.hi)*0.5-0.5)*blend,dt);
        phase=side>0?"AVOID_RIGHT":"AVOID_LEFT";
        // A sudden close detection can request a target faster than smoothing permits.
        // Keep the motor neutral until the requested target enters the available interval.
        double target=0.5+offsetFraction;
        if(bottom(box)>=cfg.cone_full_avoid_y && (target<chosen.lo-0.001 || target>chosen.hi+0.001)) {
            holdStop=true;phase="TARGET_TRANSITION_STOP";
        }
    }
    // 绘制所有候选和当前跟踪框，显示阶段、通过计数与横向偏移；红框表示请求停车。
    void draw(cv::Mat& image,const std::string& displayedPhase="") const {
        if(laneMode())visual.draw(image);
        else{
            for(const Candidate& c:candidates) cv::rectangle(image,c.box,cv::Scalar(150,100,0),1);
            if(box.area()>0) cv::rectangle(image,box,holdStop?cv::Scalar(0,0,255):cv::Scalar(255,220,0),2);
        }
        // Test stops can override the next internal phase before it reaches Output.
        std::string text="CONE "+(displayedPhase.empty()?phase:displayedPhase)+" pass="+std::to_string(passedCount)+(laneMode()?"/"+std::to_string(cfg.cone_lane_count):"")+" offset="+cv::format("%.3f",offsetFraction);
        cv::putText(image,text,cv::Point(5,49),cv::FONT_HERSHEY_SIMPLEX,0.43,
                    holdStop?cv::Scalar(0,0,255):cv::Scalar(255,255,0),1,cv::LINE_AA);
    }
};
