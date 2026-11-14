// 图像半区停车：确认蓝挡板占据哪半边 → 缓慢偏向另一半 → 连续检测二维码后锁存停车。
// 识别只做二维码定位，不解码内容；普通白板或只有蓝挡板不能直接满足二维码终停条件。
#pragma once
#include "blue_board.hpp"
#include "lane_v2.hpp"
#include "fixed_maneuver.hpp"
// parking_* 偏移/死区使用车道宽度比例；qr_* 尺寸和行号使用原始图像像素。
// 同一停车任务中 side 一旦确认便保持：+1 为图像右侧空区，-1 为左侧空区。
struct ParkingQrObservation {
    bool ready=false,found=false;int side=0;double capturedAt=0;
    cv::Rect box;
};
struct Parking {
    // 选定挡板反侧后持续沿该边线走，不执行定时绕行或回中。
    int parking_edge_follow=0,parking_follow_pwm=10900;
    double parking_edge_margin_fraction=.18,edgeError=0;
    double parking_shift_rate=.10,parking_pd_kp=.18,parking_pd_kd=.08;
    double parking_steer_limit=3.,parking_steer_rate=6.,parking_error_smooth_seconds=.20;
    // 室内新流程：固定侧移、反打摆正，然后闭环沿边；不会重新回到中线。
    int parking_edge_entry=0,parking_qr_async=0;
    double parking_heading_kp=0,edgeHeading=0,qrValidSince=0,lastQrAt=-1;
    FixedPlan entryPlan;FixedManeuver entry;bool entryStarted=false,deferQr=false;
    bool entryControl()const {return parking_edge_follow&&parking_edge_entry&&entryStarted&&entry.active()&&!hold&&!stopped;}
    double entryCorrection()const {return side*entry.step().steer;}
    int entryMotor()const {return entry.step().motor;}
    void entryFeedback(bool applied){entry.feedback(applied);}
    bool edgeReady=false,edgeApplied=false,edgePdReady=false;
    double edgePreviousError=0;
    void edgeFeedback(bool applied){edgeApplied=applied;if(!applied)edgePdReady=false;}
    double edgeCorrection(){
        double change=edgePdReady?edgeError-edgePreviousError:0.;
        edgePreviousError=edgeError;edgePdReady=true;
        return std::max(-parking_steer_limit,std::min(parking_steer_limit,parking_pd_kp*edgeError+parking_heading_kp*edgeHeading+parking_pd_kd*change));
    }
    double edgeWidthA=0,edgeWidthB=0;bool edgeWidthReady=false;
    bool edgeControl()const {return parking_edge_follow&&side&&!hold&&!stopped&&(!parking_edge_entry||entry.done());}
    bool edgeBounds(const LaneV2& lane,double y,double& l,double& r)const {
        double w=edgeWidthA*y+edgeWidthB;
        bool haveWidth=edgeWidthReady&&std::isfinite(w)&&w>=10&&w<=1280;
        if(haveWidth&&side>0&&lane.rightValid){r=lane.ra*y+lane.rb;l=r-w;}
        else if(haveWidth&&side>0&&lane.leftValid){l=lane.la*y+lane.lb;r=l+w;}
        else if(haveWidth&&side<0&&lane.leftValid){l=lane.la*y+lane.lb;r=l+w;}
        else if(haveWidth&&side<0&&lane.rightValid){r=lane.ra*y+lane.rb;l=r-w;}
        else {l=lane.leftAt(y);r=lane.rightAt(y);}
        return std::isfinite(l)&&std::isfinite(r)&&r-l>=10;
    }
    bool updateEdge(const LaneV2& lane,double dt) {
        if(lane.leftValid&&lane.rightValid){
            double a=lane.ra-lane.la,b=lane.rb-lane.lb;
            double near=a*lane.bottom+b,far=a*lane.top+b;
            if(std::isfinite(near)&&std::isfinite(far)&&near>=parking_min_lane_width&&near<=1280&&far>=10&&far<=1280){
                double alpha=edgeWidthReady?1-std::exp(-dt/.3):1.;
                edgeWidthA+=alpha*(a-edgeWidthA);edgeWidthB+=alpha*(b-edgeWidthB);edgeWidthReady=true;
            }
        }
        double midpoint=0,width=0,weights=0;
        for(int i=10;i<86;i+=2){
            double y=lane.top+(lane.bottom-lane.top-1)*i/95.0,l,r;
            if(!edgeBounds(lane,y,l,r))return false;
            double weight=std::min(8,1+std::min(i,95-i)/6);
            midpoint+=(l+r)*.5*weight;width+=(r-l)*weight;weights+=weight;
        }
        midpoint/=weights;width/=weights;
        bool first=!edgeReady;
        if(parking_edge_entry){
            // 用车道宽度归一化近端距离与远近差。远近差反映车头朝向，提前反打，
            // 避免只盯平均横向距离，到边线才摆正。相同横向偏差的平行线不会产生假航向。
            offset=side*(.5-parking_edge_margin_fraction);
            double yn=lane.top+(lane.bottom-lane.top)*.85,yf=lane.top+(lane.bottom-lane.top)*.20;
            double ln,rn,lf,rf;if(!edgeBounds(lane,yn,ln,rn)||!edgeBounds(lane,yf,lf,rf))return false;
            double en=((ln+rn)*.5+(rn-ln)*offset-320.)/(rn-ln);
            double ef=((lf+rf)*.5+(rf-lf)*offset-320.)/(rf-lf);
            double alpha=first?1.:1.-std::exp(-dt/parking_error_smooth_seconds);
            edgeError+=alpha*(40.*en-edgeError);edgeHeading+=alpha*(40.*(ef-en)-edgeHeading);
            edgeReady=true;if(first)edgePdReady=false;
            return true;
        }
        if(first){
            // 以当前车辆图像中心初始化目标，不把目标一步跳到另一半车道。
            offset=std::max(-.5,std::min(.5,(320.-midpoint)/width));
            edgeError=0;edgeReady=true;edgePdReady=false;
        }else if(edgeApplied){
            double goal=side*(.5-parking_edge_margin_fraction);
            offset+=std::max(-parking_shift_rate*dt,std::min(parking_shift_rate*dt,goal-offset));
        }
        double requested=(midpoint+width*offset-320.)/8.;
        if(!first)edgeError+=(1-std::exp(-dt/parking_error_smooth_seconds))*(requested-edgeError);
        return true;
    }
    int parking_fixed_maneuver=0,parking_fixed_side=0,parking_qr_seek_pwm=10900;
    double parking_maneuver_start_y=340.,parking_qr_seek_steer=0.;
    FixedPlan fixedPlan;FixedManeuver fixed;
    bool fixedStarted=false;
    bool fixedControl()const {return !parking_edge_follow&&parking_fixed_maneuver&&fixedStarted&&!hold&&!stopped;}
    double fixedCorrection()const {return side*(fixed.active()?fixed.step().steer:parking_qr_seek_steer);}
    int fixedMotor()const {return fixed.active()?fixed.step().motor:parking_qr_seek_pwm;}
    double fixedRemaining()const {return fixed.remaining();}
    void fixedFeedback(bool applied){fixed.feedback(applied);}
    void validateFixedMotor(int neutral)const {
        if(parking_edge_entry)entryPlan.validateMotor(neutral,"parking_edge_entry_steps");
        if(parking_edge_follow&&parking_follow_pwm<=neutral)throw std::runtime_error("parking_follow_pwm must exceed neutral");
        if(parking_fixed_maneuver){fixedPlan.validateMotor(neutral,"parking_maneuver_steps");
            if(parking_qr_seek_pwm<=neutral)throw std::runtime_error("parking_qr_seek_pwm must exceed neutral");}
    }
    double parking_side_deadband=0.04;
    double parking_min_lane_width=60;
    int parking_side_frames=3;
    double parking_target_offset=0.25;
    double parking_offset_rate=0.35;
    int qr_min_size=20;
    int qr_min_y=100;
    double qr_match_iou=0.3;
    int qr_confirm_frames=3;
    int qr_detect_scale=2;
    int qr_panel_v_min=110,qr_panel_s_max=80;
    // 仅加载存在的字段，并验证阈值、确认帧数和放大倍率；这里不改变巡线参数。
    void load(const cv::FileStorage& f){
#define F(n) if(!f[#n].empty())f[#n]>>n
        F(parking_edge_follow);F(parking_follow_pwm);F(parking_edge_margin_fraction);
        F(parking_edge_entry);F(parking_qr_async);F(parking_heading_kp);
        F(parking_shift_rate);F(parking_pd_kp);F(parking_pd_kd);F(parking_steer_limit);F(parking_steer_rate);F(parking_error_smooth_seconds);
        F(parking_fixed_maneuver);F(parking_fixed_side);F(parking_maneuver_start_y);
        F(parking_qr_seek_pwm);F(parking_qr_seek_steer);
#undef F
        if((parking_edge_entry!=0&&parking_edge_entry!=1)||(parking_qr_async!=0&&parking_qr_async!=1)||
           (parking_edge_entry&&!parking_edge_follow)||!std::isfinite(parking_heading_kp)||parking_heading_kp<0||parking_heading_kp>3)
            throw std::runtime_error("Invalid parking entry/heading/async configuration");
        if(parking_edge_entry){
            entryPlan.load(f,"parking_edge_entry_steps");
            if(entryPlan.steps.size()!=3||entryPlan.steps[0].steer<=0||entryPlan.steps[1].steer>=0||entryPlan.steps[2].steer!=0)
                throw std::runtime_error("parking_edge_entry_steps requires OUT(+), ALIGN(-), SETTLE(0)");
        }
        for(double v:{parking_shift_rate,parking_pd_kp,parking_pd_kd,parking_steer_limit,parking_steer_rate,parking_error_smooth_seconds})
            if(!std::isfinite(v))throw std::runtime_error("Non-finite parking steering parameter");
        if(parking_shift_rate<=0||parking_shift_rate>1||parking_pd_kp<0||parking_pd_kp>2||parking_pd_kd<0||parking_pd_kd>2||
           parking_steer_limit<1||parking_steer_limit>15||parking_steer_rate<=0||parking_steer_rate>100||
           parking_error_smooth_seconds<.02||parking_error_smooth_seconds>2)
            throw std::runtime_error("Invalid parking steering parameters");
        if((parking_edge_follow!=0&&parking_edge_follow!=1)||(parking_edge_follow&&parking_fixed_maneuver)||
           parking_follow_pwm<7000||parking_follow_pwm>12000||!std::isfinite(parking_edge_margin_fraction)||
           parking_edge_margin_fraction<.08||parking_edge_margin_fraction>.45)
            throw std::runtime_error("Invalid parking edge-follow settings (exclusive of fixed maneuver)");
        if((parking_fixed_maneuver!=0&&parking_fixed_maneuver!=1)||parking_fixed_side< -1||parking_fixed_side>1||
           !std::isfinite(parking_maneuver_start_y)||parking_maneuver_start_y<0||parking_maneuver_start_y>479||
           !std::isfinite(parking_qr_seek_steer)||std::abs(parking_qr_seek_steer)>31||parking_qr_seek_pwm<7000||parking_qr_seek_pwm>12000)
            throw std::runtime_error("Invalid fixed parking settings");
        if(parking_fixed_maneuver)fixedPlan.load(f,"parking_maneuver_steps");
        if(!f["qr_detect_scale"].empty())f["qr_detect_scale"]>>qr_detect_scale;
        if(qr_detect_scale<1||qr_detect_scale>3)throw std::runtime_error("qr_detect_scale must be 1..3");
        if(!f["qr_panel_v_min"].empty())f["qr_panel_v_min"]>>qr_panel_v_min;
        if(!f["qr_panel_s_max"].empty())f["qr_panel_s_max"]>>qr_panel_s_max;
        if(qr_panel_v_min<0||qr_panel_v_min>255||qr_panel_s_max<0||qr_panel_s_max>255)throw std::runtime_error("Invalid QR panel thresholds");
        if(!f["parking_side_deadband"].empty())f["parking_side_deadband"]>>parking_side_deadband;
        if(!std::isfinite(double(parking_side_deadband))||parking_side_deadband<0||parking_side_deadband>0.2)throw std::runtime_error("Invalid parking_side_deadband");
        if(!f["parking_min_lane_width"].empty())f["parking_min_lane_width"]>>parking_min_lane_width;
        if(!std::isfinite(double(parking_min_lane_width))||parking_min_lane_width<20||parking_min_lane_width>500)throw std::runtime_error("Invalid parking_min_lane_width");
        if(!f["parking_side_frames"].empty())f["parking_side_frames"]>>parking_side_frames;
        if(!std::isfinite(double(parking_side_frames))||parking_side_frames<1||parking_side_frames>30)throw std::runtime_error("Invalid parking_side_frames");
        if(!f["parking_target_offset"].empty())f["parking_target_offset"]>>parking_target_offset;
        if(!std::isfinite(double(parking_target_offset))||parking_target_offset<0.05||parking_target_offset>0.4)throw std::runtime_error("Invalid parking_target_offset");
        if(!f["parking_offset_rate"].empty())f["parking_offset_rate"]>>parking_offset_rate;
        if(!std::isfinite(double(parking_offset_rate))||parking_offset_rate<0.05||parking_offset_rate>1.0)throw std::runtime_error("Invalid parking_offset_rate");
        if(!f["qr_min_size"].empty())f["qr_min_size"]>>qr_min_size;
        if(!std::isfinite(double(qr_min_size))||qr_min_size<8||qr_min_size>200)throw std::runtime_error("Invalid qr_min_size");
        if(!f["qr_min_y"].empty())f["qr_min_y"]>>qr_min_y;
        if(!std::isfinite(double(qr_min_y))||qr_min_y<0||qr_min_y>450)throw std::runtime_error("Invalid qr_min_y");
        if(!f["qr_match_iou"].empty())f["qr_match_iou"]>>qr_match_iou;
        if(!std::isfinite(double(qr_match_iou))||qr_match_iou<0.05||qr_match_iou>0.9)throw std::runtime_error("Invalid qr_match_iou");
        if(!f["qr_confirm_frames"].empty())f["qr_confirm_frames"]>>qr_confirm_frames;
        if(!std::isfinite(double(qr_confirm_frames))||qr_confirm_frames<2||qr_confirm_frames>30)throw std::runtime_error("Invalid qr_confirm_frames");
    }

    // sideHits/proposal 用于空区选侧确认，qrHits/previousQR 用于二维码跨帧匹配。
    // hold 是当前帧等待请求，stopped 是最终停车锁存；两者由上层转换为电机中位。
    int side=0,sideHits=0,proposal=0,qrHits=0;bool stopped=false,hold=false;
    double offset=0,last=-1;int boardGroundY=0;cv::Rect previousQR;cv::QRCodeDetector qr;
    std::string state="SEARCH_BOARD";
    // 按顺序尝试原图定位、整图放大定位、亮面板候选局部放大定位。
    // 放大或裁剪找到的四角都会换回原图坐标，后续位置判断始终用同一尺度。
    bool detectQR(const cv::Mat& frame,cv::Mat& points){
        if(qr.detect(frame,points))return true;
        if(qr_detect_scale>1){cv::Mat enlarged;cv::resize(frame,enlarged,cv::Size(),qr_detect_scale,qr_detect_scale,cv::INTER_CUBIC);if(qr.detect(enlarged,points)){points/=qr_detect_scale;return true;}}
        // A small QR can be missed in the full image. Find bright, low-saturation
        // panel candidates ABOVE the observed blue-board foot, then run the SAME
        // QR detector on those crops. A plain panel is never a stop marker.
        // 局部搜索只看已观测蓝挡板脚点上方且低于 qr_min_y 的亮面板，最多尝试最大的 8 个。
        // 面板只是缩小搜索范围，仍必须通过同一个 QRCodeDetector::detect。
        if(boardGroundY<=qr_min_y+qr_min_size)return false;
        cv::Mat hsv,mask;cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
        cv::inRange(hsv,cv::Scalar(0,0,qr_panel_v_min),cv::Scalar(179,qr_panel_s_max,255),mask);
        mask.rowRange(0,qr_min_y).setTo(0);
        mask.rowRange(std::min(frame.rows,boardGroundY-4),frame.rows).setTo(0);
        std::vector<std::vector<cv::Point>> contours;cv::findContours(mask,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);
        std::vector<cv::Rect> candidates;
        for(const auto& contour:contours){auto b=cv::boundingRect(contour);double ratio=double(b.width)/b.height;
            if(b.width>=qr_min_size&&b.height>=qr_min_size&&b.area()>=qr_min_size*qr_min_size*2&&ratio>.35&&ratio<2.5&&b.width<frame.cols*.7)candidates.push_back(b);}
        std::sort(candidates.begin(),candidates.end(),[](const cv::Rect& a,const cv::Rect& b){return a.area()>b.area();});
        for(size_t i=0;i<std::min(size_t(8),candidates.size());++i){auto b=candidates[i];cv::Mat crop;cv::resize(frame(b),crop,cv::Size(),qr_detect_scale,qr_detect_scale,cv::INTER_CUBIC);
            if(qr.detect(crop,points)){points/=qr_detect_scale;auto* p=points.ptr<cv::Point2f>();for(size_t j=0;j<points.total();++j)p[j]+=cv::Point2f(float(b.x),float(b.y));return true;}}
        return false;
    }
    // dt 最多按 0.1 秒推进偏移；已经 stopped 后直接维持 QR_STOP_LATCHED。
    // boardGroundY 保留最近看到的挡板底边，作为悬空二维码与地面边界比较的共同深度行。
    void update(const BlueBoardStop& board,const LaneV2& lane,const cv::Mat& frame,double now,bool canStart=true){
        double dt=last<0?0:std::max(0.0,std::min(.1,now-last));last=now;hold=false;
        if(parking_fixed_maneuver)fixed.tick(now);
        if(parking_edge_entry)entry.tick(now);
        if(stopped){state="QR_STOP_LATCHED";return;}
        if(board.candidate)boardGroundY=board.box.br().y;
        // 只有挡板已确认、当前仍可见且两侧真实车道线有效时，才按挡板位于哪半边提出反侧。
        // 靠近中线死区或车道过窄时不选侧；同一非零提案连续达到帧数后锁定。
        if(side==0){
            if(parking_fixed_maneuver&&parking_fixed_side&&board.stopped&&board.candidate){
                side=parking_fixed_side;
            }else if((board.stopped||parking_edge_entry)&&board.candidate&&lane.leftValid&&lane.rightValid){
                double y=board.box.br().y,l=lane.leftAt(y),r=lane.rightAt(y),x=board.box.x+board.box.width*.5;
                int p=x<(l+r)*.5-(r-l)*parking_side_deadband?1:x>(l+r)*.5+(r-l)*parking_side_deadband?-1:0;
                if(r-l<parking_min_lane_width)p=0;
                // 挡板自己的确认仍必须完成；侧别确认与其并行，避免再额外停等三帧。
                if(parking_edge_entry&&board.hits==1){sideHits=0;proposal=0;}
                sideHits=(p&&p==proposal)?sideHits+1:1;proposal=p;
                if(board.stopped&&p&&sideHits>=parking_side_frames)side=p;
            }else{sideHits=0;proposal=0;}
            // Keep moving while board-side recognition is unresolved.
            if(board.stopped&&!side){hold=false;state="WAIT_BOARD_SIDE";return;}
            if(!side){state="SEARCH_BOARD";return;}
        }
        if(parking_edge_follow){
            // 挡板消失后仍锁定同一侧，持续跟随边线，不回中。
            if(parking_edge_entry&&!entryStarted){
                if(!canStart){state="PARK_ENTRY_READY";return;}
                entry.start(entryPlan,now);entryStarted=true;
            }
            if(!updateEdge(lane,canStart?dt:0)){
                edgeReady=false;edgePdReady=false;qrValidSince=now;
                hold=true;state=side>0?"WAIT_RIGHT_EDGE":"WAIT_LEFT_EDGE";
                qrHits=0;previousQR={};return;
            }
            state=entryControl()?"PARK_SHIFT_"+entry.phase():side>0?"PARK_FOLLOW_RIGHT_EDGE":"PARK_FOLLOW_LEFT_EDGE";
        }else if(parking_fixed_maneuver){
            offset=0;
            if(!fixedStarted){
                if(!canStart||!board.candidate||boardGroundY<parking_maneuver_start_y){
                    state="PARK_APPROACH_BOARD";return;
                }
                fixed.start(fixedPlan,now);fixedStarted=true;
            }
            state=fixed.active()?"PARK_FIXED_"+fixed.phase():"PARK_FIXED_WAIT_QR";
            // 绕板动作不依赖车道边界。动作后保持固定前进，继续原有半区二维码确认。
            // 无真实边界时不认证二维码；动作进行中不因此中断，结束后沿用原等待规则。
            if(!lane.leftValid||!lane.rightValid){
                qrHits=0;previousQR={};
                if(!fixed.active()){hold=true;state="WAIT_REAL_BOUNDARIES";}
                return;
            }
        }else{
            if(!lane.leftValid||!lane.rightValid){hold=true;state="WAIT_REAL_BOUNDARIES";return;}
            double target=side*parking_target_offset;offset+=std::max(-parking_offset_rate*dt,std::min(parking_offset_rate*dt,target-offset));
            state=side>0?"PARK_RIGHT_WAIT_QR":"PARK_LEFT_WAIT_QR";
        }
        // 实车室内由后台检测器提供结果，控制循环不执行耗时QR搜索。
        if(!deferQr)applyQR(observeQR(frame,lane,now),now);
    }
    ParkingQrObservation observeQR(const cv::Mat& frame,const LaneV2& lane,double now){
        cv::Mat points;
        bool found=detectQR(frame,points);cv::Rect box;
        if(found&&points.total()==4){
            const auto* p=points.ptr<cv::Point2f>();std::vector<cv::Point2f> v(p,p+4);box=cv::boundingRect(v);
            // QR is mounted ABOVE the ground on the opposite board. Comparing
            // it with ground-lane bounds at QR height wrongly rejects it near
            // the horizon. Use the observed board FOOT as the common depth row.
            double y=std::max(box.br().y,boardGroundY),l=lane.leftAt(y),r=lane.rightAt(y),x=box.x+box.width*.5;
            // 另一边线缺失时，使用当前选定边线和最近真实双线的宽度估计二维码半区。
            bool boundsOK=!parking_edge_follow||edgeBounds(lane,y,l,r);
            found=boundsOK&&box.width>=qr_min_size&&box.height>=qr_min_size&&box.y>qr_min_y&&x>l&&x<r&&(side>0?x>(l+r)*.5:x<(l+r)*.5);
        }else found=false;
        ParkingQrObservation seen;seen.ready=true;seen.found=found;seen.side=side;seen.capturedAt=now;seen.box=box;return seen;
    }
    void applyQR(const ParkingQrObservation& seen,double now){
        if(!seen.ready||hold||stopped||!side)return;
        // 每个后台结果只消费一次；过期/丢线前/其他侧别结果不能累计终停确认。
        if(deferQr&&(seen.side!=side||seen.capturedAt<qrValidSince||seen.capturedAt<=lastQrAt||now-seen.capturedAt>2.)){
            qrHits=0;previousQR={};return;
        }
        if(deferQr&&lastQrAt>=0&&seen.capturedAt-lastQrAt>2.){qrHits=0;previousQR={};}
        lastQrAt=seen.capturedAt;bool found=seen.found;cv::Rect box=seen.box;
        // 本帧必须通过尺寸、行号和半区检查，再与上一框匹配；任一帧无效就重置确认计数。
        // 这里的重叠率也按两框外接矩形面积作分母，保持现有实现。
        if(found){double overlap=previousQR.area()?double((box&previousQR).area())/std::max(1,(box|previousQR).area()):0;qrHits=overlap>qr_match_iou?qrHits+1:1;previousQR=box;}
        else{qrHits=0;previousQR={};}
        // 连续确认达标后仅锁存停车状态；Suite 随后把电机设为中位，任务不会自动重新开始。
        if(qrHits>=qr_confirm_frames){stopped=true;state="QR_STOP_LATCHED";}
    }
};
