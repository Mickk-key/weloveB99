#pragma once
#include "blue_board.hpp"

// 比赛顺序控制，不实现新的视觉算法，也不输出 GPIO。
// 完整模块组合启用此控制器；单项/部分模块测试保持原来的独立运行方式。
class RaceSequence {
public:
    enum Stage { INDEPENDENT, WAIT_START_BOARD, WAIT_START_CLEAR,
                 ZEBRA, LIGHT, CONE_CRUISE, CONE, PARKING_CRUISE, PARKING, FINISHED };
private:
    Stage stage_;
    BoardConfig cfg;
    BlueBoardStop startBoard; // 与终点停车的 BlueBoardStop 完全独立，不能复用锁存。
    int clearHits_=0;
    double clearSince=-1,lastClear=-1,launchedAt=-1;
    double coneCruiseElapsed=0,coneCruiseLast=-1;
    double parkingCruiseElapsed=0,parkingCruiseLast=-1;
public:
    RaceSequence(bool enabled,const BoardConfig& c):
        stage_(enabled?WAIT_START_BOARD:INDEPENDENT),cfg(c),startBoard(c){}
    bool enabled()const{return stage_!=INDEPENDENT;}
    bool waiting()const{return stage_==WAIT_START_BOARD||stage_==WAIT_START_CLEAR;}
    bool boardConfirmed()const{return startBoard.stopped;}
    int clearHits()const{return clearHits_;}
    Stage stage()const{return stage_;}
    const char* name()const{
        switch(stage_){
        case WAIT_START_BOARD:return "WAIT_START_BOARD";
        case WAIT_START_CLEAR:return "WAIT_START_CLEAR";
        case ZEBRA:return "ZEBRA";
        case LIGHT:return "LIGHT";
        case CONE_CRUISE:return "CONE_CRUISE";
        case CONE:return "CONE";
        case PARKING_CRUISE:return "PARKING_CRUISE";
        case PARKING:return "PARKING";
        case FINISHED:return "FINISHED";
        default:return "INDEPENDENT";
        }
    }
    void updateStart(const cv::Mat& frame,double now){
        if(!waiting())return;
        // 原检测器先确认出现。始终没看到挡板时，绝不能把空画面当作发车信号。
        startBoard.update(frame,now);
        if(!startBoard.stopped)return;
        stage_=WAIT_START_CLEAR;
        if(startBoard.candidate){clearHits_=0;clearSince=lastClear=-1;return;}
        // stopped 是原检测器的永久锁存；移开判断必须读取每帧 candidate。
        // 消失确认沿用 board_confirm_frames/seconds；帧间断档按原检测器的
        // 0.25 秒连续性规则重计，单帧漏检或长时间无图不会直接发车。
        if(clearHits_==0||now-lastClear>.25){clearHits_=1;clearSince=now;}
        else ++clearHits_;
        lastClear=now;
        if(clearHits_>=cfg.board_confirm_frames&&now-clearSince>=cfg.board_confirm_seconds&&now>=1.){
            launchedAt=now;stage_=ZEBRA;
        }
    }
    // 等挡板的时间不消耗巡线起步档或任何比赛动作计时。
    // 原程序 now<1 是启动中位保护，已经在发车门控中满足；移开确认后从 1 开始，
    // 不再多等一次启动秒数。所有后续模块仍使用同一单调时钟。
    double runTime(double wallTime)const{return launchedAt<0?wallTime:1.+wallTime-launchedAt;}
    bool zebraAllowed()const{return !enabled()||stage_==ZEBRA;}
    bool lightAllowed()const{return !enabled()||stage_==LIGHT;}
    bool coneAllowed()const{return !enabled()||stage_==CONE;}
    bool parkingAllowed()const{return !enabled()||stage_==PARKING||stage_==FINISHED;}
    void finishZebra(bool done){if(stage_==ZEBRA&&done)stage_=LIGHT;}
    void finishLight(bool done,double now){
        if(stage_==LIGHT&&done){stage_=CONE_CRUISE;coneCruiseElapsed=0;coneCruiseLast=now;}
    }
    // 只在完整比赛灯阶段放行后累计前进时间；此阶段coneAllowed仍为false。
    // forward来自上一帧最终电机命令，停车时间不计入；保持原巡线转向。
    void updateConeCruise(double now,double seconds,bool forward){
        if(stage_!=CONE_CRUISE)return;
        const double dt=std::max(0.,now-coneCruiseLast);coneCruiseLast=now;
        if(forward)coneCruiseElapsed+=dt;
        if(coneCruiseElapsed+1e-9>=seconds)stage_=CONE;
    }
    double coneCruiseRemaining(double seconds)const{return stage_==CONE_CRUISE?std::max(0.,seconds-coneCruiseElapsed):0.;}
    void finishCones(bool done,double now){
        if(stage_==CONE&&done){stage_=PARKING_CRUISE;parkingCruiseElapsed=0;parkingCruiseLast=now;}
    }
    void updateParkingCruise(double now,double seconds,bool forward){
        if(stage_!=PARKING_CRUISE)return;
        const double dt=std::max(0.,now-parkingCruiseLast);parkingCruiseLast=now;
        if(forward)parkingCruiseElapsed+=dt;
        if(parkingCruiseElapsed+1e-9>=seconds)stage_=PARKING;
    }
    double parkingCruiseRemaining(double seconds)const{return stage_==PARKING_CRUISE?std::max(0.,seconds-parkingCruiseElapsed):0.;}
    void finishParking(bool done){if(stage_==PARKING&&done)stage_=FINISHED;}
    void drawStart(cv::Mat& view)const{
        startBoard.draw(view);
        cv::putText(view,std::string(name())+" clear="+std::to_string(clearHits_),
                    {5,125},cv::FONT_HERSHEY_SIMPLEX,.55,{0,255,255},2);
    }
};
