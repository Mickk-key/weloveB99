#pragma once
#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>
// 当前只支持原室外形状检测＋视觉绕桶。下列字段都由检测或视觉控制实际使用。
// 像素坐标为640×480，fraction是图像宽度比例；保留原默认值及有效范围。
struct ConeConfig {
    int cone_lane_count=2;
    int cone_shape_min_height=18,cone_shape_min_width=8;
    double cone_shape_top_ratio=.65,cone_shape_aspect_min=1.05;
    double cone_handoff_y=420.;
    int cone_roi_top=280;
    int cone_min_height=4,cone_max_width=180,cone_confirm_frames=3;
    double cone_avoid_start_y=340,cone_full_avoid_y=420,cone_pass_y=445;
    double cone_clearance_fraction=0.18,cone_min_gap_fraction=0.025;
    double cone_lost_grace_s=0.30,cone_approach_pixels=35,cone_approach_seconds=0.20;
    double cone_close_depth_px=70,cone_smooth_seconds=0.10,cone_offset_rate=1.0;
    double cone_start_blend=0.4,cone_confirm_seconds=0.,cone_lane_extrapolation_px=320.;
    void load(const cv::FileStorage& fs){
        // 旧模式名称仅用于拒绝不兼容的外部配置，不再保存开关或提供旧实现。
        for(const char* name:{"cone_fixed_maneuver","cone_s_course_once","cone_direct_turn"})
            if(!fs[name].empty()&&int(fs[name])!=0)throw std::invalid_argument("Unsupported legacy cone mode");
        for(const char* name:{"cone_lane_mode","cone_shape_mode"})
            if(!fs[name].empty()&&int(fs[name])!=1)throw std::invalid_argument("Only visual shape cone mode is supported");
#define CONE_READ(k) if(!fs[#k].empty())fs[#k]>>k
        CONE_READ(cone_lane_count);
        CONE_READ(cone_shape_min_height);
        CONE_READ(cone_shape_min_width);
        CONE_READ(cone_shape_top_ratio);
        CONE_READ(cone_shape_aspect_min);
        CONE_READ(cone_handoff_y);
        CONE_READ(cone_roi_top);
        CONE_READ(cone_min_height);
        CONE_READ(cone_max_width);
        CONE_READ(cone_confirm_frames);
        CONE_READ(cone_avoid_start_y);
        CONE_READ(cone_full_avoid_y);
        CONE_READ(cone_pass_y);
        CONE_READ(cone_clearance_fraction);
        CONE_READ(cone_min_gap_fraction);
        CONE_READ(cone_lost_grace_s);
        CONE_READ(cone_approach_pixels);
        CONE_READ(cone_approach_seconds);
        CONE_READ(cone_close_depth_px);
        CONE_READ(cone_smooth_seconds);
        CONE_READ(cone_offset_rate);
        CONE_READ(cone_start_blend);
        CONE_READ(cone_confirm_seconds);
        CONE_READ(cone_lane_extrapolation_px);

#undef CONE_READ
        validate();
    }
    void validate()const{
        if(cone_shape_min_height<8||cone_shape_min_height>100||cone_shape_min_width<4||cone_shape_min_width>100||
           !std::isfinite(cone_shape_top_ratio)||cone_shape_top_ratio<.3||cone_shape_top_ratio>.8||
           !std::isfinite(cone_shape_aspect_min)||cone_shape_aspect_min<.65||cone_shape_aspect_min>2.)
            throw std::invalid_argument("Invalid cone shape settings");
        if(cone_lane_count<1||cone_lane_count>8||!std::isfinite(cone_handoff_y)||
           cone_handoff_y<=cone_avoid_start_y||cone_handoff_y>cone_pass_y)
            throw std::invalid_argument("Invalid visual cone count/handoff");
        const double vals[]={cone_avoid_start_y,cone_full_avoid_y,cone_pass_y,cone_clearance_fraction,
            cone_min_gap_fraction,cone_lost_grace_s,cone_approach_pixels,cone_approach_seconds,
            cone_close_depth_px,cone_smooth_seconds,cone_offset_rate,cone_start_blend,cone_confirm_seconds,cone_lane_extrapolation_px};
        for(double v:vals)if(!std::isfinite(v))throw std::invalid_argument("Non-finite cone configuration");
        if(cone_lane_extrapolation_px<0||cone_lane_extrapolation_px>640||cone_start_blend<0||cone_start_blend>1||
           cone_confirm_seconds<0||cone_confirm_seconds>2||cone_roi_top<0||cone_roi_top>=480||
           cone_min_height<2||cone_max_width<cone_min_height||cone_max_width>640||cone_confirm_frames<2||cone_confirm_frames>60||
           cone_avoid_start_y<=cone_roi_top||cone_full_avoid_y<=cone_avoid_start_y||cone_pass_y<cone_full_avoid_y||cone_pass_y>480||
           cone_clearance_fraction<=0||cone_clearance_fraction>=.45||cone_min_gap_fraction<=0||cone_min_gap_fraction>=.3||
           cone_lost_grace_s<=0||cone_lost_grace_s>2||cone_approach_pixels<=0||cone_approach_seconds<=0||
           cone_close_depth_px<0||cone_close_depth_px>480||cone_smooth_seconds<=0||cone_offset_rate<=0||cone_offset_rate>5)
            throw std::invalid_argument("Invalid cone configuration");
    }
};
