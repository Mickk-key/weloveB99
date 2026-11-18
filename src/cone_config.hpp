#pragma once
#include "fixed_maneuver.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <stdexcept>
// Coordinates/thresholds refer to the runtime's 640x480 camera frame.
// Clearance is an IMAGE-SPACE tuning ratio, not measured physical clearance.
// *_y 为图像底边行阈值，*_s/seconds 为秒，*_fraction 为图像中车道宽度比例。
// clearance 是图像空间余量，不是实测厘米；turn_pwm 是直接转向时相对中心的修正量。
struct ConeConfig {
    int cone_lane_mode=0,cone_lane_count=2;
    int cone_shape_mode=0,cone_shape_min_height=18,cone_shape_min_width=8;
    double cone_shape_top_ratio=.65,cone_shape_aspect_min=1.05;
    double cone_handoff_y=420.;
    int cone_fixed_maneuver=0,cone_fixed_side=0,cone_s_course_once=0;
    double cone_rearm_seconds=.5;
    FixedPlan fixedPlan;
    int cone_direct_turn=0;
    double cone_turn_pwm=5.,cone_turn_seconds=.65;
    int cone_roi_top=280, cone_h_min=100, cone_h_max=130;
    int cone_s_min=40, cone_v_min=55, cone_min_pixels=10;
    int cone_min_height=4, cone_max_width=180, cone_confirm_frames=3;
    double cone_aspect_min=0.55, cone_aspect_max=4.5, cone_fill_min=0.22;
    double cone_avoid_start_y=340, cone_full_avoid_y=420, cone_pass_y=445;
    double cone_clearance_fraction=0.18, cone_min_gap_fraction=0.025;
    double cone_lost_grace_s=0.30, cone_approach_pixels=35, cone_approach_seconds=0.20;
    double cone_close_depth_px=70, cone_smooth_seconds=0.10, cone_offset_rate=1.0;
    // Zero preserves existing behaviour; outdoor profiles enable return-only limits.
    double cone_return_steer_limit=0,cone_return_steer_rate=0;

    double cone_start_blend=0.4, cone_confirm_seconds=0., cone_lane_extrapolation_px=320.;

    void load(const cv::FileStorage& fs) {
#define CONE_READ(k) if(!fs[#k].empty()) fs[#k] >> k
        CONE_READ(cone_shape_mode);CONE_READ(cone_shape_min_height);CONE_READ(cone_shape_min_width);CONE_READ(cone_shape_top_ratio);
        CONE_READ(cone_shape_aspect_min);
        CONE_READ(cone_lane_mode);CONE_READ(cone_lane_count);CONE_READ(cone_handoff_y);
        CONE_READ(cone_s_course_once);CONE_READ(cone_fixed_maneuver);CONE_READ(cone_fixed_side);CONE_READ(cone_rearm_seconds);
        if(cone_fixed_maneuver)fixedPlan.load(fs,cone_s_course_once?"cone_s_steps":"cone_maneuver_steps");
        CONE_READ(cone_direct_turn); CONE_READ(cone_turn_pwm); CONE_READ(cone_turn_seconds);
        CONE_READ(cone_roi_top); CONE_READ(cone_h_min); CONE_READ(cone_h_max);
        CONE_READ(cone_s_min); CONE_READ(cone_v_min); CONE_READ(cone_min_pixels);
        CONE_READ(cone_min_height); CONE_READ(cone_max_width); CONE_READ(cone_confirm_frames);
        CONE_READ(cone_aspect_min); CONE_READ(cone_aspect_max); CONE_READ(cone_fill_min);
        CONE_READ(cone_avoid_start_y); CONE_READ(cone_full_avoid_y); CONE_READ(cone_pass_y);
        CONE_READ(cone_clearance_fraction); CONE_READ(cone_min_gap_fraction);
        CONE_READ(cone_lost_grace_s); CONE_READ(cone_approach_pixels); CONE_READ(cone_approach_seconds);
        CONE_READ(cone_close_depth_px); CONE_READ(cone_smooth_seconds); CONE_READ(cone_offset_rate);
        CONE_READ(cone_return_steer_limit);CONE_READ(cone_return_steer_rate);
        CONE_READ(cone_lane_extrapolation_px); CONE_READ(cone_start_blend); CONE_READ(cone_confirm_seconds);
#undef CONE_READ
        validate();
    }
    // 检查识别阈值、开始/充分绕行/通过的先后关系，以及时间、偏移和外推范围。
    void validate() const {
        if(!std::isfinite(cone_return_steer_limit)||cone_return_steer_limit<0||cone_return_steer_limit>15||
           !std::isfinite(cone_return_steer_rate)||cone_return_steer_rate<0||cone_return_steer_rate>200)
            throw std::invalid_argument("Invalid cone return steering limits");
        if((cone_shape_mode!=0&&cone_shape_mode!=1)||(cone_shape_mode&&!cone_lane_mode)||
           cone_shape_min_height<8||cone_shape_min_height>100||cone_shape_min_width<4||cone_shape_min_width>100||
           !std::isfinite(cone_shape_top_ratio)||cone_shape_top_ratio<.3||cone_shape_top_ratio>.8||
           !std::isfinite(cone_shape_aspect_min)||cone_shape_aspect_min<.65||cone_shape_aspect_min>2.)
            throw std::invalid_argument("Invalid cone shape settings");
        if((cone_lane_mode!=0&&cone_lane_mode!=1)||cone_lane_count<1||cone_lane_count>8||
           (cone_lane_mode&&(!std::isfinite(cone_handoff_y)||cone_handoff_y<=cone_avoid_start_y||cone_handoff_y>cone_pass_y))||
           (cone_lane_mode&&(cone_fixed_maneuver||cone_s_course_once||cone_direct_turn)))
            throw std::invalid_argument("Invalid visual cone mode/handoff or conflicting fixed modes");
        if((cone_s_course_once!=0&&cone_s_course_once!=1)||(cone_s_course_once&&!cone_fixed_maneuver))
            throw std::invalid_argument("cone_s_course_once requires cone_fixed_maneuver=1");
        if((cone_fixed_maneuver!=0&&cone_fixed_maneuver!=1)||cone_fixed_side< -1||cone_fixed_side>1||
           !std::isfinite(cone_rearm_seconds)||cone_rearm_seconds<0||cone_rearm_seconds>10)
            throw std::invalid_argument("Invalid fixed cone settings");
        if((cone_direct_turn!=0&&cone_direct_turn!=1)||!std::isfinite(cone_turn_pwm)||cone_turn_pwm<1||cone_turn_pwm>15||!std::isfinite(cone_turn_seconds)||cone_turn_seconds<.1||cone_turn_seconds>3)throw std::invalid_argument("Invalid direct cone turn settings");
        const double vals[]={cone_aspect_min,cone_aspect_max,cone_fill_min,
            cone_avoid_start_y,cone_full_avoid_y,cone_pass_y,cone_clearance_fraction,
            cone_min_gap_fraction,cone_lost_grace_s,cone_approach_pixels,cone_approach_seconds,
            cone_close_depth_px,cone_smooth_seconds,cone_offset_rate,cone_start_blend,cone_confirm_seconds,cone_lane_extrapolation_px};
        for(double v:vals) if(!std::isfinite(v)) throw std::invalid_argument("Non-finite cone configuration");
        if(cone_lane_extrapolation_px<0 || cone_lane_extrapolation_px>640 || cone_start_blend<0 || cone_start_blend>1 || cone_confirm_seconds<0 || cone_confirm_seconds>2 || cone_roi_top<0 || cone_roi_top>=480 || cone_h_min<0 || cone_h_max>179 || cone_h_min>cone_h_max ||
           cone_s_min<0 || cone_s_min>255 || cone_v_min<0 || cone_v_min>255 ||
           cone_min_pixels<3 || cone_min_height<2 || cone_max_width<cone_min_height || cone_max_width>640 ||
           cone_confirm_frames<2 || cone_confirm_frames>60 || cone_aspect_min<=0 || cone_aspect_max<=cone_aspect_min ||
           cone_fill_min<=0 || cone_fill_min>1 || cone_avoid_start_y<=cone_roi_top ||
           cone_full_avoid_y<=cone_avoid_start_y || cone_pass_y<cone_full_avoid_y || cone_pass_y>480 ||
           cone_clearance_fraction<=0 || cone_clearance_fraction>=0.45 ||
           cone_min_gap_fraction<=0 || cone_min_gap_fraction>=0.3 ||
           cone_lost_grace_s<=0 || cone_lost_grace_s>2 || cone_approach_pixels<=0 ||
           cone_approach_seconds<=0 || cone_close_depth_px<0 || cone_close_depth_px>480 ||
           cone_smooth_seconds<=0 || cone_offset_rate<=0 || cone_offset_rate>5)
            throw std::invalid_argument("Invalid cone configuration");
    }
};
