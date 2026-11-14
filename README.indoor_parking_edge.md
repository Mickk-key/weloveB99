# 室内停车：侧移、反打摆正、持续沿边

当前室内入口读取 `/home/pi/yidui/jgy/race_suite_outdoor_v2/config/indoor.yaml`。新流程用于解决确认挡板后停一下、侧移不明显，以及转过去后车头仍斜向边线的问题。

## 当前流程

1. 挡板识别与左右位置确认同时进行；挡板达到原有确认条件后直接接管。挡板偏左选右边，挡板偏右选左边。同一停车任务锁定这个方向。
2. `PARK_SHIFT_1_OUT`：向空闲侧固定转向，完成初步侧移。
3. `PARK_SHIFT_2_ALIGN`：反打固定时间，把车头摆回来。
4. `PARK_SHIFT_3_SETTLE`：舵机回正，短暂直行。
5. `PARK_FOLLOW_RIGHT_EDGE` 或 `PARK_FOLLOW_LEFT_EDGE`：持续跟随选定边线内侧的目标；同时看近端横向位置和远近方向差，车头斜向边线时提前反打。不会按时间回到车道中间。
6. 所选半区二维码连续确认后进入 `QR_STOP_LATCHED`，最终停车。

正常可判断左右时，不再先停下来额外确认三帧。如果挡板确实居中或双线不足以选侧，仍会 `WAIT_BOARD_SIDE`；选侧后所选边线丢失则 `WAIT_RIGHT_EDGE` / `WAIT_LEFT_EDGE`，暂停动作计时。另一条边线可以丢失，此时用最近真实双线宽度估计边线内侧目标。红灯、斑马线等待也暂停固定动作，终停优先。

## 先调固定动作

```yaml
parking_fixed_maneuver: 0  # 旧整套绕板功能关闭
parking_edge_follow: 1
parking_edge_entry: 1
parking_edge_entry_steps:
  - { name: OUT,    steer:  4.0, seconds: 0.60, motor_pwm: 10900 }
  - { name: ALIGN,  steer: -4.0, seconds: 0.60, motor_pwm: 10900 }
  - { name: SETTLE, steer:  0.0, seconds: 0.15, motor_pwm: 10900 }
```

`steer` 是舵机 PWM 修正量，不是角度。正值向选定空闲侧，负值向相反侧。中心80时，向右侧移的序列为76、84、80；向左侧移镜像为84、76、80。均受原有55..86机械范围限制。

`seconds` 是对应前进指令持续秒数，不是实测路程。三段必须依次为正转向、负转向、回正；没有回中段。初值未经实车轨迹标定，不能保证0.60秒对应足够的避板间距。

| 表现 | 优先调整 |
|---|---|
| 转向不明显或侧移距离不够 | `OUT.steer` 从4增到5，或者 `OUT.seconds` 从0.60增到0.70；一次只改一个 |
| 初始侧移太猛 | `OUT.steer` 从4减到3，或缩短 `OUT.seconds` |
| 侧移后车头仍斜向所选边线 | 增加 `ALIGN.seconds`，例如0.60到0.70 |
| 摆正过头、车头斜回车道中间 | 减少 `ALIGN.seconds`，例如0.60到0.50 |
| 动作途中电机带不动 | 改这三段各自的 `motor_pwm` |

左右实际转角可能不对称，不能只凭同样绝对值认定车头已经摆正。先把OUT/ALIGN调到车头大致顺着车道，再调持续沿边参数。

## 再调持续沿边

```yaml
parking_follow_pwm: 10900
parking_edge_margin_fraction: 0.25
parking_pd_kp: 0.18
parking_pd_kd: 0.08
parking_heading_kp: 0.8
parking_steer_limit: 3.0
parking_steer_rate: 3.0
parking_error_smooth_seconds: 0.20
parking_qr_async: 1
```

本次保留用户已调的留边0.25、变化速率3.0和油门10900。`parking_follow_pwm` 控制固定三段结束后的沿边速度，不再被 `cone_slow_pwm` 压低；三段期间使用各段 `motor_pwm`。

`parking_edge_margin_fraction` 是车中心与边线的间距占车道宽度比例，0.25表示内侧留25%；越大越向里，越小越贴边。不是厘米或车身外沿间距。离边线太近可由0.25加到0.28。

`parking_heading_kp` 控制远近方向差的修正力度；车头斜向边线却反打不够可小步增大，左右摆动可小步减小。`parking_steer_limit` 只限制持续沿边阶段的转向幅度，3表示77..83；`parking_steer_rate` 只限制沿边阶段每秒PWM变化。它们不再限制固定OUT/ALIGN，所以不要用它们来调第一下侧移。`parking_shift_rate` 只用于旧 `parking_edge_entry: 0` 渐进模式，当前不生效。

## 二维码与日志

实时室内停车使用独立线程检测二维码，控制循环不再等待全图、放大图和裁剪重试。待处理队列只保留最新帧；每个结果只消费一次，使用该帧自己的边线几何验证半区。超过2秒、来自丢线恢复之前、侧别错误的结果不累计终停。离线图片/回放维持同步以保证可复现。原有小二维码样例的识别限制没有在此轮修复。

终端不是逐帧打印，而是每10帧或状态切换打印一次，不能把两条终端日志间隔直接当成帧周期。新增 `frame_dt_s` 表示真实相邻控制帧间隔，`processing_ms` 表示当前帧算法耗时；`parking_side` 为1向右、-1向左，`edge_heading` 为方向差，`qr_hits` 为终停确认次数，`qr_age_s` 为最近处理二维码结果的图像年龄。`fixed_remaining_s` 显示当前固定段剩余时间。

## 命令与验证

```bash
cd /home/pi/yidui/jgy/race_suite_outdoor_v2
sh run_indoor.sh --modules lane,parking --check-config
sudo sh run_indoor.sh --modules lane,parking --enable-motion
# 全流程，S形完成后解锁停车
sudo sh run_indoor.sh --modules all --enable-motion
```

驾驶默认无预览。保存YAML并退出重启即生效，不用重新编译。主逻辑在 `src/parking_qr.hpp`、仲裁在 `src/suite.hpp`，后台QR在 `src/async_parking_qr.hpp`。

`sh tests/run_parking_entry.sh` 是无摄像头、无GPIO验证：左右选侧/侧移/反打/沿边、停车优先级、旧沿边控制、二维码、室内S路线及室外固定动作。程序验证不能代替实车轨迹确认；本轮未自动驾驶，也未改相机姿态。
