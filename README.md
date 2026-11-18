# race

树莓派比赛程序：巡线、绕桶、斑马线、红绿灯及停车。仓库只保留源码、运行配置、编译/启动脚本和本说明。

## 编译与运行

依赖 C++14、OpenCV 4、pigpio、pthread。在树莓派执行：

```bash
cd /home/pi/yidui/jgy/race
bash build_pi.sh

# 全流程：使用默认红绿灯识别、原 parking 停车
sudo bash run.sh --env-outdoor --modules all --enable-motion

# 全流程：V2 夜间红绿灯识别、parking3 停车
sudo bash run.sh --env-outdoor --modules lane,cone,zebra,light,parking3 --light-night --enable-motion
```

室内将 `--env-outdoor` 换成 `--env-indoor`，每次只选一个环境。默认只巡线，附加功能必须通过 `--modules` 指定；不带 `--enable-motion` 时只计算结果，不输出 GPIO。

只看红绿灯识别，不等斑马线：

```bash
bash run.sh --env-outdoor --modules lane,light --show
# 夜间预览加 --light-night
```

`--show` 需要图形桌面或 X11 转发。不带时可看终端 JSON 日志。`--out runs/名称` 设置日志前缀，默认 `runs/latest`；`--run-seconds N` 设置运行时限，Ctrl+C 退出。

## 模块选择

| 模块 | 功能 |
|---|---|
| lane | 基础巡线，始终启用 |
| cone | 绕桶 |
| zebra | 斑马线制动、等待，然后继续巡线 |
| light | 红绿灯，默认用提供的原版检测文件；加 `--light-night` 用 V2 |
| parking | 原停车流程，使用环境 YAML 的 `parking_mode` |
| parking2 | 蓝板、黄色横线选侧后，按 parking2 路径行驶并检测白矩形停车 |
| parking3 | 选侧后沿该侧外侧白线行驶，检测白矩形停车 |
| all | lane,cone,zebra,light,parking |

`parking`、`parking2`、`parking3` 只能选一个。单独试停车可用 `--modules lane,parking3`。启用 cone 时，停车模块还受 `parking_after_cones` 解锁条件控制。

## 红绿灯

默认检测器 `src/traffic_light_detector.h` 保持原文件和默认参数，包含原有亮度判断；不追加目标过滤或确认。`--light-night` 使用 V2 的颜色/亮芯算法，不要求找到灯罩，参数在 `config/light_night_v2.yaml`。两种模式共用以下控制流程：

1. 启用 zebra 时，等斑马线制动、等待完成，再正常巡线 3 秒。
2. 前进 PWM 限制到 10700，降速保持 0.3 秒后开始识别，默认最多 10 Hz，巡线仍每帧运行。
3. 检测器确认红灯立即停车，不等 y。此前正在前进时，以 8000 制动 0.8 秒，再保持中位 10000；重复红灯不延长制动。
4. 停车后连续确认绿灯 0.5 秒且制动结束才放行。丢灯或未知保持停车，不定时自动放行。放行后本轮红绿灯任务完成；初始为绿灯时继续识别，之后变红仍可停。

3 秒和降速时间按最终前进命令累计，停车不计时。没有启用 zebra 时，启动等待 1 秒后直接降速、识别，不额外等 3 秒。`--light-night` 只切换检测器，不自动启用 light 或 zebra。没有灯罩定时停车模式。

## 停车

- **parking**：使用环境配置的原停车流程。当前室外 yellow 模式先执行 OUT / ALIGN / SETTLE 入位动作，再沿黄白路径行驶并识别白矩形停车；室内按自己的 YAML 执行。
- **parking2**：蓝板确认并通过黄色横线选侧，进入自己的路径控制，检测稳定白矩形后保持中位。
- **parking3**：蓝板确认 → 黄色横线选侧 → 沿所选侧外侧白线行驶 → 白矩形确认停车。丢线先短时保持，再使用该侧画面边缘补位。停车后白板消失也不重新启动。

parking3 的蓝板门槛使用共享 `board_*`，`board_pixel_mode=1` 时 `board_min_pixels` 是单个连通蓝区真实像素面积。`parking3_board_full_width=1` 只扩大水平 ROI，不降低面积门槛。黄色横线选侧使用 `parking3_yellow_*`。

parking2/parking3 的白矩形识别支持亮白和暗光两条路径；暗光路径还检查深色边框。两条路径都受面积、位置、填充率、确认帧数和持续时间限制。终点停车优先于红绿灯；按 `parking_photo_delay_seconds` 延时拍照，不自动退出。

## 调参

室内用 `config/indoor.yaml`，室外用 `config/outdoor.yaml`；修改后重启。各字段在 YAML 中有注释，像素参数基于 640×480 图像。

| 参数 | 用途 |
|---|---|
| speed、servo_center、lane_pd_kp/kd | 巡线油门、转向中位、PD |
| cone_* | 绕桶识别和动作 |
| zebra_stop_y、zebra_brake_*、zebra_hold_seconds | 斑马线停车位置、制动和等待 |
| light_after_zebra_seconds | 斑马线结束后正常巡线时间，当前 3 秒 |
| light_slow_pwm、light_slow_settle_seconds | 红绿灯降速上限与时间，当前 10700 / 0.3 秒 |
| light_detect_interval_ms | 红绿灯识别间隔，当前 100 毫秒 |
| light_brake_pwm、light_brake_seconds | 红灯制动，当前 8000 / 0.8 秒 |
| light_green_seconds | 停车后绿灯连续确认时间，当前 0.5 秒 |
| board_min_pixels、board_confirm_* | 共享蓝板面积与确认条件；面积增大通常更晚触发 |
| parking3_distance_near_px / far_px | 跟随白线时向车道内偏移的像素数，减小更靠外侧白线 |
| parking3_error_bias | 路径水平偏置，正值向画面右侧移动 |
| parking2_follow_pwm、parking3_follow_pwm | 对应停车路径的前进油门 |
| parking2_stop_*、parking3_stop_* | 白矩形停车条件；min_area / min_bottom 增大通常更晚停车 |
| parking_photo_delay_seconds | 终点停车后拍照延时 |

电机沿用直接 PWM：GPIO13、200 Hz、range 40000；`motor_pwm` 是控制命令，不是实际测速。环境参数可能不同，以对应 YAML 为准。

配置检查会在打开摄像头和硬件前返回：

```bash
bash run.sh --env-outdoor --modules all --check-config
bash run.sh --env-indoor --modules lane,zebra,light,parking3 --light-night --check-config
```
