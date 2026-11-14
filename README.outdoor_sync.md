# 室外同步室内行为版本（2026-11-15）

基准是本次连接树莓派时用户已经调好的 `config/indoor.yaml`。室外 V2 和原版室外都使用这套动作、速度、转向及任务切换参数，各自仍使用原有的室外巡线图像算法。室内配置原样保留。

## 入口和编译

```bash
cd /home/pi/yidui/jgy/race_suite_outdoor_v2
sh build_pi.sh
sh run_outdoor_v2.sh --modules all --check-config
```

常用室外入口 `run_outdoor_v2.sh` 明确读取本目录的 `config/outdoor_v2.yaml`，日志应显示 `env=outdoor_v2 lane=outdoor_v2`。原版室外入口 `run_outdoor.sh` 读取 `config/outdoor.yaml`，日志显示 `env=outdoor lane=outdoor`。

```bash
# 以下命令会让车行驶，按本次调试需要只执行其中一条。
sudo sh run_outdoor_v2.sh --modules lane --enable-motion
sudo sh run_outdoor_v2.sh --modules lane,cone --enable-motion
sudo sh run_outdoor_v2.sh --modules lane,parking --enable-motion
sudo sh run_outdoor_v2.sh --modules all --enable-motion
```

驾驶默认没有预览，误加 `--show` / `--masks` 也会过滤。只做图像检测并且有图形桌面时，可不加 `--enable-motion`，显式使用 `--masks`。修改 YAML 后重启即可；修改 C++ 后需要重新编译。

## 与当前室内相同的行为

- 普通巡线保留启动时约一秒的 `WAIT`。启动之后，识别挡板、确认侧别期间继续前进，`WAIT_BOARD_SIDE` 不发中位停车命令。
- 锥桶只识别第一个，按它所在的一侧选择反侧，执行完整九段 S 形。动作完成后恢复巡线，本轮不再识别第二个桶；同时启用停车模块时，整套 S 完成才解锁停车。
- 挡板偏左，向右侧移；挡板偏右，向左侧移。执行 `OUT → ALIGN → SETTLE`，再持续沿选定边线，二维码连续确认后锁存停车。
- 停车缺线继续采用当前室内的宽度缓存、单侧和画面边缘补位逻辑。选侧前仍要求能够判断挡板所在半区；判断未完成时继续巡线。
- 实时停车二维码在后台识别，离线图片／回放仍同步识别。红灯、斑马线等待及二维码终停的优先级保持室内规则。

## 当前直接复制的控制值

| 参数 | 本次室内基准值 |
|---|---|
| `speed.neutral_pwm` | `10000` |
| `speed.start_pwm / corner_pwm / after_corner_pwm` | `10900 / 10900 / 10900` |
| `speed.corner_brake_s` | `0` |
| `lane_pd_kp / lane_pd_kd` | `0.2 / 0.5` |
| `servo_center / servo_min / servo_max` | `80 / 55 / 86` |
| `cone_confirm_frames / cone_avoid_start_y` | `2 / 300` |
| `cone_slow_pwm` | `10700` |
| `parking_side_frames` | `3` |
| `parking_follow_pwm` | `10900` |
| `parking_edge_margin_fraction` | `0.25` |
| `parking_pd_kp / parking_pd_kd` | `0.18 / 0.08` |
| `parking_steer_limit / parking_steer_rate` | `3 / 3` |
| `parking_heading_kp` | `0.8` |
| `zebra_stop_y / zebra_brake_pwm` | `0.65 / 8000` |

整套 `cone_s_steps` 原样复制：FIRST_OUT `+5/0.65s`，FIRST_ALIGN `-5/0.65s`，FIRST_PASS `0/0.45s`，CROSS `-5/0.95s`，SECOND_ALIGN `+5/0.95s`，SECOND_PASS `0/0.45s`，RETURN `+5/0.65s`，FINAL_ALIGN `-5/0.65s`，SETTLE `0/0.20s`；各段电机 PWM 都为 `10900`。

停车三段为：

```yaml
parking_edge_entry_steps:
  - { name: OUT,    steer:  5.0, seconds: 0.60, motor_pwm: 10900 }
  - { name: ALIGN,  steer: -4.0, seconds: 0.60, motor_pwm: 10900 }
  - { name: SETTLE, steer:  0.0, seconds: 2.00, motor_pwm: 10900 }
```

`steer` 是 PWM 修正，不是角度。向右侧移时目标为 `80-steer`，向左侧移时镜像，仍受 `55..86` 的机械限位。正数向空闲侧打，负数反打，零为舵机回中。`seconds` 是该段前进命令的持续秒数。后续调转出、回正与直行的幅度／时长，就调整这三个动作的 `steer` 和 `seconds`；贴边距离调整 `parking_edge_margin_fraction`；贴边前进速度调整 `parking_follow_pwm`。

## 保留的室外识别标定

`src/lane_original.hpp`、`src/lane_outdoor_v2.hpp` 和各识别器的代码保持原样。两个室外 YAML 保留各自的后端、曝光、颜色阈值、ROI、形状与拟合参数。确认帧数、触发位置和制动等任务控制值使用室内基准。

相对室内，已有公共字段中仍不同的识别值是：`lane_backend`、`set_auto_exposure: 3`、`set_exposure: 1500`、`board_s_min: 140`、`board_min_aspect: 0.65`。自动曝光模式下 `set_exposure` 不决定实时曝光；室外 V2 额外保留全部 `outdoor_*` 检测字段。

自动相机回位仍关闭（`camera_home_enabled: 0`），使用当前手调相机姿态。GPIO 输出函数、频率、量程及退出处理保持本次读取到的室内基准代码。`motor_pwm` 是绝对 PWM 数值，动作不逐帧累加它。

## 备份与验证

改动前的源码、配置、脚本、测试、文档及原可执行程序已存入树莓派：

`/home/pi/yidui/jgy/race_suite_outdoor_v2/backups/outdoor_sync_indoor_20261115_01/before.tar.gz`

SHA-256：`f12dfbb28c59998549a5123f66ec4051c462124b17f8d444888055bedef8f0f8`。备份归档不上传 GitHub。

本次离线检查入口：

```bash
sh tests/run_outdoor_sync.sh
```

它编译并运行纯图像／状态机测试，不创建硬件输出对象，不打开摄像头。覆盖三后端停车动作、左右镜像、识别时继续前进、缺线补位、二维码终停、室内及室外 V2 的真实合成图检测与一次性 S 路线。合成测试和编译不代表室外实车轨迹已经验收；室外光照和地面变化仍需现场确认。

其余历史测试及文档保留用于追溯，部分仍针对旧的逐桶／缺线停车方案，不能把它们的旧断言作为当前动作要求。当前行为及参数以本说明和 YAML 为准。
