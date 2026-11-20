# outdoor 独立比赛程序

本文件夹可完整复制到另一台已安装 OpenCV 4、pigpio、g++ 的树莓派上，独立编译运行；源码和配置只引用本目录。树莓派二进制不能直接在Windows电脑运行。

本环境使用**室外原版灰度边缘巡线**。`--modules all` 默认不识别红绿灯：人行道阶段结束后，巡线10秒，执行一次制动，再停车10秒，随后按 `cone_after_light_seconds` 巡线（当前5秒），最后开始识别锥桶。前后两段前进都使用原巡线PD修正转向。本次只更新室外流程。

## 编译与运行

```bash
cd /home/pi/yidui/smq/outdoor
bash build_pi.sh
bash run.sh --modules all --check-config
sudo bash run.sh --modules all --enable-motion
```

`all` 默认停车为 **parking4**，来自 `jgy/race`。`all` 默认的定时灯阶段不运行任何红绿灯检测，`--light-night` 在此模式下被忽略。单独调试红绿灯可使用：

```bash
sudo bash run.sh --modules lane,light --light-night --enable-motion
```

只巡线用 `--modules lane`（也是未传modules时的默认值）。保留 `lane,light,cone,zebra,parking3,parking4` 中现用模块组合；只有包含全部比赛模块时启用挡板发车及严格顺序。部分模块调试没有完整比赛的发车门控。`--image`、`--replay`、`--show`、`--masks` 和相机诊断仍保留。图片/回放不允许开启运动。

## 完整比赛顺序

1. `WAIT_START_BOARD`：保持中位，先确认蓝挡板确实出现。
2. `WAIT_START_CLEAR`：等待挡板连续消失确认；单帧漏检不发车。等待时间不消耗巡线起步计时。
3. `ZEBRA`：巡线并检测斑马线；按原配置制动、等待，然后放行，本轮只处理一次。
4. `LIGHT`：斑马线原有制动与等待完成后，进入 `LIGHT_FIXED_CRUISE`，按 `all_light_cruise_pwm`（11000）前进，保留原巡线PD和转向限速，持续 `all_light_cruise_seconds`（10秒）。之后进入 `LIGHT_FIXED_BRAKE`，按原 `light_brake_pwm`（7000）制动 `light_brake_seconds`（0.8秒），再进入 `LIGHT_FIXED_HOLD`，按电机中位10000停满 `all_light_stop_seconds`（10秒）。巡线只累计最终前进命令的时间，制动不会被重复帧延长；10秒停车不包含制动。只有制动和停车时舵机居中。
5. `CONE_CRUISE`：灯阶段停车结束后，恢复原巡线和分段油门，巡线 `cone_after_light_seconds`（当前5秒）后才启用锥桶检测。仅累计前进命令的时间，停车不计入；整个前置流程不检测、跟踪或累计锥桶，也不提前解锁停车模块。该参数在定时和视觉灯模式下均生效，设为0可立即开始识别锥桶。
6. `CONE`：原室外形状候选、跟踪和视觉临时边界绕桶；确认两个桶通过并回中后，进入 `PARKING_CRUISE`，按 `parking_after_cones_seconds`（默认5秒）巡线，期间不运行停车检测；计时完成后进入停车。
7. `PARKING`：默认parking4使用jgy/race的选侧与停车实现：蓝板靠左选右、靠右选左，沿选定白边行驶；白板或黄色T形确认后，一次制动并中位锁停。保留PARK3状态名兼容拍照；parking3_*控制跟边/白板/制动，parking4_*控制选侧和黄色T形检测。
8. `FINISHED`：制动结束后保持停车；等待原配置的拍照延迟后保存照片。发车挡板状态不会复用为终点挡板。

## 默认定时参数

在 `config/outdoor.yaml` 修改，保存后重新启动程序生效：

```yaml
all_light_fixed_enabled: 1        # 1：all默认定时，0：恢复原视觉灯流程
all_light_cruise_seconds: 10.0    # 人行道阶段结束后、定时停车前的巡线时间
all_light_cruise_pwm: 11000       # 该阶段前进PWM，转向使用巡线PD
all_light_stop_seconds: 10.0      # 制动结束后中位等待时间
cone_after_light_seconds: 5.0     # 灯阶段结束后，再巡线多久才识别锥桶
parking_after_cones_seconds: 5.0  # 绕桶并回中后，再巡线多久才启用停车识别
servo_center: 80                 # 制动、等待时中位；前进时由巡线PD修正
light_brake_pwm: 7000
light_brake_seconds: 0.8
# 停车中位在speed映射内：speed.neutral_pwm: 10000
```

上述巡线与停车时长范围均为0～60秒，前进PWM必须大于电机中位且不超过12000。没有新增字段时，默认灯前巡线10秒、停车10秒、灯后巡线5秒。兼容上一版 `all_light_straight_seconds/pwm` 名称，新 `all_light_cruise_seconds/pwm` 优先；无论使用哪个名称，前进时均巡线。完整显式模块组合与 `all` 行为相同；未传modules时仍是只巡线。

启动检查输出应包含 `light_mode=fixed_timed light_detection=OFF cruise_seconds=10 cruise_pwm=11000 cruise_steering=lane brake_pwm=7000 brake_seconds=0.8 stop_seconds=10 cone_delay_seconds=5`。运行日志中 `light_detection_calls` 始终为0，`light_sequence_remaining_s` 显示灯阶段动作剩余时间，`cone_cruise_remaining_s` 显示灯后巡线剩余时间；灯阶段结束原因是 `light_release_reason=fixed_timer`，不会伪造绿灯识别结果。

若设 `all_light_fixed_enabled: 0`，恢复原视觉灯流程：人行道后巡线 `light_after_zebra_seconds`（6秒），减速稳定后检测灯色；首次红灯制动后最多等 `light_red_timeout_seconds`（10秒，含制动），绿灯连续确认可提前放行；放行后同样巡线 `cone_after_light_seconds` 再识别锥桶。单独 `lane,light` 和 `lane,cone` 保持原有行为。

## 源码职责

| 文件 | 职责 |
|---|---|
| `main.cpp` | 参数、相机/图片/回放输入、主循环、输出和日志 |
| `suite.hpp` | 每帧调用模块并汇总最终电机/转向命令 |
| `race_sequence.hpp` | 挡板发车与阶段门控 |
| `fixed_light.hpp` | all默认的定时巡线、一次制动、完整中位等待 |
| `lane_backend.hpp`、`lane_original.hpp`、`lane_geometry.hpp` | 本环境巡线入口、原算法和通用边界表示 |
| `cone_v2.hpp`、`cone_shape.hpp`、`cone_lane_visual.hpp`、`cone_config.hpp` | 室外原形状识别、视觉绕桶及配置 |
| `traffic_light.hpp`、`traffic_light_detector.h`、`light_night_detector.hpp`、`light_sequence.hpp` | 默认/夜间灯色识别、制动放行和识别时序 |
| `zebra_original.hpp`、`blue_board.hpp` | 原斑马线、蓝挡板识别 |
| `parking3.hpp`、`parking_photo.hpp` | 唯一停车方案和终点照片 |
| `runtime.hpp`、`gpio_guard.hpp` | 原PD/分段油门、硬件输出、制动截止及退出中位 |
| `camera_controls.hpp`、`async_console.hpp` | 相机控制及异步诊断输出 |

```text
main：读取本目录配置与选项
  若只检查配置 → 返回（不打开相机、不初始化GPIO）
  打开输入，创建Suite和Output
  每帧：
    Suite.step：
      等发车时 → 只更新发车挡板，输出中位并返回
      调用本目录lane_backend获取基础误差和车道边界
      按race_sequence只启用当前阶段
      斑马线 → 巡线10秒 → 一次制动 → 停10秒 → 巡线cone_after_light_seconds秒 → 视觉绕桶 → 巡线parking_after_cones_seconds秒 → parking4
      用原PD/油门生成控制，按等待/停车状态覆盖
      反馈最终命令用于既有计时，返回结果
    Output.send：发送最终命令（仅显式enable-motion时启用GPIO）
    记录日志、终点照片，按需显示原图/标注
  退出：按原逻辑回到中位并释放输入
```

两个目录分别保留独立源码与配置；室外时序新增后，不再要求主流程与室内逐字相同。没有跨目录依赖；以后单独修改一个副本不会自动同步另一个。旧parking、parking2、二维码/深度/定时停车、固定S和TCP仿真入口已从活动程序移除。

已清理文件内部残留：旧固定/直接绕桶配置、旧锥桶颜色阈值和专用PD分支、parking3不再使用的蓝色/白色/纵向黄线参数，以及无人使用的辅助变量和形参。生效参数与检测、转向公式保持原值。

停车白板检测已同步 `jgy/race`：取消宽250/高300像素的最大尺寸限制，保留最小尺寸、面积、宽高比、位置、填充率和连续确认条件。停车制动使用本目录当前的 `parking3_brake_pwm`、`parking3_brake_seconds`；后续帧不会重启制动，照片延迟从中位锁停开始计算。

## 红灯及时制动

本节只适用于单独light组合或 `all_light_fixed_enabled: 0` 的视觉灯流程。

`light_red_confirm_frames: 1` 表示原识别器首次给出红灯候选就触发制动，避免继续前进等待第二次确认。改为2可恢复原来的连续确认后制动。这里不改灯牌、发光、颜色或夜间检测算法；绿灯仍须原识别器确认并连续满足 `light_green_seconds` 才提前放行。单次红灯误检也会触发停车；停车锁定后，丢灯不会重新前进，重复红灯不会重启制动或10秒计时。

日志 `light_confirmed` 仍表示原识别器的连续确认结果，因此可能出现 `light_confirmed=0` 同时 `LIGHT_BRAKE`，这是先制动的预期行为。制动力和时长仍由 `light_brake_pwm`、`light_brake_seconds` 控制。

## 当前停车现场配置

`parking3_stop_min_area: 1000`、`parking3_wait_side_pwm: 10600`、`parking3_follow_timeout_seconds: 0` 已与 jgy/race 同步；0表示禁用跟边超时停车，继续等待白板确认。停车制动为 `parking3_brake_pwm: 8000`、`parking3_brake_seconds: 0.3`。后续现场调参以本目录 `config/outdoor.yaml` 为准。

## 无GPIO回归验证

运行 `bash tests/run_fixed_light.sh`。合成偏移车道测试覆盖挡板发车、斑马线原流程、10秒巡线时转向修正、0.8秒一次制动、10秒中位等待、灯后5秒/2秒巡线与检测门控、灯色检测次数为0，以及夜间开关、视觉模式恢复、部分模块和计时边界。测试不创建硬件Output，测试桩也拒绝GPIO输出。

## parking4接入

`all` 默认使用parking4；单项可用 `--modules lane,parking4`。与jgy/race当前室外停车参数同步：跟边PWM10700、等待选侧PWM10600、近端边距150px、白板最小面积1000、制动8000/0.3秒，黄色T形连续2帧且0.1秒确认。停车识别仅在绕桶并回中后巡线指定时间完成才解锁；日志 `parking_cruise_remaining_s` 显示剩余时间。参数保存后重启生效。此次接入仅做编译和配置检查，实车由用户启动。
