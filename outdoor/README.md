# outdoor 独立比赛程序

本文件夹可完整复制到另一台已安装 OpenCV 4、pigpio、g++ 的树莓派上，独立编译运行；源码和配置只引用本目录。树莓派二进制不能直接在Windows电脑运行。

本环境使用**室外原版灰度边缘巡线**。本目录独立维护；本次新增红灯超时和绕桶前巡线等待，只更新室外流程。现有巡线、灯色、锥桶和停车识别算法及现场参数保持原值，新增时序参数见下文。

## 编译与运行

```bash
cd /home/pi/yidui/smq/outdoor
bash build_pi.sh
bash run.sh --modules all --check-config
sudo bash run.sh --modules all --enable-motion
```

默认停车只有 **parking3**，默认红绿灯调用现有原版识别器。夜间运行添加 `--light-night`：

```bash
sudo bash run.sh --modules all --light-night --enable-motion
```

只巡线用 `--modules lane`（也是未传modules时的默认值）。保留 `lane,light,cone,zebra,parking3` 中现用模块组合；只有包含全部比赛模块时启用挡板发车及严格顺序。部分模块调试没有完整比赛的发车门控。`--image`、`--replay`、`--show`、`--masks` 和相机诊断仍保留。图片/回放不允许开启运动。

## 完整比赛顺序

1. `WAIT_START_BOARD`：保持中位，先确认蓝挡板确实出现。
2. `WAIT_START_CLEAR`：等待挡板连续消失确认；单帧漏检不发车。等待时间不消耗巡线起步计时。
3. `ZEBRA`：巡线并检测斑马线；按原配置制动、等待，然后放行，本轮只处理一次。
4. `LIGHT`：斑马线制动与等待完成后，按 `light_after_zebra_seconds` 前进巡线（当前6秒，这段不调用灯色识别），再按 `light_slow_pwm` 减速（当前10500），稳定0.3秒，开始原有灯色识别。现有识别器首次给出红灯候选即制动，并等待连续绿灯0.5秒；从首次触发红灯制动起最多等待 `light_red_timeout_seconds`（默认10秒，含制动）。绿灯确认通过提前放行，超时则直接完成灯阶段；重复红灯、未知和丢灯不重置计时。没有触发过红灯制动时不启动此超时。全流程初始连续绿灯也可完成该阶段。
5. `CONE_CRUISE`：绿灯放行或红灯超时后，沿原巡线前进 `cone_after_light_seconds`（默认5秒）；这段不调用锥桶检测、不累计通过数量，也不解锁停车。计时依据上一帧最终前进指令，停车时间不计入；不强制锁定舵机中位。
6. `CONE`：原室外形状候选、跟踪和视觉临时边界绕桶；确认两个桶通过并回中后进入停车。
7. `PARKING`：parking3独立确认终点蓝挡板，按黄横线选侧，沿所选巡线边界行驶，白色矩形确认后进入 `PARK3_BRAKE`：按 `parking3_brake_pwm`、`parking3_brake_seconds` 制动，转向居中；随后中位锁停。
8. `FINISHED`：制动结束后保持停车；等待原配置的拍照延迟后保存照片。发车挡板状态不会复用为终点挡板。

## 本次新增时序参数

在 `config/outdoor.yaml` 修改，保存后重新启动程序生效：

```yaml
light_red_timeout_seconds: 10.0  # 首次触发红灯制动起计时，0禁用超时
cone_after_light_seconds: 5.0   # 仅all：灯阶段放行后巡线多久才识别锥桶，0立即识别
```

两项范围均为0～60秒。红灯超时同样适用于室外单独light组合；绕桶前等待仅完整比赛启用，单独 `lane,cone` 测试不增加等待。识别结果不被伪造成绿灯：日志 `light_release_reason` 区分 `green` 与 `red_timeout`，`light_red_wait_remaining_s` 和 `cone_cruise_remaining_s` 显示剩余时间。超时由每帧时钟检查，不必等下一次灯色采样；仍会先完成原制动。

## 源码职责

| 文件 | 职责 |
|---|---|
| `main.cpp` | 参数、相机/图片/回放输入、主循环、输出和日志 |
| `suite.hpp` | 每帧调用模块并汇总最终电机/转向命令 |
| `race_sequence.hpp` | 挡板发车与阶段门控 |
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
      斑马线 → 红绿灯 → 绕桶前巡线等待 → 视觉绕桶 → parking3
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

`light_red_confirm_frames: 1` 表示原识别器首次给出红灯候选就触发制动，避免继续前进等待第二次确认。改为2可恢复原来的连续确认后制动。这里不改灯牌、发光、颜色或夜间检测算法；绿灯仍须原识别器确认并连续满足 `light_green_seconds` 才提前放行。单次红灯误检也会触发停车；停车锁定后，丢灯不会重新前进，重复红灯不会重启制动或10秒计时。

日志 `light_confirmed` 仍表示原识别器的连续确认结果，因此可能出现 `light_confirmed=0` 同时 `LIGHT_BRAKE`，这是先制动的预期行为。制动力和时长仍由 `light_brake_pwm`、`light_brake_seconds` 控制。

## 当前停车现场配置

`parking3_stop_min_area: 2200`、`parking3_wait_side_pwm: 10500`、`parking3_follow_timeout_seconds: 0` 已与 jgy/race 同步；0表示禁用跟边超时停车，继续等待白板确认。停车制动为 `parking3_brake_pwm: 8000`、`parking3_brake_seconds: 0.3`。后续现场调参以本目录 `config/outdoor.yaml` 为准。
