> 2026-11-15：本文包含历史行为和参数。当前室外已同步室内的一次性 S 形、固定侧移回正后持续贴边；当前入口、参数与验证请先看 [室外同步说明](README.outdoor_sync.md)。

# 固定动作绕锥桶与蓝挡板

室内停车最新改为选定挡板反侧并持续沿该侧边线，参数 `parking_follow_pwm` / `parking_edge_margin_fraction`，详见 [室内沿边停车](README.indoor_parking_edge.md)。旧的定时绕板与回中说明不适用于当前室内配置。

室内最新流程已改成“一次识别触发整套S形绕两桶”，使用 `indoor.yaml` 的 `cone_s_steps`，详见 [室内S形调参](README.indoor_s.md)。本文逐桶确认/消失计数的说明仅适用于未开启 `cone_s_course_once` 的旧模式。

2026-11-14：`run_outdoor_v2.sh` 默认读取 `config/outdoor_v2.yaml`，已启用固定分段绕行。普通巡线算法、红绿灯与斑马线检测保留。下列动作数值是供现场手调的初值，未作为实车绕行成功的标定值。

室内调试入口为 `sh run_indoor.sh`，明确读取本目录的 `config/indoor.yaml`，动作参数在该文件末尾。以下 `outdoor_v2.yaml` 的路径说明适用于室外；字段含义两版相同。

## 改哪里

修改 `config/outdoor_v2.yaml` 末尾两组动作：

| 参数 | 用途 |
|---|---|
| `cone_maneuver_steps` | 绕锥桶的完整动作顺序，默认转出→摆到旁边→直行越桶→回转→摆正 |
| `parking_maneuver_steps` | 绕蓝挡板的完整动作顺序，默认转出→绕过→反打摆正 |
| 每段 `steer` | 转向量；正数向所选绕行侧打舵，负数反打，0回正 |
| 每段 `seconds` | 本段前进命令持续秒数，可以填写小数 |
| 每段 `motor_pwm` | 本段固定油门，覆盖普通巡线油门和 `cone_slow_pwm`，当前要求 `10000 < 值 <= 12000` |
| `cone_fixed_side` / `parking_fixed_side` | `0` 自动选择，`1` 固定从右绕，`-1` 固定从左绕；开始后方向锁定 |
| `cone_avoid_start_y` | 锥桶框底边达到此图像行才触发，当前340；越小越早绕 |
| `parking_maneuver_start_y` | 蓝挡板框底边达到此图像行才触发，当前340；设0表示确认、选侧后立即绕 |
| `cone_rearm_seconds` | 完成全套动作后等待多久才允许通过计数/重新识别，当前0.5秒；还要求原目标消失 `cone_lost_grace_s` 秒 |
| `parking_qr_seek_pwm` / `parking_qr_seek_steer` | 绕蓝挡板结束后的固定前进油门和转向量，默认10900与0，保持在选定半边寻找二维码 |

例如，改锥桶第一段：

```yaml
cone_maneuver_steps:
  - { name: OUT, steer: 5.0, seconds: 0.65, motor_pwm: 10900 }
```

这表示：以10900油门向绕行侧打5个PWM单位，保持0.65秒。**这里的5不是5度。** 舵机命令按 `servo_center - side * steer` 计算，再受 `servo_min` / `servo_max` 限制。目前中心80、最小55、最大86：右绕且 `steer: 5` 输出75，左绕输出85；反打方向相反。左打最多到86，因此左向修正大于6仍只会输出86；不要靠放宽机械边界增加转角。

`seconds` 越长，该段走得越久；`steer` 绝对值越大，该段打舵越多；`motor_pwm` 影响实际行进速度。因此动作轨迹由三者共同决定，不是按几何计算保证完成某个角度或严格半圆。

可自由添加、删除、重排动作。每段均需有 `name/steer/seconds/motor_pwm`；`name` 用英文字母、数字、下划线；最多20段，每段大于0且不超过15秒，总计不超过60秒。删除不需要的动作段即可，不能用0秒代替删除。

## 实际执行规则

1. 锥桶仍按原蓝色/形状识别和连续帧确认，达到起绕行号后锁定左右方向，按列表顺序执行。动作中忽略巡线误差，短暂看不到桶或车道线也不提前退出。
2. 每段直接给固定舵机命令和固定油门，不受巡线PD与 `steer_rate` 改写。普通巡线仍使用原PD和转向缓变。机械限位继续生效。
3. 全部锥桶动作完成后恢复巡线；只有原目标消失且冷却完成，才增加一次 `passed`。目标一直可见时等待离开，不重新执行同一套动作。默认累计通过2桶后才开放停车模块，仍由 `parking_after_cones` 控制。
4. 蓝挡板默认按原有双侧车道边界判断空闲半边；未选清时保持 `WAIT_BOARD_SIDE`。选侧后尚未达到起绕行号时以原巡线接近，达到阈值才执行固定动作。可用 `parking_fixed_side` 人工指定方向。
5. 绕板动作结束后，以 `parking_qr_seek_pwm/steer` 固定前进寻找二维码，不再用 `parking_target_offset/parking_offset_rate` 拉动巡线目标。原来的二维码尺寸、半区和连续确认规则保留；动作过程中若二维码已经确认，也立即终停。终停为锁存状态，不会重新起步。
6. 绕板动作中缺少真实边界不会打断固定动作，但无法确认二维码；动作结束后若仍缺真实边界，按原规则进入 `WAIT_REAL_BOUNDARIES` 等待，不用图像边缘冒充车道来判断二维码半区。
7. 红灯、斑马线制动/等待和最终停车仍优先。固定动作根据最终发出的前进命令计时，交通停车期间不消耗剩余动作时间，放行后接着执行。这里是命令计时，没有编码器/IMU角度闭环；低帧率会让各段变长，一次延迟也不会跳过未执行的后续动作。

当前配置中旧的 `cone_pd_*`、偏移速率、通道余量等参数不控制固定绕桶动作；`parking_target_offset/parking_offset_rate` 不控制固定绕板动作。`indoor.yaml` 与 `outdoor_v2.yaml` 均已启用固定动作，各自独立调参。室内使用 `sh run_indoor.sh`，详见 `README.indoor_fixed.md`。其他环境YAML与旧模式保留。若显式选择 `--parking-mode depth`，运行的是原深度停车，以上蓝挡板动作不生效。

## 保存和运行

每次保存 YAML 后，退出并重启程序使参数生效，无需重新编译。先检查参数：

```bash
cd /home/pi/yidui/jgy/race_suite_outdoor_v2
sh run_outdoor_v2.sh --modules all --check-config
```

用户自行试跑时可分别使用：

```bash
# 巡线 + 绕锥桶
sudo sh run_outdoor_v2.sh --modules lane,cone --enable-motion
# 从停车区前单独测试：绕蓝挡板 + 原二维码终停，不等待通过两桶
sudo sh run_outdoor_v2.sh --modules lane,parking --enable-motion
# 全部模块
sudo sh run_outdoor_v2.sh --modules all --enable-motion
```

日志的 `state` 会显示 `CONE_FIXED_1_OUT`、`CONE_FIXED_2_LEVEL`、`PARK_FIXED_1_OUT` 等当前段号/名称；`fixed_remaining_s` 是本段剩余秒数。红灯/斑马线接管时以最终 `state` 为准。动作结束寻二维码时显示 `PARK_FIXED_WAIT_QR`，确认后为 `QR_STOP_LATCHED`。

调参时先固定油门，调整第一段转向量和时间确认侧移量，再调直行段长度避免过早回转，最后调反打与摆正段。两类障碍使用独立列表，可分别调整。

本次程序验证使用无GPIO编译和合成图像/已有二维码图片，覆盖动作顺序、方向锁定、漏检、连续确认、冷却、交通停车暂停、转向限幅和二维码终停；未自动启动实车行驶，也未占用摄像头。现场绕行距离和各段时间仍需按真实车辆调整。

原有 `tests/small-qr.jpg` 小二维码样例在这台树莓派上使用修改前的检测代码也未识别，因此完整 `python3 tests/run.py` 仍会在该原有图片断言处失败。检测器代码逐字保留，本次没有用放宽二维码条件来消除此失败；新动作、普通二维码样例及模块状态分别验证。不要将本次改动视为小二维码识别问题已经解决。
