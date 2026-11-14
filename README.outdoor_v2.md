> 2026-11-15：本文包含历史行为和参数。当前室外已同步室内的一次性 S 形、固定侧移回正后持续贴边；当前入口、参数与验证请先看 [室外同步说明](README.outdoor_sync.md)。

# 室外巡线 V2：新增版本，不替换原版

2026-11-14 更新：默认室外V2的锥桶与蓝挡板已改为 YAML 固定分段绕行。调整角度/时间/油门请看 [固定绕行调参说明](README.fixed_maneuver.md)，主配置末尾为 `cone_maneuver_steps` 与 `parking_maneuver_steps`。

现场逐项调参和任务命令见 [现场调参手册](现场调参手册.md)。

本次交付是独立的 `jgy/race_suite_outdoor_v2` 文件夹，旧 `jgy/race_suite` 整个文件夹保留不动。

新版源码 `src/lane_outdoor_v2.hpp`，独立配置 `config/outdoor_v2.yaml`。原版 `src/lane_original.hpp` 与原 `Downloads/lane/main.cpp` 不变。新旧版本在同一个集成入口中通过参数明确选择。

## 上传与运行

把整个新文件夹 race_suite_outdoor_v2 上传到 `/home/pi/yidui/jgy/race_suite_outdoor_v2`，在树莓派重新编译：

```bash
cd /home/pi/yidui/jgy/race_suite_outdoor_v2
sh build_pi.sh
# 新版只检测，直接弹出原图、识别图和掩码
sh run_outdoor_v2.sh --modules lane --masks
# 新版巡线行驶
sudo sh run_outdoor_v2.sh --modules lane --enable-motion
# 新版巡线＋红绿灯＋绕障
sudo sh run_outdoor_v2.sh --modules lane,light,cone --enable-motion
# 新版巡线＋人行道
sudo sh run_outdoor_v2.sh --modules lane,zebra --enable-motion
# 新版全部模块（二维码停车的已知限制仍见主报告）
sudo sh run_outdoor_v2.sh --modules all --enable-motion
# 切回原版室外巡线
sudo sh run.sh --env outdoor --modules lane --enable-motion
```

新文件夹的 `run.sh` 默认也选择 outdoor_v2。`--env outdoor_v2` 等价于新版脚本；室内仍用 `--env indoor`。同一时刻只启动一个占用相机的程序。窗口需要树莓派桌面或 MobaXterm X11。

## 保留与改进

保留单侧缺线时用画面另一边补位、同一误差单位、PD/舵机标定及模块组合。没有把缺失白线假装成真实边界。没有把原版直接覆盖成另一套算法。

原版灰度边缘把跑道纹理、其他标记也纳入 Hough 候选；角度差阈值 200° 在每侧约 72° 的允许区间里几乎不起筛选作用，容易混合不同方向/不同实际边界的线段；ROI 也包含较远、较低对比度区域。截图还经过压缩，曝光与地面反光也会改变边缘，但仅靠截图不能确定曝光是主因。

新版增加 3×3 降噪，结合低饱和度、亮度和局部对比度提取白色涂线，再对同一条线的候选按位置一致性分组，用 x(y) 拟合。左右线一致性在实际观察到地面白线的高度检查，避免固定高度落在天空/地平线而错误丢弃一侧。相邻帧只平滑当前确实检测到的线；缺线仍立即按画面边缘补位。

新配置取消了与任务无关的第 27 秒定时制动（`speed.corner_brake_s: 0`）；红灯和人行道主动制动仍执行。原版配置未因此更改。

## 现场调参数

修改 `config/outdoor_v2.yaml`，保存后退出重启，不需要重编译。

| 参数 | 作用及调整方向 |
|---|---|
| `outdoor_white_s_max: 130` | 越大允许越偏色的白线，但更可能收进红地面；不要只为多出线而一味增大 |
| `outdoor_white_v_min: 50` | 白线最低亮度；较暗时可小幅降低，过低会增加暗部噪声 |
| `outdoor_local_contrast: 3` | 白线相对周围的亮度差；降低会更敏感，也更容易认入纹理 |
| `outdoor_roi_top: 275` / `outdoor_roi_bottom: 475` | 640×480 中处理的地面行范围；调视角后先看掩码，不要把天空放进主要检测区 |
| `outdoor_min_span: 50` | 线段组需覆盖的最小竖直范围；降低能保留短线，但易把杂物认成线 |
| `outdoor_fit_tolerance: 16` | 线段分组容差，越大越容易合并不同白线 |
| `outdoor_smooth_alpha: 0.65` | 新帧所占权重；增大反应更快，降低更平滑；不预测消失的线 |
| `lane_pd_kp` / `lane_pd_kd` | 转向控制；先确认线框位置正确，再调控制增益 |
| `speed.start_pwm/corner_pwm/after_corner_pwm` | 行驶速度；10000 为当前中位，非 km/h |

先修识别再调 PD：如果绿色线框跑到了错误边缘上，增大转向增益只会更快朝错误目标转。

## 曝光与制动配置

曝光可以直接在 YAML 调。下面是手动示例，数值需在相机查询范围内：

```yaml
set_auto_exposure: 1  # 手动；3 为自动
set_exposure: 200     # 自动模式时此值不会控制自动曝光
light_brake_pwm: 9000
light_brake_seconds: 0.8
light_green_seconds: 0.5
zebra_brake_pwm: 9000
zebra_brake_seconds: 0.8
zebra_hold_seconds: 10.
```

行进中首次红灯→反转制动一次→中位等灯；持续红灯不反复反转，UNKNOWN 不解锁，连续绿灯满足 0.5 秒且制动已结束才放行。已经停车时不再次反转。没有“红灯 10 秒自动走”的规则。

制动 PWM 越低代表离当前 10000 中位越远，但真正刹车力度/是否倒退由实车电调决定；不能用仿真结果代替标定。时长单位为**秒**，不要把旧 README 的 `light_brake_ms: 800` 填成这里的 800 秒。三份旧环境配置也增加了 `light_brake_pwm/light_brake_seconds`。

## 验证和复现

8 张现场截图来自用户拼图，未把拼图伪造为连续驾驶视频。当前输出为 4 张双侧、4 张单侧，绿色线贴合可见白线；黄色虚线是缺侧的画面边缘，紫色为目标。不是“所有图片都必须双线”才正确。

56 个离线扰动用例（亮度×0.55/0.75/1.3、阴影带、模糊、JPEG 35、噪声）中，检测侧别保持一致，目标相对各自干净图的偏移均小于 20 像素；当前最大约 2.95 像素。此数值是对扰动的稳定性，不是人工标注精度，也不是实际道路成功率。

```bash
python3 tools/test_outdoor_v2.py
sh sim.sh --modules lane --scenario curve --lane-backend outdoor_v2 --config config/outdoor_v2.yaml
sh sim.sh --modules lane --scenario curve --lane-backend outdoor_v2 --config config/outdoor_v2.yaml --mirror
sh sim.sh --modules lane,zebra --scenario zebra --lane-backend outdoor_v2 --config config/outdoor_v2.yaml --offset 0.1 --yaw-deg 4 --noise 0.005
```

物理仿真只传图像与时间给 C++。程序只用真值判断测试是否越线，不使用真值帮车循迹。最新闭环结果见 `evidence/新版室外与制动验证.md`。本机算法耗时约 40～50ms/帧（同时有软件仿真负载），不能代表树莓派帧率。没有连接实车自动驾驶；还需要用实际相机原始视频和现场路段确认泛化。
