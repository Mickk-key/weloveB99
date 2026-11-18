# indoor 独立比赛程序

本文件夹可完整复制到另一台已安装 OpenCV 4、pigpio、g++ 的树莓派上，独立编译运行；源码和配置只引用本目录。树莓派二进制不能直接在Windows电脑运行。

本环境仅巡线不同：**室内白线（HSV筛白＋霍夫/拟合）**。挡板、斑马线、红绿灯、绕桶、停车和控制流程均与室外对齐。非巡线参数取拆分前室外配置的原值，巡线保留本环境原算法。绕桶不再使用室内固定 S。

## 编译与运行

```bash
cd /home/pi/yidui/smq/indoor
bash build_pi.sh
bash run.sh --modules all --check-config
sudo bash run.sh --modules all --enable-motion
```

默认停车只有 **parking3**，默认红绿灯是原版颜色识别，不检测灯罩。夜间运行添加 `--light-night`：

```bash
sudo bash run.sh --modules all --light-night --enable-motion
```

只巡线用 `--modules lane`（也是未传modules时的默认值）。保留 `lane,light,cone,zebra,parking3` 中现用模块组合；只有包含全部比赛模块时启用挡板发车及严格顺序。部分模块调试没有完整比赛的发车门控。`--image`、`--replay`、`--show`、`--masks` 和相机诊断仍保留。图片/回放不允许开启运动。

## 完整比赛顺序

1. `WAIT_START_BOARD`：保持中位，先确认蓝挡板确实出现。
2. `WAIT_START_CLEAR`：等待挡板连续消失确认；单帧漏检不发车。等待时间不消耗巡线起步计时。
3. `ZEBRA`：巡线并检测斑马线；按原配置制动、等待，然后放行，本轮只处理一次。
4. `LIGHT`：斑马线完成后前进3秒，再以10700减速0.3秒，开始原有灯色识别。确认红灯制动后等待连续绿灯0.5秒；未知保持等待。全流程初始连续绿灯也可完成该阶段。
5. `CONE`：原室外形状候选、跟踪和视觉临时边界绕桶；确认两个桶通过并回中后进入停车。
6. `PARKING`：parking3独立确认终点蓝挡板，按黄横线选侧，沿所选巡线边界行驶，白色矩形确认后锁停。
7. `FINISHED`：保持停车；等待原配置的拍照延迟后保存照片。发车挡板状态不会复用为终点挡板。

## 源码职责

| 文件 | 职责 |
|---|---|
| `main.cpp` | 参数、相机/图片/回放输入、主循环、输出和日志 |
| `suite.hpp` | 每帧调用模块并汇总最终电机/转向命令 |
| `race_sequence.hpp` | 挡板发车与阶段门控 |
| `lane_backend.hpp`、`lane_v2.hpp`、`lane_geometry.hpp` | 本环境巡线入口、原算法和通用边界表示 |
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
      斑马线 → 红绿灯 → 视觉绕桶 → parking3
      用原PD/油门生成控制，按等待/停车状态覆盖
      反馈最终命令用于既有计时，返回结果
    Output.send：发送最终命令（仅显式enable-motion时启用GPIO）
    记录日志、终点照片，按需显示原图/标注
  退出：按原逻辑回到中位并释放输入
```

主流程、非巡线源码和夜间配置在两个目录中各自保存相同副本。没有跨目录依赖；以后单独修改一个副本不会自动同步另一个。旧parking、parking2、二维码/深度/定时停车、固定S和TCP仿真入口已从活动程序移除。

已清理文件内部残留：旧固定/直接绕桶配置、旧锥桶颜色阈值和专用PD分支、parking3不再使用的蓝色/白色/纵向黄线参数，以及无人使用的辅助变量和形参。生效参数与检测、转向公式保持原值。
