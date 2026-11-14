# Race suite：室内控制参数同步室外

2026-11-15：室外采用当前已经调好的室内动作与控制参数，保留室外图像识别算法、曝光和检测标定。完整参数、改动前备份及调参说明见 [室外同步说明](README.outdoor_sync.md)。

默认室外入口是 `run_outdoor_v2.sh`，读取 `config/outdoor_v2.yaml`。`run_outdoor.sh` 使用原版室外视觉及 `config/outdoor.yaml`；`run_indoor.sh` 和室内 YAML 保持原样。

## 编译与检查

在树莓派安装有 OpenCV 4 开发库、g++、pkg-config 和 pigpio 的环境中：

```bash
cd /home/pi/yidui/jgy/race_suite_outdoor_v2
sh build_pi.sh
sh run_outdoor_v2.sh --modules all --check-config
sh tests/run_outdoor_sync.sh
```

最后两条是配置检查和离线合成图测试，不打开摄像头或 GPIO。桌面 Linux / WSL 可用 `sh build_local.sh` 编译无 GPIO 版本。GitHub 只保存源码，下载后需要在目标机器编译；运行日志和二进制不入库。

## 室外行驶

以下命令按需执行一条：

```bash
sudo sh run_outdoor_v2.sh --modules lane --enable-motion
sudo sh run_outdoor_v2.sh --modules lane,cone --enable-motion
sudo sh run_outdoor_v2.sh --modules lane,parking --enable-motion
sudo sh run_outdoor_v2.sh --modules all --enable-motion
```

驾驶默认无预览，兼容过滤旧命令中的 `--masks` / `--show`。纯识别预览可在有图形显示的终端使用 `sh run_outdoor_v2.sh --modules lane --masks`。所有模块共享同一摄像头和最终控制出口，一次运行一个程序。

## 当前任务流程

- 锥桶：只检测第一个桶，执行左右镜像的九段 S 形，然后恢复巡线；本次运行不再触发第二桶。
- 停车：挡板侧别确认期间继续前进，选定反侧后固定侧移、反打回正、短直行，再持续贴边至二维码确认停车。
- 同时启用锥桶和停车时，整套 S 完成后才解锁停车；只启用停车时直接识别挡板。
- 红灯、斑马线等待和二维码终停保留当前室内仲裁规则；实时 QR 搜索运行于后台。

`steer` 是相对舵机中心的 PWM 修正，`seconds` 是秒，`motor_pwm` 是绝对输出。电机中位是 `10000`。YAML 修改后重启生效；C++ 修改后重新编译。

本次执行编译和离线验证，未启动实车行驶。室外视觉仍需实际光照与跑道验证。历史文档和测试保留用于追溯；旧的逐桶绕行、缺线停等和旧速度表不代表当前室外行为，以 [当前同步说明](README.outdoor_sync.md) 与配置为准。室外视觉算法细节可参考 [室外 V2 说明](README.outdoor_v2.md)。
