# smq：室内与室外独立程序

活动程序只有 `indoor/` 与 `outdoor/` 两份。每份包含自己的 `src/`、`config/`、`build_pi.sh`、`run.sh`、`README.md` 和编译后的 `build/race`，可以单独复制出去编译运行，不引用另一份或父目录。

| 内容 | indoor | outdoor |
|---|---|---|
| 巡线 | 原室内HSV白线/霍夫拟合 | 原室外灰度边缘巡线 |
| 挡板、斑马线、灯色、绕桶、停车 | 与室外相同 | 原室外实现 |
| 绕桶 | 形状识别＋视觉绕两桶并回中 | 相同 |
| 停车 | 只有parking3 | 只有parking3 |
| 红绿灯 | 默认原版；可选--light-night | 相同 |

完整流程：确认蓝挡板出现 → 确认移开后发车 → 斑马线 → 红绿灯 → 绕桶 → parking3 → 锁停。

```bash
# 室内
cd /home/pi/yidui/smq/indoor
sudo bash run.sh --modules all --enable-motion

# 室外
cd /home/pi/yidui/smq/outdoor
sudo bash run.sh --modules all --enable-motion
```

夜间在任一命令后增加 `--light-night`。不传modules时仍默认只巡线；全流程需要 `--modules all`。

各目录运行 `bash build_pi.sh` 可单独编译；根目录 `bash build_pi.sh` 编译两份。根目录 `run.sh --env-indoor/--env-outdoor` 仅为旧命令转发器。详细调用关系、源码表和阶段说明见对应目录README。

非巡线参数采用现有室外配置，数值未重新调整；室内保留原巡线算法和白线参数。旧停车方案、固定动作表和不再生效的配置字段已经移除。

Git仓库只保存源码、配置、编译/启动脚本和说明。克隆后先在对应目录运行 `bash build_pi.sh`；编译生成的 `build/` 和运行生成的 `runs/` 均已忽略，不提交到Git。树莓派现有可执行文件保留在各自的 `build/race`。

本地历史源码、旧日志及离线验证资料已整理为 `backup/` 中的归档，供需要时查阅，不参与编译和启动，也不提交到Git。

验证包括两份程序各915帧的控制输出对照，以及默认/夜间模式的配置与无GPIO图像入口检查；这是离线验证，现场行驶仍需实车确认。
