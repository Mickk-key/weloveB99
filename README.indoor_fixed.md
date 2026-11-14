# 室内固定绕行调试

室内停车最新改为选定挡板反侧并持续沿该侧边线，参数 `parking_follow_pwm` / `parking_edge_margin_fraction`，详见 [室内沿边停车](README.indoor_parking_edge.md)。旧的定时绕板与回中说明不适用于当前室内配置。

入口 `run_indoor.sh` 明确指定 `--env indoor --config /home/pi/yidui/jgy/race_suite_outdoor_v2/config/indoor.yaml`（路径由脚本所在目录计算）。配置使用 `lane_backend: indoor`，室内白线识别、原有PD、10900前进油门和8000斑马线制动值保持原样。已启用一次识别触发整套S形绕两桶、挡板反侧沿边停车；动作数值为待实车调整的初值。

```bash
cd /home/pi/yidui/jgy/race_suite_outdoor_v2
# 参数检查，不开摄像头、不驱动车辆
sh run_indoor.sh --modules all --check-config
# 只看摄像头原图、识别图和车道掩码，不驱动车辆
sh run_indoor.sh --modules lane,cone --masks
# 实车：巡线 + 绕锥桶
sudo sh run_indoor.sh --modules lane,cone --enable-motion
# 实车：从停车区前测试绕蓝挡板 + 二维码停车（不等通过两桶）
sudo sh run_indoor.sh --modules lane,parking --enable-motion
# 实车：全部模块，整套S动作完成后启用停车
sudo sh run_indoor.sh --modules all --enable-motion
```

以上命令择一运行。驾驶时禁用预览，入口会自动忽略 --masks/--show；纯识别时仍可显式启用窗口，需树莓派桌面或X11转发。结束当前程序后再启动下一条。启动日志应显示 `env=indoor lane=indoor`，以及绝对路径 `file=/home/pi/yidui/jgy/race_suite_outdoor_v2/config/indoor.yaml`。

S形参数在 config/indoor.yaml 的 cone_s_steps 中；停车沿边速度改 parking_follow_pwm，贴边距离改 parking_edge_margin_fraction。停车不再使用 parking_maneuver_steps 或 parking_qr_seek_pwm。保存后退出并重启程序，无需重新编译。

完整动作语义、红灯/斑马线暂停、二维码最终停车和现有小二维码样例限制见 `README.fixed_maneuver.md`。本轮进行无GPIO的S形状态测试与编译，不启动摄像头或实车行驶。

室内S形路线示意、逐段参数和调参顺序见 [S形调参说明](README.indoor_s.md)。
