Network-RC 摄像头只读页面

使用
  cd /home/pi/yidui/smq/controller
  bash start.sh

电脑/手机打开：
  http://192.168.137.139:8086/

停止（另开终端也可以）：
  cd /home/pi/yidui/smq/controller
  bash stop.sh

stop.sh 只停止本目录的 server.py，等待它关闭自己的摄像头采集进程；
服务未运行时会直接提示，无需重复处理。也可在启动服务的终端按 Ctrl+C 停止。
双击电脑上的 index.html 会打开上述树莓派页面，使用前须启动视频服务。

页面
  使用一队备份中的官方 Network-RC 2.7.3 界面、CSS、SVG 图标和 logo。
  保留开关切换、油门/方向/云台滑块、设置弹层、全屏和页面布局编辑。
  顶栏“编辑”（纸笔图标）开关打开后，可拖动控件和摄像头区域、调整大小；
  关闭编辑后，可操作滑块。位置和参数保存在当前浏览器，刷新后保留。
  左上角标签按原版显示“直连:数字”。数字为模拟延迟，并非实测值：
  约每0.9～1.1秒更新，平时在2～8之间小幅波动，偶尔短暂升高后回落，
  整体范围为2～24。页面隐藏时暂停数值变化，关闭时取消计时；不发送ping。
  页面开关和滑块的变化只反映浏览器本地状态，
  不代表小车状态，不会使小车运动、转向、转动云台、开灯或通断电。
  原前端的控制发送、配置读写和视频传输入口已经替换。WebSocket/WebRTC
  无真实连接，控制与设置请求被留在浏览器；只有视频与状态 GET 访问服务。
  原声音、TTS、重启/关机/更新操作不在小车上执行。视频固定为640x480、15fps，
  页面上的帧率参数不会修改采集设置。

视频服务
  server.py 仅提供静态页面、GET /status 与 GET /stream.mjpg。
  POST/PUT/PATCH/DELETE 返回 405；/control、/config 等接口不存在。
  仅使用 Python 标准库和树莓派已有的 ffmpeg。直接读取 H65 的 MJPEG 640x480、
  15fps 画面，不加载 GPIO 库，不启动原 network-rc，不执行回位或电机指令。
  有浏览器观看时才打开摄像头，多窗口共享同一个采集进程；关闭所有观看窗口后
  自动释放摄像头。检测到设备已被其他程序占用时显示提示，不停止其他程序。
  如果室内/室外比赛程序正在独占摄像头，本页面不能同时打开同一摄像头。
  不修改比赛程序、摄像头曝光设置、原遥控服务或系统开机启动设置。

文件
  index.html              页面入口
  assets/official.css      原版样式
  assets/logo-256.png      原版 logo
  assets/viewer.css        视频元素适配样式
  assets/viewer.js         本地交互适配、通信隔离和只读视频
  assets/viewer-config.js  原版界面的初始配置（非车辆实时状态）
  assets/official-ui.js    保留官方交互、替换控制通信后的界面代码
  assets/runtime.js        官方前端运行时
  assets/vendor.js         官方前端依赖
  assets/vendor.LICENSE.txt 依赖许可说明
  server.py               只提供视频的独立服务
  start.sh                启动脚本
  stop.sh                 停止本目录视频服务
  README.txt              本说明

来源（只读提取，原文件未改）
  /home/pi/yidui/backup/pre-three-parts-20261113/tools/remote_control/
  network-rc/front-end/build/
