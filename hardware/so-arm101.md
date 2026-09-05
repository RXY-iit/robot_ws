## follower
临时端口：/dev/ttyACM1
稳定路径：/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6054086-if00
USB：1a86:55d3 QinHeng USB Single Serial
序列号：5AE6054086

### leader USB 口：
Leader 5V：/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6084777-if00

## 整机安装外参（安装更新后的初值，2026-09-02）

参考：

- `hardware/real-bot-figure/IMG_0176.jpg`
- `hardware/real-bot-figure/IMG_0177.jpg`
- `hardware/real-bot-figure/IMG_0179.jpg`

SO101 固定在 D435 相机底座上方的 linear motor / 升降滑块安装架上。
2026-09-02 operator 调整了机械安装，使 Arm 底座平面与地面平行。该事实只
支持将 roll/pitch 设为 0；新照片不能可靠给出新的 xyz/yaw，因此它们暂时保留
此前初值，等待双相机 ChArUco 数据和独立尺寸测量进一步校准。

当前 `base_link -> arm/world` 照片估计初值：

| 参数 | 初值 |
|---|---:|
| x | +0.320 m（与 D435 底座相同）|
| y | 0.000 m（与 D435 底座相同）|
| z | +1.1175 m（当前人工确认 linear motor 位于最上方）|
| roll | 0 rad（底座平行地面）|
| pitch | 0 rad（底座平行地面）|
| yaw | 0 rad（2026-09-02 RViz 前后方向检查后修正）|

这些数值同时用于实际 Arm 和橙色 Preview 模型。D435 的物理底部螺孔高度
`camera_z=0.6375 m` 作为本轮相对高度零点；operator 给出的 Arm 底座可动范围为
相对该零点 `+0.25～+0.48 m`，即 `base_link` 下约 `z=0.8875～1.1175 m`。
2026-08-30 operator 确认当前 linear motor 位于最上方，因此静态显示采用上端
`0.48 m`，即 `arm_visual_z=0.6375+0.48=1.1175 m`。

该 z 不是已标定的 lift feedback。未来获得可信 ABZO/reference 后，TF 应改为：

`base_link -> lift_base_link -> lift_carriage_link -> arm_mount_link -> arm/base_link`
