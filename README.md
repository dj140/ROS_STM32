# X-CTRL PRO M12
https://github.com/FASTSHIFT/X-CTRL

# 更新日志

## [v0.1] - 2020-04-16 
* 1.硬件搭建完成
* 2.软硬件测试通过
* 3.移植RCX协议栈

## [v0.2] - 2020-04-18
* 1.控制页面UI美工完成

## [v0.3] - 2020-04-19
* 1.控制页面设计完成
* 2.状态栏设计完成

## [v0.4] - 2020-04-20
* 1.添加强力振动马达，软件PWM驱动

## [v0.5] - 2020-04-22
* 1.主页面设计完成
* 2.握手页面设计完成

## [v0.6] - 2020-04-29
* 1.添加射频配置页面
* 2.添加扫频页面
* 3.参数掉电保存的BUG已修复

## [v0.7] - 2020-05-04
* 1.屏幕使用DMA发送，配合lvgl乒乓缓冲，画面无撕裂
* 2.修复Xbox360手柄模式退出后，音量变为0的bug

## [v0.8] - 2020-05-06
* 1.添加改进版的lv_settings菜单库
* 2.Radio配置页面完成
* 3.Channel配置页面完成
* 4.添加陀螺仪配置页面
* 5.添加通道绑定保存

## [v0.9] - 2020-05-14
* 1.在通道反向配置页面加入通道值显示
* 2.在陀螺仪配置页面添加限幅设置，以及显示
* 3.Misc设置页面完成
* 4.添加蓝牙配置页面
* 5.振动回传通道使用RCX_CHANNEL_DATA_MAX作为参考

## [v1.0] - 2020-05-18
* 1.为摇杆ADC读取的值添加可选的一阶低通滤波器
* 2."Buzz" 更名为"Audio"
* 3.手柄振动回传添加到Passback组
* 4.添加JoystickMap摇杆映射管理器
* 5.更新MTM的CPU占用统计使能开关
* 6.可越过I2C设备扫描检查
* 7.添加秒表图标和游戏图标

## [v1.1] - 2020-05-27
* 1."Motor" 和 "BigMotor" 改名为 "LRA Motor" 和 "ERM Motor"
* 2.添加摇杆配置页面
* 3.添加Model模型管理器以及配置页面
* 4.在关机时自动保存数据，减少储存器磨损

## [v1.2] - 2020-06-02
* 1.优化模型管理
* 2.修复主菜单的BUG
* 3.反转Home页面的翻动按钮逻辑
* 4.lv_joystick控件封装完成
* 5.摇杆曲线配置界面完成

## [v1.3] - 2020-06-06
* 1.添加电流显示

## [v1.4] - 2020-06-12
* 1.更新ButtonEvent
* 2.更新SwitchEvent
* 3.添加旋转编码器事件库
# ROS_STM32
