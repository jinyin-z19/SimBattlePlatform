> ver:2023/7/21 实现1v1比赛demo，且方便拓展至NvN。

### 对战平台使用方法

#### 1.环境配置

按照单机器人情况完成仿真环境配置与安装。

#### 2.原仿真文件处理 

* 将`battle1v1demo.wbt` 放入 `THMOS_webots_sim/worlds/`
* 将`battle_robot_uni` 放入 `THMOS_webots_sim/controllers/`
* 将`battlebot.launch` 放入 `bitbots_thmos_meta/bitbots_motion/bitbots_quintic_walk/launch/` 
* 将`teleop_keyboard.py` 放入`bitbots_thmos_meta/bitbots_misc/bitbots_teleop/scripts/`替换原文件，并给予控制权限

#### 3.测试

* 启动踢球和走路节点
`roslaunch bitbots_quintic_walk battlebot.launch` 

* 启动webots仿真
`webots ~/THMOS_webots_sim/worlds/battle1v1demo.wbt` 

* 启动键盘测试控制器
`rosrun bitbots_teleop teleop_keyboard.py --id=r1`

>  1. 该脚本通过键盘按键控制机器人的走路、踢球和转头
>  2. 机器人的代号是红方r1，r2...，蓝方b1，b2，...
>  3. 上面以红方1号机器人为例。


