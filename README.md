> ver:2023/7/21 实现1v1比赛demo，且方便拓展至NvN。

### 对战平台使用方法

#### 1.环境配置

按照单机器人情况完成仿真环境配置与安装。

#### 2.文件替换 

* 将battle1v1demo.wbt放入THMOS_webots_sim/worlds/文件夹下
* 将battle_robot_uni放入THMOS_webots_sim/controllers文件夹下
* 将battlebot.launch放入bitbots_thmos_meta/bitbots_motion/bitbots_quintic_walk/launch/文件夹下 
* 将teleop_keyboard.py放入bitbots_thmos_meta/bitbots_misc/bitbots_teleop/scripts文件夹下，替换原文件

#### 3.测试

* 启动踢球和走路节点
`roslaunch bitbots_quintic_walk battlebot.launch` 

* 启动webots仿真
`webots ~/THMOS_webots_sim/worlds/battle1v1demo.wbt` 

* 启动控制器

>  1. 该脚本通过键盘按键控制机器人的走路、踢球和转头，运行时有提示(也可以使用rqt控制，但rqt不方便测试踢球和转头) 2. 利用命令行传参选择机器人(机器人的代号是红方r1，r2...，蓝方b1，b2，...)下面以红方1号机器人为例。

`rosrun bitbots_teleop teleop_keyboard.py --id=r1`
