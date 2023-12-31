> ver:2023/7/21 实现1v1比赛demo，且方便拓展至NvN。

> ver:2023/7/27 
> 1.增加了IMU直接计算获得的方位信息。
> 2.对安装过程进行简易化处理

> ver:2023/8/2 基于双线程与queue实现ros和网络通讯

### 对战平台使用方法

#### 1.环境配置

* 按照单机器人情况完成仿真环境配置与安装。
* 安装[SoccerXComm](https://moshumanoid.github.io/SoccerXComm/) SDK
```shell
pip3 install soccerxcomm
```
* 安装显卡驱动， N卡需要opengl才能让webots加速
#### 2.原仿真文件处理 

* 使用现在的`THMOS_webots_sim`替换原来的webots仿真文件夹。

* 将`battle_platform`放入工作空间的src下，并安装包。

```shell
catkin build battle_platform
```

* 踢球、步态参数均放在`battle_platform/config`下，然后相应的修改在`battle_platform/launch`下的`battlebot.launch`中的调用。

#### 3.测试

* 启动踢球和走路节点

```shell
roslaunch battle_platform battlebot.launch
```
若单纯测试通讯：

```shell
roscore
```

* 启动webots仿真

```shell
webots /battle_webots_sim/worlds/battle1v1demo.wbt
```

* 启动网络控制器

```shell
rosrun battle_platform battle_manager.py --bot_num=1
rosrun battle_platform demo_client.py
```

>  1. 该脚本基于[SoccerXComm](https://moshumanoid.github.io/SoccerXComm/) 与 `battle_player.py`
>  2. 参数`bot_num`为一边上场的机器人的数目

* 启动键盘测试控制器(测试用)

```shell
rosrun battle_platform keyboard_controller.py --id=r1
```

>  1. 该脚本通过键盘按键控制机器人的走路、踢球和转头
>  2. 机器人的代号是红方r1，r2...，蓝方b1，b2，...
>  3. 上面以红方1号机器人为例。
