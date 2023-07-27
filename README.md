# ubuntu_uav_ass
git remote set-url origin https:// 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
@github.com/https://github.com/LWL0857/ubuntu_uav_ass.gitLWL0857/ubuntu_uav_ass.git
fzp_p  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
scasc

# 机器环境

Ubuntu20.04 ros2 foxy

## 库依赖

### serial库 --用来实现串口发送

下载这个ros 2  serial库

```
 https://github.com/ZhaoXiangBox/serial
```

#### 安装步骤

Adapt to ROS2 foxy.

#### Install

Get the code:

    # open a new terminal 
    cd serial
    mkdir build

Build:

    cd build
    cmake ..
    make

Install:

    sudo make install

也可以不用 make install ,直接和你想要调用本API的ROS2 节点放在同一个 src 下编译即可。有关于本串口操作包的 API 在线文档如下：

参考链接：http://t.csdn.cn/9y3An

### vrpn_mocap客户端--用于接收局域网下动捕主机广播的数据

ros的vrpn_mocap库位置

```
https://github.com/LWL0857/T_vrpn_mocap-release.git
```

## vrpn\_mocap

ROS2 [VRPN](https://github.com/vrpn/vrpn) client built pirmarily to interface
with motion capture devices such as VICON and OptiTrack. A detailed list of
supported hardware can be found on
[vrpn wiki](https://github.com/vrpn/vrpn/wiki/Available-hardware-devices).

## Installation

安装依赖

```
sudo apt-get install ros-foxy-vrpn
```

首先clone源码到ros2工作空间的src文件夹下，

将clone的文件夹改名成vrpn_mocap功能包名

在src文件夹同级目录进行

```
colcon build
```

然后

```
source install/setup.bash
```

```
ros2 launch client.launch.yaml server:=<server ip> port:=<port>
```



1. Clone this repo into your ROS2 workspace
2. Run `rosdep install --from-paths src -y --ignore-src` to install dependencies
3. Run `colcon build`
4. Your usual ROS2 routines: `source install/setup.zsh`, etc.

### Usage

#### Launch Default Configuration from Command Line

Run the following command,

```bash
ros2 launch client.launch.yaml server:=<server ip> port:=<port>
```

replacing `<server ip>` and `<port>` with your VRPN server ip and port, e.g.

```bash
ros2 launch client.launch.yaml server:=192.168.0.4 port:=3883
```

Then with `ros2 topic list`, you should be able to see the following topics

```bash
/vrpn_mocap/client_node/<tracker_name>/pose
/vrpn_mocap/client_node/<tracker_name>/twist # optional when mocap reports velocity data
/vrpn_mocap/client_node/<tracker_name>/accel # optional when mocap reports acceleration data
```

where `<tracker_name>` is usually the name of your tracked objects.

#### Customized Launch

Check out the default [parameter file](config/client.yaml) and
[launch file](launch/client.launch.yaml). You can then write your own launch
file with custom configurations.

#### Parameters

- `server (string)` -- server name, either ip address or domain name (default: `"localhost"`)
- `port (int)` -- VRPN server port (default: `3883`)
- `frame_id (string)` -- frame name of the fixed world frame (default: `"world"`)
- `update_freq (double)` -- frequency of the motion capture data publisher (default: `100.`)
- `refresh_freq (double)` -- frequency of dynamic adding new tracked objects (default: `1.`)
- `multi_sensor (bool)` -- set to true if there are more than one sensor (frame) reporting on
  the same object (default: `false`)

# 本库代码使用

## Install

首先clone源码到工作空间下，并将文件夹改名为src，

即src下有源码中的四个功能包 uav_base uav_msg等

在src同级文件目录下

```
colcon build
ros2 launch uav_base uav.launch.py  use_mocap:=true port_name:="ttyUSB0"
```









