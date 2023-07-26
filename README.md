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

安装方法在readme里

参考链接：http://t.csdn.cn/9y3An

### vrpn_mocap客户端--用于接收局域网下动捕主机广播的数据

ros的vrpn_mocap库位置

```
https://github.com/LWL0857/T_vrpn_mocap-release.git
```

使用方法在里面readme

## 本库代码使用

首先clone源码到工作空间下，并将文件夹改名为src，

即src下有源码中的四个功能包 uav_base uav_msg等

在src同级文件目录下

```
colcon build`

`ros2 launch uav_base uav.launch.py
```









使用方法



