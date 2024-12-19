### Universe环境

- Ubuntu ：22.04
- ROS2 ：humble
- autoware ：Universe

Autoware 作为第一款自动驾驶开源框架，自诞生以来就培养的一大批学习中的开发者。甚至对于一些机器人项目和自动驾驶项目，也起到了孵化作用，能够快速地帮助搭建一个可用的自动驾驶系统，以便在此基础上进行自研。

为了区分，官方用 Autoware.AI 指代 ROS1 版本，而 Autoware.Unverse 是目前基于 ROS2 的最新版本。无论是论文还是代码我们尽量学习最新的，在两代 Autoware 框架上，即使你完全没有看过 Autoware.AI，也可以直接研读和参考 Autoware.Universe，毕竟学习的是算法，ROS 甚至 C++ 都是工具而已，而工具都是可以在使用中学会。下面着手从autoware.universe上进行跟进学习 ，部分教程仍然适用。

```sh
# autoware.universe 文档
https://autowarefoundation.github.io/autoware.universe/main/
# 官网源码安装教程链接
https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/
# github humble
https://github.com/autowarefoundation/autoware/tree/humble
```

参考链接 ： https://blog.csdn.net/robotpengxu/article/details/127975566

### 安装

```sh
sudo apt-get -y update
sudo apt-get -y install git
```

- 克隆源码 

```sh
mkdir ~/autoware_universe ; cd autoware_universe

# 1、不推荐 - 官方最新的仓库 （因为后面的是基于 awsim-stable 分支开发，可以checkout 到 awsim-stable 分支，或者使用gitee仓库）
git clone -b awsim-stable https://github.com/autowarefoundation/autoware.git
# 重命名并切换到 humble 版本
mv autoware autoware_ws ; cd autoware_ws ; git checkout humble
cd ~/autoware_universe/autoware_ws ; mkdir src
# 建议适用 vpn 全局代理
vcs import src < autoware.repos

# 2、推荐 - 后续基于的开发分支 , 
git clone git@gitee.com:QY_Benny/autoware.universe.git
mv autoware.universe autoware_ws ; cd autoware_ws ; git checkout awsim-stable
```

- 依赖系统

官方给了一个 setup-dev-env.sh 脚本进行依赖安装，同时也指出必须清楚脚本内容，这里直接运行脚本安装一系列的依赖 ：

```sh
cd ~/autoware_universe/autoware_ws
sudo apt install python3.10-venv
# 如果出错就多执行几次，可能是网络问题
./setup-dev-env.sh
# 重新启动
reboot
```

- 安装程序依赖

```sh
cd ~/autoware_universe/autoware_ws
source /opt/ros/humble/setup.bash
# 可以使用rosdepc
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

- 编译

241包编译成功了就行

```sh
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```



### 补充安装细节

#### error1 : 

- /opt/ros/humble/include/lanelet2_core/geometry/impl/LineString.h:130:9: error: typedef ‘using BasicPoint = PointT’ locally defined but not used [-Werror=unused-local-typedefs]  130 |   using BasicPoint = PointT ;

需要修改下源码解决这个报错：

```sh
code /opt/ros/humble/include/lanelet2_core/geometry/impl/LineString.h

# 将 130 行注释
// using BasicPoint = PointT;
```

#### error2 ：

- C++: fatal error: Killed signal terminated program cc1plus

编译时 swap 内存不足，本文的笔记本另外设置 swap 32G 内存，运行内存条32G ，参见 ：[11_swap.md](../../../Linux/Ubuntu/11_swap.md)
