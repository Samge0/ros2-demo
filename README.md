## ros2 demo

### 当前dev环境
- vscode >= `v1.89.1`
- vscode插件：`Dev Containers`，使用[devcontainer.json](.devcontainer/devcontainer.json)进入docker环境进行测试，需要修改[docker-compose.yml](.devcontainer/docker-compose.yml)中的`volumes`项目路径映射
- docker环境


### ros相关链接：
- [ros2 docker iamge: jazzy-desktop-full](https://hub.docker.com/r/osrf/ros/tags?page=&page_size=&ordering=&name=jazzy)
- [ros2 doc](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
- [鱼香ros论坛](https://fishros.org.cn/forum/)
- [鱼香ros2中文文档-动手学ROS2](https://fishros.com/d2lros2/#/)


### ros的仿真器gazebo
- ros的`jazzy`版本需要与`Gazebo Harmonic`搭配使用, [参考1>>](https://gazebosim.org/docs/harmonic/install_ubuntu), [参考2>>](https://gazebosim.org/docs)
- ubuntu系统下通过docker跑ros、gazebo时，需要在主机终端执行命令允许容器访问X11显示：
```shell
xhost +local:
```
- 同时运行docker时需要增加关于`x11`的相关配置(`volumes`、`environment`), 修改后的`docker-compose.yml`：
```yaml
version: '3.8'
services:
  ros2-demo:
    build: 
      context: ..
      dockerfile: .devcontainer/Dockerfile
      args:
        PROXY: http://192.168.xx.xx:7890
    volumes:
      - /xxx/ros2-demo:/app    
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # - ${XAUTHORITY}:/root/.Xauthority
    command: sleep infinity
    restart: unless-stopped
    network_mode: host
    ipc: host
    privileged: true
    environment:
      - DISPLAY
      - QT_X11_NO_MITSHM=1
```

### 构建自定义的ros包
- 创建ros包模板（python）
```shell
ros2 pkg create --build-type ament_python --license Apache-2.0 --node-name my_node my_package
```

- 创建功能块并在`setup.py`中声明，例如这里创建了[publisher_node.py](src/my_package/my_package/publisher_node.py)跟[subscriber_node.py](src/my_package/my_package/subscriber_node.py)，声明如下
```text
entry_points={
    'console_scripts': [
        'my_node = my_package.my_node:main',
        'subscriber_node = my_package.subscriber_node:main',
        'publisher_node = my_package.publisher_node:main'
    ],
},
```

- ros编译时指定目标包，减少构建耗时
```shell
colcon build --packages-select my_package
```

如果希望后续自动根据源码的变更动态编译，则可加上`--symlink-install`，免除每次都手动build的繁琐（可以理解为自动`reload`或自动`rebuild`）,记住修改这个后，需要执行`source install/local_setup.bash`生效，不然还是用的旧配置。
```shell
colcon build --symlink-install --packages-select my_package
```

- ros使用指定的安装包依赖（仅在当前窗口有效）
```shell
source install/local_setup.bash
```

- ros运行自定义包
```shell
ros2 run my_package my_node
ros2 run my_package subscriber_node
ros2 run my_package publisher_node
```


### ros相关说明
- ROS 2（Robot Operating System 2）起到了一个统筹和管理的作用，它为机器人系统提供了一个分布式的框架，可以用来构建和管理各种机器人应用。ROS 2 支持多种编程语言，包括 C++ 和 Python，这些语言可以用于开发 API、socket 通信、topic 消息订阅等功能，具体的工作由 ROS 2 来管理和调度。
- ROS 2 的基本概念
  - Node（节点）：节点是 ROS 2 中的基本执行单元，一个节点通常代表一个独立的功能模块，比如传感器数据处理、控制算法等。节点可以用 C++ 或 Python 编写。
  - Topic（话题）：节点之间通过发布（publish）和订阅（subscribe）话题来进行通信。话题是节点之间交换数据的一种方式，适合发布-订阅模式的数据传输。
  - Service（服务）：服务是 ROS 2 中的另一种通信机制，适合请求-响应模式的数据传输。一个节点可以提供服务，另一个节点可以调用该服务。
  - Action（动作）：动作是比服务更复杂的通信机制，适合长时间运行的任务。动作允许客户端发送请求并接收反馈和结果。
  - Parameter（参数）：参数是节点的配置数据，可以在运行时动态修改。


### 树莓派相关工具：
- [烧录工具下载](https://pidoc.cn/downloads/)
- [树莓派系统下载-ubuntu](https://cn.ubuntu.com/download/raspberry-pi)
- ubuntu系统安装步骤：
  - 下载并安装烧录工具
  - 下载操作系统（也可直接在烧录工具中选择下载）
  - 插入读卡器-tf卡
  - 烧录工具中选择树莓派版本+安装的系统镜像+tf卡
  - 一键烧录系统
  - tf卡插回树莓派，链接鼠标键盘显示器，按正常步骤进行系统的安装即可。
  - 安装完毕后，可选择ssh远程操作
    ```shell
    sudo apt update
    sudo apt install vim net-tools curl ssh-server -y
    sudo systemctl start ssh
    sudo systemctl enable ssh
    sudo systemctl status ssh
    ```
