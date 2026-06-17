# ChingMu VRPB ROS2 插件安装步骤
## 1.安装vrpn库
ros2版本foxy下安装
```
sudo apt install ros-foxy-vrpn-mocap
```
ros2版本humble下安装
```
sudo apt install ros-humble-vrpn-mocap
```
依赖
```
rosdep install --from-paths src -y --ignore-src
```

## 2.创建工作空间
```
mkdir -p ~/chingmu_ws/src 
cd ~/chingmu_ws/
```
`catkin_ws` 工作空间的名称，可以自定义  
`mkdir -p ~/catkin_ws/src`  递归创建目录  
`colcon build`  ros2下的编译指令  

## 3.拷贝程序代码，编译
拷贝vrpn_mocap至前一步创建的src文件夹下
在chingmu_ws目录下使用colcon编译工程
```
colcon build
```
## 4.刷新环境变量
```
source install/setup.bash
```

## 5.运行代码
```
ros2 launch ./src/vrpn_mocap/launch/client.launch.yaml server:=<server ip> port:=<port>
```
例如：
```
ros2 launch ./src/vrpn_mocap/launch/client.launch.yaml server:=192.168.0.4 port:=3883
```

## 6.查看接收到的数据
`ros2 topic list`
```
cszecsz-virtual-machine:-/Desktop$ ros2 topic lis 
/parameter_events
/rosout
/vrpn_mocap/MCServer/accel_id_0
/vrpn_mocap/MCServer/pose_td_0
/vrpn_mocap/MCServer/velocity_id_0
```

`ros2 topic echo /vrpn_mocap/MCServer/pose_id_0`
```
cszecsz-virtual-machine:~/Desktop$ ros2 topic echo /vrpon_mocap/MCServer/pose
header:
  stamp:
    sec: 1699596837
    nanosec: 155918202
  frame id: world
pose
  position:
    X:188.65660095214844
    y:4.012427806854248
    z:571.76123046875
  orientation:
    x: 0.07953430712223053
    y:-0.23086997866630554
    z:0.9400380849838257
    w: -0.2381211668252945
```

## 参数
client.launch.yaml文件
```
launch:

- arg:
    name: "server"
    default: "localhost"
- arg:
    name: "port"
    default: "3883"

- node:
    pkg: "vrpn_mocap"
    namespace: "vrpn_mocap"
    exec: "client_node"
    name: "vrpn_mocap_client_node"
    param:
    -
      from: "$(find-pkg-share vrpn_mocap)/config/client.yaml"
    -
      name: "server"
      value: "$(var server)"
    -
      name: "port"
      value: "$(var port)"
```
- update_freq (double) -- frequency of the motion capture data publisher (default: 100.)
- refresh_freq (double) -- frequency of dynamic adding new tracked objects (default: 1.)
- sensor_data_qos -- use best effort QoS for VRPN data stream, set to false to use system default QoS which is reliable (default: true)
- multi_sensor (bool) -- set to true if there are more than one sensor (frame) reporting on the same object (default: false)



