# ros2_HW2_readme

## 1.패키지 구조

```bash
HW2/
├── my_hw2_publish_pkg/
│   ├── include/
│   ├── src/
│   │   └── my_pub1_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
├── my_hw2_subscribe_pkg/
│   ├── include/
│   ├── src/
│   │   └── my_sub1_node.cpp
│   ├── CMakeLists.txt
│   └── package.xml
└── ros2_custom_interface/
├── include/
├── msg/
│   └── Hw2.msg
├── CMakeLists.txt
└── package.xml
```

## 2.패키지 클론

```bash
cd ~/robot_ws/src
git clone https://github.com/your-repo/ros2_custom_interface_echo.git
git clone https://github.com/your-repo/my_hw2_publish_pkg.git
git clone https://github.com/your-repo/my_hw2_subscribe_pkg.git
```

## 3.빌드

```bash
cd ~/robot_ws
colcon build --packages-select ros2_custom_interface my_hw2_publish_pkg my_hw2_subscribe_pkg
```

## 4.소싱

```bash
source ~/robot_ws/install/setup.bash
```

## 5.실행

```bash
ros2 run my_hw2_publish_pkg my_pub1_node
```

다른 터미널에서 sub노드 실행

```bash
ros2 run my_hw2_subscribe_pkg my_sub1_node
```
