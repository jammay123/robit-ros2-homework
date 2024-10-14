# ros2_HW3_readme

## ROS2와 Qt를 활용한 TurtleSim 제어

- 패키지 구조

```bash
ros2_custom_interfacd_pkg/
├── ros2_custom_interface/        
│   ├── action/
│   ├── include/
│   ├── msg/                    
│   ├── srv/
│   ├── src/
│   ├── CMakeLists.txt            
│   └── package.xml               
└── turtle_control/               
    ├── .vscode/
    ├── build/
    ├── include/
    ├── src/
    │   ├── main.cpp              
    │   ├── mainwindow.cpp        
    │   └── turtle_control.cpp    
    ├── ui/
    │   └── mainwindow.ui         
    ├── CMakeLists.txt            
    └── package.xml              

```

- 프로젝트 클론

```bash
git clone https://github.com/robit_ros2_homework/HW3.git
```

- 빌드

```bash
cd turtle_control_pkg
colcon build
```

- 소싱

```bash
source install/setup.bash
```

- 실행

```bash
ros2 run ui_test ui_test
ros2 run turtlesim turtlesim_node
```
