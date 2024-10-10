# ros2_HW1_readme

## 노드 실행

1.워크 스페이스 생성

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2.패키지 클론

이 레포지토리를 클론하여 패키지를 워크스페이스에 추가

```bash
git clone https://github.com/jammay123/robit-ros2-homework.git
```

워크스페이스로 돌아가 빌드

```bash
cd ~/ros2_ws
colcon build
```

3.환경 설정

빌드 후, ros2 환경 변수를 설정하여 패키지에 접근할 수 있도록 해야한다.

```bash
source ~/ros2_ws/install/setup.bash
```

4.노드 실행

아래 명령어 입력

```bash
ros2 run myteleop_pkg teleop_turtle
```

사용법

- 방향키를 사용하여 터틀을 움직일 수 있습니다.
- 's'를 눌러 사각형을 그립니다.
- 'c'를 눌러 원을 그립니다.
- 't'를 눌러 삼각형을 그립니다.
- 'r'을 눌러 중심으로 돌아옵니다.(미구현)
- 'q'를 눌러 프로그램을 종료합니다.

기본적인 코드의 전체적인 구조는 오로카 ros강의 25강 퍼블리시 노드 코드와 틀이 같다.

[https://cafe.naver.com/openrt/24451](https://cafe.naver.com/openrt/24451)

## 터미널 세팅 함수

터미널에서 값을 받아와야 하기에 터미널 세팅을 해줘서 값을 받을 준비를 해주어야 한다.

원본 teleop_keyboard는 python으로 구현되어 있는데 python의 curses 라이브러리를 사용하는데 curses는 터미널의 입력과 출력을 관리하는 라이브러리로, 비캐노니컬 모드와 논블로킹 입력을 쉽게 처리할 수 있게 도와준다. 그리고 방향키 같은 경우도 c++과 달리 아스키 시퀀스로 변환 해줄 필요 없이 다양한 키 매핑이 가능하다. 

하지만 이번 과제에서 나는 c++로 구현했기 때문에 ‘termios.h’ 터미널 세팅을 해주었다.

```cpp
 void setup_terminal()		//터미널 세팅하는 함수
    {
        // 현재 터미널 설정을 가져옴
        tcgetattr(STDIN_FILENO, &oldt); 
        termios newt = oldt; 
        
        // 터미널 모드 설정
        newt.c_lflag &= ~(ICANON | ECHO); //에코 비활성화
        newt.c_cc[VMIN] = 1; 
        newt.c_cc[VTIME] = 0; 
        
        tcsetattr(fileno(stdin), TCSANOW, &newt);	//새로운 터미널
    }

    void restore_terminal()
    {
        // 원래 터미널 설정으로
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

```

- **`tcgetattr(STDIN_FILENO, &oldt);`**: 현재 터미널의 속성을 가져와 `oldt`라는 `termios` 구조체에 저장합니다. `STDIN_FILENO`는 표준 입력(즉, 터미널)에서 데이터를 읽을 때 사용하는 파일 디스크립터입니다.
- **`termios newt = oldt;`**: 기존의 터미널 속성을 복사하여 `newt`라는 새로운 `termios` 구조체를 생성합니다. 이 구조체는 터미널의 설정을 변경할 때 사용됩니다.
- **`newt.c_lflag &= ~(ICANON | ECHO);`**:
    - `ICANON`: 이 플래그가 설정되어 있으면 입력이 라인 단위로 처리됩니다. 이를 비활성화하면 입력이 즉시 처리됩니다(캐릭터 단위).
    - `ECHO`: 이 플래그가 설정되어 있으면 입력한 문자가 터미널에 표시됩니다. 이를 비활성화하면 입력한 문자가 화면에 보이지 않습니다.
    
    두 플래그를 모두 비활성화하여 사용자 입력이 즉시 처리되고 화면에 표시되지 않도록 합니다.
    
- **`newt.c_cc[VMIN] = 1;`**: 최소 입력 문자 수를 1로 설정합니다. 즉, 최소한 1개의 문자가 입력되어야 `read()` 호출이 반환됩니다.
- **`newt.c_cc[VTIME] = 0;`**: 입력 타임아웃을 0으로 설정합니다. 즉, 입력이 없을 경우 즉시 반환됩니다.
- **`tcsetattr(fileno(stdin), TCSANOW, &newt);`**: 변경된 `newt` 구조체를 사용하여 터미널 속성을 즉시 업데이트합니다. `TCSANOW`는 변경 사항을 즉시 적용하도록 지시합니다.
- **`tcsetattr(STDIN_FILENO, TCSANOW, &oldt);`**: 저장해 두었던 원래 터미널 속성(`oldt`)을 사용하여 터미널 설정을 복원합니다. 이 작업은 프로그램이 종료될 때 또는 필요할 때 호출되어 터미널이 정상 상태로 돌아가도록 합니다.

터미널 세팅 참고 링크 : https://github.com/aarsht7/teleop_cpp_ros2/blob/main/src/teleop_cpp_ros2.cpp

## 도형 그리기

```cpp
void draw_line(double linear, double angular)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear;
        twist.angular.z = angular;
        publisher_->publish(twist);
    }
```

- draw_line함수 기반으로 사각형, 원, 삼각형 구현

```cpp
// 사각형 그리기
    void draw_square()
    {
        RCLCPP_INFO(this->get_logger(), "Drawing Square");
        for (int i = 0; i < 4; i++)
        {
            draw_line(2.0, 0.0);
            sleep(1);
            draw_line(0.0, 1.57); // 1.57(rad) = 90도
            sleep(1);
        }
    }
    
    // 원 그리기
    void draw_circle()
    {
        RCLCPP_INFO(this->get_logger(), "Drawing Circle");
        for(int i = 0; i < 4; i++)
        {
            draw_line(2.0, 1.57); 
            sleep(1);
        }
    }
    
    // 삼각형 그리기
    void draw_triangle()
    {
        RCLCPP_INFO(this->get_logger(), "Drawing Triangle");
        for (int i = 0; i < 3; i++)
        {
            draw_line(2.0, 0.0); 
            sleep(1);
            draw_line(0.0, 2.09); // 2.09(rad) = 120도
            sleep(1);
        }
    }
```

- delay 시켜주는 sleep함수 사용

## 콜백함수

아스키 시퀀스 2,3번째 부터 인지 ⇒ ‘[’ 그리고 ‘a(위),b(아래),c(우),d(좌)’

노드 종료 ‘q’입력시 rclcpp::shutdown();; 함수 ⇒ 노드 종료

## 메인함수

ros2 25강 기반

spin(node) 함수로 실행
