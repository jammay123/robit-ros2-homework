#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class TeleopTurtle : public rclcpp::Node
{
public:
    TeleopTurtle()
    : Node("myteleop"), linear_speed_(2.0), angular_speed_(2.0)
    {
        // 퍼블리셔 설정
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos_profile);
        RCLCPP_INFO(this->get_logger(), "Teleop Turtle Node Started. Use Arrow keys to move the turtle, 's' for square, 'c' for circle, 't' for triangle, 'r' to return to center, and 'q' to quit.");

        // 터미널 설정
        setup_terminal();

        // 타이머 설정 (100ms 주기)
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TeleopTurtle::timer_callback, this));
    }

    ~TeleopTurtle()
    {
        // 원래 터미널 설정 복원
        restore_terminal();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_speed_;
    double angular_speed_;
    termios oldt;   // 원래 터미널 속성을 저장할 변수

    void setup_terminal()
    {
        // 현재 터미널 속성을 가져옴
        tcgetattr(STDIN_FILENO, &oldt); 
        termios newt = oldt; // 기존 설정을 복사
        
        // 터미널 모드 설정
        newt.c_lflag &= ~(ICANON | ECHO); // Canonical 모드 비활성화 및 ECHO 비활성화
        newt.c_cc[VMIN] = 1; // 최소 입력 문자 수
        newt.c_cc[VTIME] = 0; // 타임아웃 설정
        
        // 새로운 터미널 속성 설정
        tcsetattr(fileno(stdin), TCSANOW, &newt);
    }

    void restore_terminal()
    {
        // 원래 터미널 설정으로 복원
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    // 직선과 회전 동작을 위한 함수
    void draw_line(double linear, double angular)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = linear;
        twist.angular.z = angular;
        publisher_->publish(twist);
    }

    // 사각형 그리기
    void draw_square()
    {
        RCLCPP_INFO(this->get_logger(), "Drawing Square");
        for (int i = 0; i < 4; ++i)
        {
            draw_line(2.0, 0.0);  // 직진
            sleep(1); // 1초 대기
            draw_line(0.0, 1.57); // 90도 회전 (1.57 라디안)
            sleep(1); // 1초 대기
        }
    }
    
    // 원 그리기
    void draw_circle()
    {
        RCLCPP_INFO(this->get_logger(), "Drawing Circle");
        for(int i = 0; i < 4; i++)
        {
            draw_line(2.0, 1.57); // 원을 그리기 위해 직진과 동시에 회전
            sleep(1); // 10초 대기
        }
    }
    
    // 삼각형 그리기
    void draw_triangle()
    {
        RCLCPP_INFO(this->get_logger(), "Drawing Triangle");
        for (int i = 0; i < 3; ++i)
        {
            draw_line(2.0, 0.0);  // 직진
            sleep(1); // 1초 대기
            draw_line(0.0, 2.09); // 120도 회전 (2.09 라디안)
            sleep(1); // 1초 대기
        }
    }

    // 터틀을 중심으로 이동
    void return_to_center()
    {
        RCLCPP_INFO(this->get_logger(), "Returning to Center");
    }

    // 타이머 콜백 함수
    void timer_callback()
    {
        char c;
        ssize_t size = read(STDIN_FILENO, &c, 1); // 입력 읽기
        if (size > 0)
        {
            if (c == '\x1b') // 방향키는 이스케이프 시퀀스로 시작
            {
                char seq[2];
                read(STDIN_FILENO, &seq[0], 1);
                read(STDIN_FILENO, &seq[1], 1);

                auto twist = geometry_msgs::msg::Twist();
                bool publish = false;

                if (seq[0] == '[') // 방향키 인식
                {
                    switch (seq[1])
                    {
                        case 'A': // 위쪽 화살표
                            twist.linear.x = linear_speed_;
                            publish = true;
                            break;
                        case 'B': // 아래쪽 화살표
                            twist.linear.x = -linear_speed_;
                            publish = true;
                            break;
                        case 'C': // 오른쪽 화살표
                            twist.angular.z = -angular_speed_;
                            publish = true;
                            break;
                        case 'D': // 왼쪽 화살표
                            twist.angular.z = angular_speed_;
                            publish = true;
                            break;
                    }
                }

                if (publish)
                {
                    publisher_->publish(twist); // 메시지 퍼블리시
                    RCLCPP_INFO(this->get_logger(), "Published Twist: linear.x=%.2f, angular.z=%.2f",
                                twist.linear.x, twist.angular.z);
                }
            }
            else if (c == 's') // 사각형 그리기 명령
            {
                draw_square();
            }
            else if (c == 'c') // 원 그리기 명령
            {
                draw_circle();
            }
            else if (c == 't') // 삼각형 그리기 명령
            {
                draw_triangle();
            }
            else if (c == 'r') // 중심으로 이동 명령
            {
                return_to_center();
            }
            else if (c == 'q') // 종료 명령
            {
                RCLCPP_INFO(this->get_logger(), "Shutting down Teleop Turtle Node.");
                rclcpp::shutdown();
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTurtle>();
    rclcpp::spin(node); // 노드 실행
    rclcpp::shutdown(); // 종료
    return 0;
}

