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
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", qos_profile);		//turtle1/cmd_vel은 토픽 이름
        RCLCPP_INFO(this->get_logger(), "Start, 's' -> square, 'c' -> circle, 't' -> triangle, 'r' -> return to center, and 'q' -> shutdown.");		//print함수 비슷함

        setup_terminal();

        timer_ = this->create_wall_timer(
            100ms, std::bind(&TeleopTurtle::timer_callback, this));
    }

    ~TeleopTurtle()
    {
        restore_terminal();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double linear_speed_;
    double angular_speed_;
    termios oldt;  

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

    //도형 기반 함수
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

    void return_to_center()
    {
        RCLCPP_INFO(this->get_logger(), "Return to Center");
    }

   
    void timer_callback() 	// 타이머 콜백 함수
    {
        char c;
        char size = read(STDIN_FILENO, &c, 1); 
        if (size > 0)
        {
            if (c == '\x1b') // 방향키 (아스키시퀀스)
            {
                char seq[2];
                read(STDIN_FILENO, &seq[0], 1);
                read(STDIN_FILENO, &seq[1], 1);

                auto twist = geometry_msgs::msg::Twist();
                bool publish = false;

                if (seq[0] == '[')
                {
                    switch (seq[1])
                    {
                        case 'A': // 위
                            twist.linear.x = linear_speed_;
                            publish = true;
                            break;
                        case 'B': // 아래
                            twist.linear.x = -linear_speed_;
                            publish = true;
                            break;
                        case 'C': // 오른쪽
                            twist.angular.z = -angular_speed_;
                            publish = true;
                            break;
                        case 'D': // 왼쪽 
                            twist.angular.z = angular_speed_;
                            publish = true;
                            break;
                    }
                }

                if (publish)
                {
                    publisher_->publish(twist); // 텍스트 송신
                    RCLCPP_INFO(this->get_logger(), "linear.x=%.2f, angular.z=%.2f",
                                twist.linear.x, twist.angular.z);
                }
            }
            else if (c == 's')
            {
                draw_square();
            }
            else if (c == 'c')
            {
                draw_circle();
            }
            else if (c == 't') 
            {
                draw_triangle();
            }
            else if (c == 'r')		//구현 못함
            {
                return_to_center();
            }
            else if (c == 'q') 
            {
                RCLCPP_INFO(this->get_logger(), "Shutdown");
                rclcpp::shutdown();
            }
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTurtle>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}
