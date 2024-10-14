// mainwindow.cpp

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <rclcpp/rclcpp.hpp>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QIcon icon(":/image/images/icon.png");

    // ROS2 노드 생성
    node_ = rclcpp::Node::make_shared("qt_turtle_control_node");
    publisher_ = node_->create_publisher<ros2_custom_interface::msg::Hw3>("turtle1/cmd_vel", 10); // 퍼블리셔 타입 변경

    setWindowIcon(icon);

    // QTimer 설정: 주기적으로 ROS 이벤트 처리
    connect(&ros_timer_, &QTimer::timeout, this, &MainWindow::process_ros_events);
    ros_timer_.start(10); // 10ms마다 호출
}

MainWindow::~MainWindow()
{
    // ROS2 종료는 main.cpp에서 처리
    delete ui;
}

void MainWindow::process_ros_events()
{
    rclcpp::spin_some(node_);
}

void MainWindow::on_forward_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = 2.0; // 앞으로 이동
    msg.angular_z = 0.0; // 회전 없음
    publisher_->publish(msg);
}

void MainWindow::on_backward_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = -2.0; // 뒤로 이동
    msg.angular_z = 0.0; // 회전 없음
    publisher_->publish(msg);
}

void MainWindow::on_turn_right_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = 0.0; // 이동 없음
    msg.angular_z = -1.0; // 우회전
    publisher_->publish(msg);
}

void MainWindow::on_turn_left_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = 0.0; // 이동 없음
    msg.angular_z = 1.0; // 좌회전
    publisher_->publish(msg);
}

