// mainwindow.cpp

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <rclcpp/rclcpp.hpp>

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QIcon icon(":/image/images/icon.png");

    node_ = rclcpp::Node::make_shared("qt_turtle_control_node");
    publisher_ = node_->create_publisher<ros2_custom_interface::msg::Hw3>("turtle1/cmd_vel", 10); 

    setWindowIcon(icon);

    connect(&ros_timer_, &QTimer::timeout, this, &MainWindow::process_ros_events);
    ros_timer_.start(10); 
}

MainWindow::~MainWindow()
{

    delete ui;
}

void MainWindow::process_ros_events()
{
    rclcpp::spin_some(node_);
}

void MainWindow::on_forward_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = 2.0; 
    msg.angular_z = 0.0;
    publisher_->publish(msg);
}

void MainWindow::on_backward_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = -2.0; 
    msg.angular_z = 0.0;
    publisher_->publish(msg);
}

void MainWindow::on_turn_right_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = 0.0; 
    msg.angular_z = -1.0; 
    publisher_->publish(msg);
}

void MainWindow::on_turn_left_btn_clicked()
{
    auto msg = ros2_custom_interface::msg::Hw3();
    msg.linear_x = 0.0;
    msg.angular_z = 1.0;
    publisher_->publish(msg);
}

