#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "ros2_custom_interface/msg/hw3.hpp"
#include "rclcpp/rclcpp.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void on_forward_btn_clicked();
    void on_backward_btn_clicked();
    void on_turn_right_btn_clicked();
    void on_turn_left_btn_clicked();

private:
    Ui::MainWindow* ui;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<ros2_custom_interface::msg::Hw3>::SharedPtr publisher_;
};

#endif // MAINWINDOW_H
