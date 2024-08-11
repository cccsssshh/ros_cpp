#include <QApplication>
#include <QTimer>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "test/main_window.hpp"
#include "test/image_subscriber_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<ImageSubscriberNode>();

    MainWindow w(node);
    w.show();

    // ROS2 스핀을 위한 별도의 스레드 생성
    std::thread ros_spin_thread([&node]() {
        rclcpp::spin(node);
    });

    // Qt 이벤트 루프 실행
    int result = app.exec();

    // 애플리케이션 종료 시 정리
    rclcpp::shutdown();
    if (ros_spin_thread.joinable()) {
        ros_spin_thread.join();
    }

    return result;
}