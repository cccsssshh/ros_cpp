#include <thread>
#include <QDebug>
#include "storagy/main_window.hpp"
#include "ui_main_window.h"

MainWindow::MainWindow(std::shared_ptr<ImageSubscriberNode> image_node,
                       std::shared_ptr<BatterySubscriberNode> battery_node,
                       std::shared_ptr<RobotControlNode> robot_node,
                       QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), 
      image_node_(image_node), battery_node_(battery_node), robot_node_(robot_node)
{
    qRegisterMetaType<RobotControlNode::RobotPose>("RobotControlNode::RobotPose");

    ui->setupUi(this);
    setupConnections();

    qDebug() << "MainWindow constructor: Starting ROS threads";

    // ROS 콜백을 처리할 쓰레드 시작
    ros_image_thread_ = std::thread([this]() {
        qDebug() << "Image thread started";
        rclcpp::spin(image_node_);
    });

    ros_battery_thread_ = std::thread([this]() {
        qDebug() << "Battery thread started";
        rclcpp::spin(battery_node_);
    });

    ros_robot_thread_ = std::thread([this]() {
        qDebug() << "Robot thread started";
        rclcpp::spin(robot_node_);
    });
}

MainWindow::~MainWindow()
{
    // ROS 노드 스핀을 중지하고 쓰레드를 종료
    rclcpp::shutdown();
    if (ros_image_thread_.joinable()) {
        ros_image_thread_.join();
    }
    if (ros_battery_thread_.joinable()) {
        ros_battery_thread_.join();
    }
    if (ros_robot_thread_.joinable()) {
        ros_robot_thread_.join();
    }

    delete ui;
}

void MainWindow::setupConnections()
{
    qDebug() << "Setting up connections";

    image_node_->image_callback = [this](const cv::Mat& cv_image) {
        qDebug() << "Image callback called";
        QImage qimg(cv_image.data, cv_image.cols, cv_image.rows, cv_image.step, QImage::Format_RGB888);
        emit imageReceived(qimg.rgbSwapped());
    };

    battery_node_->battery_callback = [this](const std::string& voltage) {
        qDebug() << "Battery callback called";
        emit batteryVoltageReceived(QString::fromStdString(voltage));
    };

    robot_node_->pose_callback = [this](const RobotControlNode::RobotPose& pose) {
        qDebug() << "Robot pose callback called";
        emit robotPoseReceived(pose);
    };

    // 시그널-슬롯 연결 추가
    connect(this, &MainWindow::imageReceived, this, &MainWindow::updateImage, Qt::QueuedConnection);
    connect(this, &MainWindow::batteryVoltageReceived, this, &MainWindow::updateBatteryVoltage, Qt::QueuedConnection);
    connect(this, &MainWindow::robotPoseReceived, this, &MainWindow::updateRobotPose, Qt::QueuedConnection);

connect(ui->set_goal_btn, &QPushButton::clicked, this, &MainWindow::onSetGoalClicked);

    qDebug() << "Connections setup completed";
}


void MainWindow::updateImage(const QImage& image)
{
    qDebug() << "updateImage called. Original image size:" << image.size();

    // 레이블의 크기 가져오기
    QSize labelSize = ui->image_label->size();

    // 이미지를 레이블 크기에 맞게 조정 (비율 유지)
    QPixmap pixmap = QPixmap::fromImage(image);
    QPixmap scaledPixmap = pixmap.scaled(labelSize, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    // 조정된 이미지를 레이블에 설정
    ui->image_label->setPixmap(scaledPixmap);

    // 조정된 이미지 크기 로그 출력
    qDebug() << "Scaled image size:" << scaledPixmap.size();

    // 레이블 크기와 조정된 이미지 크기가 다를 경우 (이미지가 레이블보다 작을 때) 가운데 정렬
    ui->image_label->setAlignment(Qt::AlignCenter);
}
void MainWindow::updateBatteryVoltage(const QString& battery)
{
    qDebug() << "updateBatteryVoltage called";
    ui->battery_line->setText(battery);
}

void MainWindow::updateRobotPose(const RobotControlNode::RobotPose& pose)
{
    qDebug() << "updateRobotPose called with values:"
             << "x:" << pose.x
             << "y:" << pose.y
             << "degree:" << pose.degree
             << "theta:" << pose.theta;
    
    if (ui->coor_x_line && ui->coor_y_line && ui->coor_degree_line && ui->coor_theta_line) {
        ui->coor_x_line->setText(QString::number(pose.x, 'f', 6));
        ui->coor_y_line->setText(QString::number(pose.y, 'f', 6));
        ui->coor_degree_line->setText(QString::number(pose.degree, 'f', 6));
        ui->coor_theta_line->setText(QString::number(pose.theta, 'f', 6));
    } else {
        qDebug() << "One or more UI elements are null";
    }
}

void MainWindow::onSetGoalClicked()
{
    qDebug() << "Set Goal button clicked";

    bool ok;
    double x = ui->goal_x_line->text().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid x coordinate";
        return;
    }

    double y = ui->goal_y_line->text().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid y coordinate";
        return;
    }

    double degree = ui->goal_degree_line->text().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid degree";
        return;
    }

    // 각도를 라디안으로 변환
    double theta = degree * M_PI / 180.0;

    qDebug() << "Navigating to goal: x =" << x << ", y =" << y << ", theta =" << theta;

    // RobotControlNode의 navigateToPose 함수 호출
    robot_node_->navigateToPose(x, y, theta);
}