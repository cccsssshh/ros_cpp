#include "storagy/main_window.hpp"
#include "ui_main_window.h"
#include <QImage>

MainWindow::MainWindow(std::shared_ptr<ImageSubscriberNode> node, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow), node_(node)
{
    ui->setupUi(this);

    node_->image_callback = [this](const cv::Mat& image) {
        QImage qimage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888);
        QMetaObject::invokeMethod(this, "updateImage", Qt::QueuedConnection, Q_ARG(QImage, qimage.rgbSwapped()));
    };
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateImage(const QImage& image)
{
    ui->image_label->setPixmap(QPixmap::fromImage(image).scaled(ui->image_label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}