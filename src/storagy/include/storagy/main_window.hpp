#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QImage>
#include "storagy/image_sub_node.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(std::shared_ptr<ImageSubscriberNode> node, QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void updateImage(const QImage& image);

private:
    Ui::MainWindow *ui;
    std::shared_ptr<ImageSubscriberNode> node_;
};

#endif // MAIN_WINDOW_HPP