#include "mainwindow.h"

#include <QApplication>

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    QApplication::setApplicationName("RSLiDAR E1R People Detection");
    QApplication::setOrganizationName("RSLiDAR Demo");

    MainWindow window;
    window.show();
    return app.exec();
}
