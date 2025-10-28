#include "app.hpp"
#include <QApplication>
#include <QSurfaceFormat>
#include <QDebug>

int App::run(int argc, char** argv) {
    QApplication app(argc, argv);

    QApplication::setApplicationName("Bridge Manual Override");
    QApplication::setOrganizationName("BridgeControlSystem");
    QApplication::setStyle("Fusion");

    QSurfaceFormat fmt;
    fmt.setSamples(4);
    QSurfaceFormat::setDefaultFormat(fmt);

    MainWindow window;
    window.setWindowTitle("Bridge Manual Override System");
    window.resize(600, 400);
    window.show();

    qDebug() << "we in lad.";
    return app.exec();
}

