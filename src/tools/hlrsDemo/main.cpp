#include "flowlayout.h"
#include "verticallabel.h"
#include "demowindow.h"

#include <QApplication>
#include <QFile>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

    QString demosPath = HLRS_DEMO_PATH + "/src/myDemos.json";
    if (!QFile::exists(demosPath))
        demosPath = HLRS_DEMO_PATH + "/src/demos.json";

    std::cerr << "Reading demos from: " << demosPath.toStdString() << std::endl;
    nlohmann::json demos = readDemosJson(demosPath);

    DemoWindow window(demos);
    window.setWindowTitle("HLRS Demo Launcher");
    window.resize(900, 700);
    window.show();
    return app.exec();
}