#include <QtGui/QApplication>
#include "control_panel/ControlPanel.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ControlPanel w;

    // Make sure ROS shuts down correctly
    //QObject::connect(&a, SIGNAL(aboutToQuit()), &w.ros_interface, SLOT(shutdown()));

    w.show();

    return a.exec();
}
