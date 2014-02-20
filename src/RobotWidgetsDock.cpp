#include "control_panel/RobotWidgetsDock.h"

RobotWidgetsDock::RobotWidgetsDock(QWidget *parent) :
    QDockWidget(parent)
{
}

void RobotWidgetsDock::closeEvent(QCloseEvent *event)
{
    emit visibilityChanged(false);
    event->ignore();
}
