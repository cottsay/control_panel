#ifndef ROBOTWIDGETSDOCK_H
#define ROBOTWIDGETSDOCK_H

#include <QDockWidget>
#include <QCloseEvent>

class RobotWidgetsDock : public QDockWidget
{
    Q_OBJECT
public:
    explicit RobotWidgetsDock(QWidget *parent = 0);

protected:
    void closeEvent(QCloseEvent *event);

signals:
    
public slots:
    
};

#endif // ROBOTWIDGETSDOCK_H
