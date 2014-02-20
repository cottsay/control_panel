#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QMainWindow>
#include <QLabel>
#include <QThread>
#include <QTimer>
#include <QSettings>

#include "control_panel/ROSInterface.h"

namespace Ui {
class ControlPanel;
}

class ControlPanel : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ControlPanel(QWidget *parent = 0);
    ~ControlPanel();
    ROSInterface ros_interface;
    QString getTabName(QWidget *w);
    void setTabName(QWidget *w, const QString &name);
    QSettings *settings;

signals:

public slots:
    void toggleRobotWidgetsDock(bool vis);
    void triggerNewBlankRobot();
    void closeRobotTab(int index);
    void updateMasterStatus();
    void updatePluginList(const QStringList &list);
    void unloadNodeletPlugin(const QString name, const QString instance);
    
private:
    void closeEvent(QCloseEvent *event);

    Ui::ControlPanel *ui;
    QLabel ms;
    QThread ros_thread;
    QTimer *master_check;
};

#endif // CONTROLPANEL_H
