#ifndef CONTROLPANEL_H
#define CONTROLPANEL_H

#include <QMainWindow>
#include <QLabel>
#include <QShortcut>
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
    void openRobot(const QString &name);
    void closeRobotTab(int index);
    void updateMasterStatus();
    void updatePluginList(const QStringList &list);
    void unloadNodeletPlugin(const QString name, const QString instance);
    void closeCurrentTab();
    void nextTab();
    void previousTab();
    
private:
    void closeEvent(QCloseEvent *event);

    Ui::ControlPanel *ui;
    QLabel ms;
    QThread ros_thread;
    QTimer *master_check;
    QShortcut shortcut_close_tab;
    QShortcut shortcut_next_tab;
    QShortcut shortcut_previous_tab;
};

#endif // CONTROLPANEL_H
