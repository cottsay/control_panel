#ifndef ROBOTTAB_H
#define ROBOTTAB_H

#include "control_panel/ControlPanel.h"
#include "control_panel/RobotWorkspace.h"

#include <QWidget>
#include <QGridLayout>
#include <QPushButton>
#include <QUuid>

class RobotWorkspace;

class RobotTab : public QWidget
{
    Q_OBJECT
public:
    explicit RobotTab(QWidget *parent = 0, ControlPanel *_cp = NULL, const QString &name = "New Robot");
    QString getName();
    
signals:
    void keyDownEvent(QKeyEvent *event);
    void keyUpEvent(QKeyEvent *event);
    
public slots:
    void configDialog();
    void saveConfig();
    void loadConfig(const QSettings &_settings);
    void setKeyCB(control_panel::ControlPanelPlugin *cpp, bool enabled);

protected:
    void keyPressEvent(QKeyEvent *event);
    void keyReleaseEvent(QKeyEvent *event);

private:
    void closeEvent(QCloseEvent *event);
    ControlPanel *cp;
    QGridLayout my_layout;
    RobotWorkspace *rws;
    QPushButton cfgbtn;
    QSettings settings;

friend class RobotWorkspace;
};

#endif // ROBOTTAB_H
