#ifndef ROBOTWORKSPACE_H
#define ROBOTWORKSPACE_H

#include "control_panel/ROSInterface.h"
#include "control_panel/RobotTab.h"

#include <QSplitter>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QChildEvent>
#include <QUuid>

class RobotTab;

class RobotWorkspace : public QSplitter
{
    Q_OBJECT
public:
    explicit RobotWorkspace(QWidget *parent = 0, const QUuid _uuid = QUuid::createUuid(), RobotTab *_rt = NULL);
    ~RobotWorkspace();
    static unsigned char getQuad(const QPoint &p, const int &w, const int &h);
    
protected:
    void dragEnterEvent(QDragEnterEvent *event);
    void dragLeaveEvent(QDragLeaveEvent *event);
    void dropEvent(QDropEvent *event);
    control_panel::ControlPanelPlugin * getNewWidget(const QString &wn);
    bool addNewWidget(const QPoint &p, const QString &wn);
    QUuid uuid;

signals:
    
public slots:
    void checkChildren();
    void layoutUpdate();
    void childrenUpdate();
    
private:
    RobotTab *rt;

friend class RobotTab;
};

#endif // ROBOTWORKSPACE_H
