#ifndef CONTROLPANELPLUGIN_H
#define CONTROLPANELPLUGIN_H

#include <nodelet/nodelet.h>
#include <QWidget>
#include <QUuid>
#include <QSettings>

namespace control_panel
{
class ControlPanelPlugin : public QWidget
{
    Q_OBJECT

public:
    virtual void setup();
    virtual void start();
    virtual void stop();
    virtual boost::shared_ptr<nodelet::Nodelet> getNodelet();
    QString name;
    QString instance;
    QUuid uuid;
    QSettings *settings;
    ~ControlPanelPlugin();
protected:
    ControlPanelPlugin();
signals:
    void unload(const QString, const QString);
    void unload_nodelet(const QString);
    void setKeyCB(control_panel::ControlPanelPlugin *cpp, bool enable);
    void setMouseCB(control_panel::ControlPanelPlugin *cpp, bool enable);
public slots:
    void delete_self();
    virtual void keyDownCB(QKeyEvent *event);
    virtual void keyUpCB(QKeyEvent *event);
};
}

#endif // CONTROLPANELPLUGIN_H
