#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H

#include <QObject>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <nodelet/detail/callback_queue.h>
#include <nodelet/detail/callback_queue_manager.h>
#include <boost/ptr_container/ptr_map.hpp>
#include <pluginlib/class_loader.h>

#include "control_panel/ControlPanelPlugin.h"

class ROSInterface : public QObject
{
    Q_OBJECT
public:
    explicit ROSInterface(QObject *parent = 0);
    ~ROSInterface();
    control_panel::ControlPanelPlugin * createNewPluginInstance(const QString &name, const QString &ns);
    
signals:
    void masterStatusMsg(QString);
    void newPluginList(const QStringList &);
    void openRobot(const QString &);

public slots:
    void process();
    void shutdown();
    void ping_master();
    bool load_nodelet(const QString name, boost::shared_ptr<nodelet::Nodelet> nodelet);
    void unload_nodelet(QString name);
    void refresh_plugins();
    void unloadPlugin(const QString &name);

private:
    ros::NodeHandle *nh;
    ros::NodeHandle *nh_priv;
    nodelet::detail::CallbackQueueManager *cqm;
    pluginlib::ClassLoader<control_panel::ControlPanelPlugin> *plugin_loader;

    class MyManagedNodelet : boost::noncopyable
    {
    public:
        MyManagedNodelet(const std::string name, boost::shared_ptr<nodelet::Nodelet> &_nodelet, nodelet::detail::CallbackQueueManager* cqm);
        ~MyManagedNodelet();
    private:
        boost::shared_ptr<nodelet::Nodelet> nodelet;
        nodelet::detail::CallbackQueuePtr st_queue;
        nodelet::detail::CallbackQueuePtr mt_queue;
        nodelet::detail::CallbackQueueManager* callback_manager;
    };

    boost::ptr_map<const std::string, MyManagedNodelet> nodelets;
    std::map<const std::string, const std::string[3]> plugins;
    std::map<const std::string, int> plugin_instances;
};

#endif // ROSINTERFACE_H
