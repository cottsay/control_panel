#include "control_panel/ROSInterface.h"

#include <QCoreApplication>

ROSInterface::MyManagedNodelet::MyManagedNodelet(const std::string name, boost::shared_ptr<nodelet::Nodelet> &_nodelet, nodelet::detail::CallbackQueueManager* cqm)
    : nodelet(_nodelet)
    , st_queue(new nodelet::detail::CallbackQueue(cqm, nodelet))
    , mt_queue(new nodelet::detail::CallbackQueue(cqm, nodelet))
    , callback_manager(cqm)
{
    callback_manager->addQueue(st_queue, false);
    callback_manager->addQueue(mt_queue, true);

    const ros::M_string remappings;
    const std::vector<std::string> my_argv;
    nodelet->init(name, remappings, my_argv, st_queue.get(), mt_queue.get());
}

ROSInterface::MyManagedNodelet::~MyManagedNodelet()
{
    callback_manager->removeQueue(st_queue);
    callback_manager->removeQueue(mt_queue);
    nodelet.reset();
}

ROSInterface::ROSInterface(QObject *parent) :
    QObject(parent),
    nh(NULL),
    nh_priv(NULL),
    cqm(NULL),
    plugin_loader(NULL)
{
}

ROSInterface::~ROSInterface()
{
    shutdown();
}

void ROSInterface::process()
{
    int argc = QCoreApplication::argc();
    char **argv = QCoreApplication::argv();

    if(!ros::ok())
        ros::init(argc, argv, "control_panel");

    ROS_DEBUG("Starting control_panel ROS interface");

    if(!nh_priv)
        nh_priv = new ros::NodeHandle("~");
    if(!nh)
        nh = new ros::NodeHandle;
    if(!cqm)
        cqm = new nodelet::detail::CallbackQueueManager;
    if(!plugin_loader)
    {
        ROS_DEBUG("Starting control_panel plugin interface");
        try
        {
            plugin_loader = new pluginlib::ClassLoader<control_panel::ControlPanelPlugin>("control_panel", "control_panel::ControlPanelPlugin");
            refresh_plugins();
            ROS_WARN_COND(!plugins.size(), "No Control Panel plugins found!");
        }
        catch(pluginlib::PluginlibException &ex)
        {
            ROS_WARN("No Control Panel plugins found or pluginlib failure");
            plugin_loader = NULL;
        }
    }

    if(!nh || !nh_priv || !cqm || !ros::ok())
        return;

    ROS_DEBUG("control_panel ROS interface startup complete");

    for(int i = 1; i < argc; i++)
        emit openRobot(argv[i]);

    ros::spin();

    QCoreApplication::exit(0);
}

void ROSInterface::refresh_plugins()
{
    if(!plugin_loader)
        return;

    ROS_DEBUG("Refreshing plugin list");

    std::vector<std::string> pl_list = plugin_loader->getDeclaredClasses();
    QStringList n_list;

    for(unsigned int i = 0; i < pl_list.size( ); i++)
    {
        const std::string desc = plugin_loader->getClassDescription(pl_list[i]);
        const size_t br1 = desc.find('#');
        const size_t br2 = desc.find('#', br1 + 1);
        const std::string dset[3] = { pl_list[i], desc.substr(br1 + 1, br2 - br1 - 1), desc.substr(br2 + 1) };
        plugins.insert(std::pair<const std::string, const std::string[3]>(desc.substr(0, br1), dset));
        n_list += desc.substr(0, br1).c_str();
    }

    emit newPluginList(n_list);
}

void ROSInterface::shutdown()
{
    ROS_DEBUG("control_panel ROS interface shutdown requested");

    ros::shutdown();

    nodelets.clear();

    delete plugin_loader;
    plugin_loader = NULL;
    delete cqm;
    cqm = NULL;
    delete nh;
    nh = NULL;
    delete nh_priv;
    nh_priv = NULL;
}

void ROSInterface::ping_master()
{
    ROS_DEBUG("Pinging master");

    QString msg;
    if(ros::master::check())
        msg = "ROS Master: OK";
    else
        msg = "ROS Master: FAIL";
    emit masterStatusMsg(msg);
}

bool ROSInterface::load_nodelet(const QString name, boost::shared_ptr<nodelet::Nodelet> nodelet)
{
    ROS_DEBUG_STREAM("Loading nodelet " << name.toStdString() );
    if(!nodelet)
    {
        ROS_ERROR_STREAM("Failed to load nodelet '" << name.toStdString() << "' because the nodelet pointer is NULL!");
        return false;
    }
    MyManagedNodelet *mn = new MyManagedNodelet(name.toStdString(), nodelet, cqm);
    if(!mn)
    {
        ROS_ERROR_STREAM("Failed to load nodelet '" << name.toStdString() << "' because the managed nodelet is NULL!");
        return false;
    }
    nodelets.insert(name.toStdString(), mn);
    return true;
}

void ROSInterface::unload_nodelet(QString name)
{
    ROS_DEBUG_STREAM("Unloading nodelet " << name.toStdString() );
    if(name.length())
        nodelets.erase(name.toStdString());
}

control_panel::ControlPanelPlugin * ROSInterface::createNewPluginInstance(const QString &name, const QString &ns)
{
    ROS_DEBUG_STREAM("Loading plugin " << name.toStdString() );
    try
    {
        if(!plugin_loader)
        {
            ROS_ERROR_STREAM("Failed to load plugin '" << name.toStdString() << "'' because the plugin loader is not valid!");
            return NULL;
        }
        control_panel::ControlPanelPlugin *plug = plugin_loader->createUnmanagedInstance(plugins.at(name.toStdString())[0]);
        if(!plug)
        {
            ROS_ERROR_STREAM("Failed to load plugin '" << name.toStdString() << "'' because the plugin loader returned a NULL instance!");
            return NULL;
        }
        plug->name = ns + "/control_panel_" + QString(name).replace(QRegExp(" "), "_").toLower().remove(QRegExp("[^a-zA-Z\\d_]")) + '_' + QString::number((long)plug);
        plug->instance = plugins.at(name.toStdString())[0].c_str();
        if(!load_nodelet(plug->name, plug->getNodelet()))
        {
            ROS_ERROR_STREAM("Failed to load plugin '" << name.toStdString() << "'' because of previous errors!");
            return NULL;
        }
        if(plugin_instances.find(plug->instance.toStdString()) == plugin_instances.end())
        {
            plugin_instances.insert(std::pair<const std::string, int>(plug->instance.toStdString(), 1));
        }
        else
        {
            plugin_instances[plug->instance.toStdString()]++;
        }
        return plug;
    }
    catch(pluginlib::PluginlibException &ex)
    {
        ROS_ERROR_STREAM("Failed to load plugin '" << name.toStdString() << "': " << ex.what());
        return NULL;
    }
}

void ROSInterface::unloadPlugin(const QString &name)
{
    ROS_DEBUG_STREAM("Unloading plugin " << name.toStdString() );
    if(plugin_instances.find(name.toStdString()) != plugin_instances.end() && --plugin_instances[name.toStdString()] > 0)
    {
        ROS_DEBUG_STREAM("Not unloading library because there are still " << plugin_instances[name.toStdString()] << " instances.");
        return;
    }
    try
    {
        if(!plugin_loader)
        {
            ROS_ERROR_STREAM("Failed to unload nodelet\"" << name.toStdString() << "\" because the plugin loader is not valid!");
            return;
        }
        if(!plugin_loader->isClassLoaded(name.toStdString()))
        {
            ROS_ERROR_STREAM("Failed to unload nodelet\"" << name.toStdString() << "\" because it is not loaded!");
            return;
        }
        int num_left = plugin_loader->unloadLibraryForClass(name.toStdString());
        ROS_DEBUG_STREAM("Unload successful. " << num_left << " plugins to go.");
    }
    catch(pluginlib::PluginlibException &ex)
    {
        ROS_ERROR_STREAM("Failed to unload plugin '" << name.toStdString() << "': " << ex.what());
    }
}
