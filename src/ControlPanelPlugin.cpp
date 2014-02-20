#include "control_panel/ControlPanelPlugin.h"

namespace control_panel
{
ControlPanelPlugin::ControlPanelPlugin() :
    QWidget(0),
    name(""),
    instance(""),
    settings(NULL)
{
}

ControlPanelPlugin::~ControlPanelPlugin()
{
    emit unload(name, instance);
}

void ControlPanelPlugin::setup()
{

}

void ControlPanelPlugin::start()
{

}

void ControlPanelPlugin::stop()
{

}

boost::shared_ptr<nodelet::Nodelet> ControlPanelPlugin::getNodelet()
{
    return boost::shared_ptr<nodelet::Nodelet>((nodelet::Nodelet *)NULL);
}

void ControlPanelPlugin::delete_self()
{
    settings->remove(uuid.toString());
    // Clean up our config
    emit deleteLater();
}

void ControlPanelPlugin::keyDownCB(QKeyEvent *event)
{
    std::cerr << "Key-down event was not overridden correctly" << std::endl;
}

void ControlPanelPlugin::keyUpCB(QKeyEvent *event)
{
    std::cerr << "Key-up event was not overridden correctly" << std::endl;
}

}
