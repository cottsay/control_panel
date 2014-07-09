#include "control_panel/ControlPanel.h"
#include "control_panel/RobotWorkspace.h"
#include "control_panel/RobotTab.h"
#include "ui_ControlPanel.h"

#include <QMessageBox>
#include <QFileInfo>
#include <QDir>
#include <QUuid>

ControlPanel::ControlPanel(QWidget *parent) :
    settings(NULL),
    QMainWindow(parent),
    ui(new Ui::ControlPanel),
    master_check(0)
{
    // Register Metatypes
    qRegisterMetaTypeStreamOperators<QList<QUuid> >("QList<QUuid>");

    // Setup Settings Interface
    QCoreApplication::setOrganizationName("SDSMT CSR");
    QCoreApplication::setOrganizationDomain("csr.sdsmt.edu");
    QCoreApplication::setApplicationName("SRS Control Panel");
    settings = new QSettings;

    // Restore Settings
    if(!settings->contains("default_robot_dir"))
        settings->setValue("default_robot_dir", QFileInfo(settings->fileName()).dir().path() + "/robots");

    // Setup the UI
    ui->setupUi(this);

    // Restore Window Geometry
    restoreGeometry(settings->value("geometry",
        QByteArray::fromHex("01d9d0cb000100000000004d000000590000023a0000021000000057000000810000023000000206000000000000")
        ).toByteArray());

    // Robot Widgets
    ui->robotWidgetDock->hide();
    ui->robotWidgetList->setDragDropMode(QAbstractItemView::DragDrop);
    ui->robotWidgetList->setAcceptDrops(false);

    // ROS Master Status
    ms.setText("ROS Master: ??");
    ui->statusBar->addPermanentWidget(&ms);

    // ROS Interface
    ros_interface.moveToThread(&ros_thread);
    connect(&ros_thread, SIGNAL(started()), &ros_interface, SLOT(process()));
    connect(&ros_interface, SIGNAL(masterStatusMsg(QString)), &ms, SLOT(setText(QString)));
    connect(&ros_interface, SIGNAL(newPluginList(QStringList)), this, SLOT(updatePluginList(QStringList)));
    connect(&ros_interface, SIGNAL(openRobot(const QString &)), this, SLOT(openRobot(const QString &)));
    ros_thread.start();

    // ROS Master Check
    master_check = new QTimer(this);
    connect(master_check, SIGNAL(timeout()), this, SLOT(updateMasterStatus()));
    master_check->start(1000);
}

ControlPanel::~ControlPanel()
{
    delete ui->robotTabs;
    ros_interface.shutdown();
    ros_thread.quit();
    ros_thread.wait();
    delete settings;
    delete ui;
}

void ControlPanel::updateMasterStatus()
{
    ros_interface.ping_master();
}

void ControlPanel::toggleRobotWidgetsDock(bool vis)
{
    ui->actionRobot_Widgets->setChecked(vis);
    ui->robotWidgetDock->setVisible(vis);
}

void ControlPanel::triggerNewBlankRobot()
{
    openRobot(ui->newBlankNameEdit->text());
}

void ControlPanel::openRobot(const QString &name)
{
    if(name.length() < 1 || name == "Home" || name == "home")
    {
        QMessageBox msg;
        msg.setWindowTitle("Error");
        msg.setText("Invalid robot name");
        msg.exec();
        return;
    }

    for(int i = 0; i < ui->robotTabs->count(); i++)
    {
        if(name == ui->robotTabs->tabText(i))
        {
            QMessageBox msg;
            msg.setWindowTitle("Error");
            msg.setText("A robot by that name already exists");
            msg.exec();
            return;
        }
    }

    RobotTab *rt = new RobotTab(ui->robotTabs, this, name);
    ui->robotTabs->setCurrentIndex(ui->robotTabs->addTab(rt, rt->getName()));
}

void ControlPanel::closeRobotTab(int index)
{
    if(ui->robotTabs->tabText(index) == "Home")
        ui->robotTabs->setCurrentIndex(index);
    else
        ui->robotTabs->widget(index)->deleteLater();
}

void ControlPanel::updatePluginList(const QStringList &list)
{
    ui->robotWidgetList->clear();
    ui->robotWidgetList->addItems(list);
}

void ControlPanel::unloadNodeletPlugin(const QString name, const QString instance)
{
    ros_interface.unload_nodelet(name);
    // TODO: Re-Enable This!
    /*if(!name.contains("control_panel_angel_controller"))
        ros_interface.unloadPlugin(instance);
    else
        std::cerr << "WARN: control_panel_angel_controller workaround" << std::endl;*/
}

QString ControlPanel::getTabName(QWidget *w)
{
    return ui->robotTabs->tabText(ui->robotTabs->indexOf(w));
}

void ControlPanel::setTabName(QWidget *w, const QString &name)
{
    ui->robotTabs->setTabText(ui->robotTabs->indexOf(w), name);
}

void ControlPanel::closeEvent(QCloseEvent *event)
{
    settings->setValue("geometry", saveGeometry());
    QMainWindow::closeEvent(event);
}
