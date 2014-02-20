#include "control_panel/RobotTab.h"

#include <QPushButton>
#include <QDialog>
#include <QLineEdit>
#include <QLabel>
#include <QFileInfo>
#include <QDir>
#include <QMessageBox>
#include <QUuid>

RobotTab::RobotTab(QWidget *parent, ControlPanel *_cp, const QString &name) :
    QWidget(parent),
    cp(_cp),
    rws(NULL),
    cfgbtn("Configure Robot"),
    settings( cp->settings->value("default_robot_dir").toString() + "/" + name + ".ini", QSettings::IniFormat)
{
    if(!settings.contains("name"))
        settings.setValue("name", name);
    if(!settings.contains("namespace"))
        settings.setValue("namespace", QString(name).replace(QRegExp(" "), "_").toLower().remove(QRegExp("[^a-zA-Z\\d_]")));
    if(!settings.contains("root_ws"))
        settings.setValue("root_ws", QUuid::createUuid().toByteArray());

    //std::cout << "Settings File:" << settings.fileName().toStdString() << std::endl;

    rws = new RobotWorkspace(this, QUuid(settings.value("root_ws").toByteArray()), this);
    connect(&cfgbtn, SIGNAL(clicked()), this, SLOT(configDialog()));
    my_layout.addWidget(&cfgbtn, 0, 0, Qt::AlignLeft);
    my_layout.addWidget(rws, 1, 0);
    setLayout(&my_layout);
    setFocusPolicy(Qt::StrongFocus);
}

QString RobotTab::getName()
{
    return settings.value("name").toString();
}

void RobotTab::configDialog()
{
    QDialog dialog;
    QGridLayout *layout = new QGridLayout;

    QLabel *nametxt = new QLabel(tr("Display Name:"));
    QLineEdit *nameedit = new QLineEdit(settings.value("name").toString());
    layout->addWidget(nametxt, 0, 0);
    layout->addWidget(nameedit, 0, 1);

    QLabel *nstxt = new QLabel(tr("Namespace:"));
    QLineEdit *nsedit = new QLineEdit(settings.value("namespace").toString());
    layout->addWidget(nstxt, 1, 0);
    layout->addWidget(nsedit, 1, 1);

    QPushButton *okbutton = new QPushButton(tr("&OK"));
    layout->addWidget(okbutton, 2, 1);

    dialog.setLayout(layout);

    dialog.setWindowTitle("Robot Configuration");

    connect(okbutton, SIGNAL(clicked()), &dialog, SLOT(accept()));

    if(!dialog.exec())
        return;

    if(cp->getTabName(this) != nameedit->text())
    {
        settings.setValue("name", nameedit->text());
        cp->setTabName(this, nameedit->text());
    }
    if(settings.value("namespace").toString() != nsedit->text())
    {
        settings.setValue("namespace", nsedit->text());
        if(QMessageBox::question(this, tr("Restart Robot"), tr("The namespace has been saved, however, changing the active namespace requires reconnecting to the robot. Proceed with reconnection?"), QMessageBox::Yes, QMessageBox::No, QMessageBox::NoButton) == QMessageBox::Yes)
        {
            delete rws;
            rws = new RobotWorkspace(this, QUuid(settings.value("root_ws").toByteArray()), this);
            my_layout.addWidget(rws, 1, 0);
            setLayout(&my_layout);
        }
    }
}

void RobotTab::keyPressEvent(QKeyEvent *event)
{
    if(event->isAutoRepeat())
        return;
    emit keyDownEvent(event);
}

void RobotTab::keyReleaseEvent(QKeyEvent *event)
{
    if(event->isAutoRepeat())
        return;
    emit keyUpEvent(event);
}

void RobotTab::closeEvent(QCloseEvent *event)
{
}

void RobotTab::saveConfig()
{
}

void RobotTab::loadConfig(const QSettings &_settings)
{
}

void RobotTab::setKeyCB(control_panel::ControlPanelPlugin *cpp, bool enabled)
{
    if(enabled)
    {
        connect(this, SIGNAL(keyDownEvent(QKeyEvent *)), cpp, SLOT(keyDownCB(QKeyEvent *)));
        connect(this, SIGNAL(keyUpEvent(QKeyEvent *)), cpp, SLOT(keyUpCB(QKeyEvent *)));
    }
    else
    {
        disconnect(this, SIGNAL(keyDownEvent(QKeyEvent *)), cpp, SLOT(keyDownCB(QKeyEvent *)));
        disconnect(this, SIGNAL(keyUpEvent(QKeyEvent *)), cpp, SLOT(keyUpCB(QKeyEvent *)));
    }

}
